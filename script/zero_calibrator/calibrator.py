import os
import json
import yaml
import numpy as np
from scipy.linalg import pinv
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import subprocess
import pinocchio
import time
from scipy.spatial.transform import Rotation as R
from IPython import embed

def _pulse_to_degree(pulse):
    # 电机脉冲转角度
    degree = ((pulse - 262144) / 262144) * 360.0
    return degree

def _to_complement_hex(value, bits=32):
    # 若是负数，获取补码表示
    if value < 0:
        # 计算补码： 2^bits + value
        value = (1 << bits) + value
    # 将整数转换为16进制字符串
    hex_value = hex(value)
    return hex_value

def _get_motor_index_and_check_status() -> list:
    import re
    try:
        result = subprocess.run(['ethercat', 'sl'], capture_output=True, text=True, check=True)
    except Exception as e:
        raise RuntimeError(f'执行 "ethercat sl" 失败: {e}')

    driver_positions = []
    non_preop = []  # (index, state)

    for line in result.stdout.splitlines():
        if 'ZeroErr Driver' in line:
            m = re.match(r'^\s*(\d+)\s+\S+\s+([A-Z]+)\b.*ZeroErr Driver', line)
            if m:
                idx = int(m.group(1))
                state = m.group(2)
                driver_positions.append(idx)
                if state != 'PREOP':
                    non_preop.append((idx, state))
            else:
                m2 = re.match(r'^\s*(\d+)\b', line)
                if m2:
                    idx = int(m2.group(1))
                    driver_positions.append(idx)
                    non_preop.append((idx, 'UNKNOWN'))

    if len(driver_positions) != 20:
        raise RuntimeError(f'期望 20 个 ZeroErr Driver，实际为 {len(driver_positions)}: {driver_positions}')

    if non_preop:
        details = ', '.join(f'{idx}:{state}' for idx, state in non_preop)
        raise RuntimeError(f'以下 ZeroErr Driver 非 PREOP 状态: {details}')

    print(f"Motor index: [{', '.join(map(str, driver_positions))}], total {len(driver_positions)} motors, all in PREOP state.")
    # 返回 ZeroErr Driver 在 ethercat 从站表中的索引列表（按出现顺序）
    return driver_positions

def _get_motor_position_deg_by_index(motor_index_list: list) -> list:
    # 通过 motor_index 获取电机的当前角度（单位：度）
    qpos_list = []
    for motor_index in motor_index_list:
        result = subprocess.run(['ethercat', 'upload', '-p', str(motor_index), '0x6064', '0'], capture_output=True, text=True)
        stdout = result.stdout
        pulse = int(stdout.split(' ')[1])
        qpos_list.append(_pulse_to_degree(pulse))
    return qpos_list

def _write_zero_offset_error(is_left: bool, index_list: list, offset_error_list: list):
    # 仅处理对应手臂的关节索引：左臂 6..12，右臂 13..19
    target_joint_ids = list(range(6, 13)) if is_left else list(range(13, 20))
    if len(offset_error_list) != len(target_joint_ids):
        raise ValueError(f"offset_error_list 长度应为 {len(target_joint_ids)}，当前为 {len(offset_error_list)}")
    pre_offset_list = []
    new_offset_list = []

    for i_local, joint_id in enumerate(target_joint_ids):
        # joint_id 是 0..19 的关节序号，index_list[joint_id] 为对应 EtherCAT 从站索引（-p 参数）
        index = index_list[joint_id]
        offset_error = offset_error_list[i_local]

        # 读取原本的零位偏差值
        result = subprocess.run(['ethercat', 'upload', '-p', str(index), '0x3b75', '0'], capture_output=True, text=True)
        pre_offset = int(result.stdout.split(' ')[1])
        pre_offset_list.append(pre_offset)

        # 计算新的零位偏差值
        new_offset = int(pre_offset + offset_error / 360.0 * 262144)
        new_offset_list.append(new_offset)

        # 写入新的零位偏差值
        result = subprocess.run(['ethercat', 'download', '-p', str(index), '0x3b75', '0', _to_complement_hex(new_offset)], capture_output=True, text=True)

        # 保存写入值（已解锁保存区）
        result = subprocess.run(['ethercat', 'download', '-p', str(index), '0x1010', '1', '0x65766173'], capture_output=True, text=True)

    print(f"pre_offset:", pre_offset_list)
    print(f"new_offset:", new_offset_list)
    time.sleep(2)
    print("Zero offsets updated!")

def set_zero_offset(is_left: bool, zero_offsets_deg: list) -> bool:
    motor_index_list = _get_motor_index_and_check_status()
    qpos_list = _get_motor_position_deg_by_index(motor_index_list)
    print("Current motor positions (deg) before calibration:", qpos_list)
    _write_zero_offset_error(is_left, motor_index_list, zero_offsets_deg)
    qpos_list = _get_motor_position_deg_by_index(motor_index_list)
    print("Current motor positions (deg) after calibration:", qpos_list)

def _parse_tcp_frame(tcp_raw):
    """
    支持多种 tcp_frame 格式：
    - 4x4 矩阵（嵌套列表或 ndarray）
    - 长度为16的一维列表（按行主序）
    - 长度为7的一维列表/数组：[px,py,pz, qw,qx,qy,qz]（注意四元数是 w,x,y,z）
    - dict 包含 {"position":[x,y,z], "orientation":[w,x,y,z]} 四元数 (w,x,y,z)
    返回 4x4 numpy 矩阵
    """
    if tcp_raw is None:
        return np.eye(4, dtype=float)
    tcp_arr = np.asarray(tcp_raw)
    if tcp_arr.shape == (4, 4):
        return tcp_arr.astype(float)
    if tcp_arr.size == 16:
        return tcp_arr.reshape((4, 4)).astype(float)
    # 支持 pos(3) + quat(4) 的扁平数组，四元数顺序为 w,x,y,z
    if tcp_arr.size == 7:
        arr = tcp_arr.flatten().astype(float)
        pos = arr[0:3]
        ori = arr[3:7]  # [w, x, y, z]
        if ori.size != 4:
            raise ValueError("orientation must be length-4 quaternion [w,x,y,z]")
        quat_xyzw = np.array([ori[1], ori[2], ori[3], ori[0]], dtype=float)  # reorder to x,y,z,w
        rot = R.from_quat(quat_xyzw)
        T = np.eye(4, dtype=float)
        T[:3, :3] = rot.as_matrix()
        T[:3, 3] = pos
        return T
    if isinstance(tcp_raw, dict):
        pos = np.asarray(tcp_raw.get("position", [0, 0, 0]), dtype=float)
        ori = np.asarray(tcp_raw.get("orientation", [1, 0, 0, 0]), dtype=float)  # quaternion w,x,y,z
        if ori.size != 4:
            raise ValueError("orientation must be length-4 quaternion [w,x,y,z]")
        quat_xyzw = np.array([ori[1], ori[2], ori[3], ori[0]], dtype=float)
        rot = R.from_quat(quat_xyzw)
        T = np.eye(4, dtype=float)
        T[:3, :3] = rot.as_matrix()
        T[:3, 3] = pos
        return T
    raise ValueError("Unsupported tcp_frame format")

class JointZeroCalibrate:
    def __init__(self, arm, data_qpos_rad, tcp_transform=None):
        self.data_qpos_rad = data_qpos_rad.T
        self.max_iterations = 100  # maximum number of iterations
        self.tolerance = 1e-5  # convergence tolerance
        # tcp_transform: 4x4 numpy array expressing tcp in the ee_frame (ee -> tcp)
        if tcp_transform is None:
            self.tcp_transform = np.eye(4, dtype=float)
        else:
            self.tcp_transform = np.asarray(tcp_transform, dtype=float)
            if self.tcp_transform.shape != (4, 4):
                raise ValueError("tcp_transform must be a 4x4 matrix")
        self._init_robot_model(arm)

    def _init_robot_model(self, arm):
        package_share_path = get_package_share_directory("w1_server")
        robot_cfg_path = Path(package_share_path) / "robot_config.yaml"
        with open(robot_cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)
        if arm == "left":
            self.robot_name = cfg.get("left_arm_name")
        elif arm == "right":
            self.robot_name = cfg.get("right_arm_name")
        else:
            raise ValueError(f"Invalid arm: {arm}, should be 'left' or 'right'")

        package_share_path = get_package_share_directory("robot_server_lib")
        urdf_path = (
            Path(package_share_path)
            / "robot_models"
            / self.robot_name
            / (self.robot_name + ".urdf")
        )
        self.arm_model = pinocchio.buildModelFromUrdf(urdf_path)

    def _skew(self, v):
        """返回 3x3 反对称矩阵"""
        return np.array([[0, -v[2], v[1]],
                         [v[2], 0, -v[0]],
                         [-v[1], v[0], 0]], dtype=float)

    def get_fk(self, q, ee_frame_name="ee_link"):
        """
        返回 4x4 齐次变换矩阵（tcp 在 base/world 坐标系下的位姿）。
        如果构造时提供了 tcp_transform（表示 ee_frame -> tcp 的变换），
        返回值为 oMf * tcp_transform。
        """
        data = pinocchio.Data(self.arm_model)

        ee_frame_id = self.arm_model.getFrameId(ee_frame_name)
        if ee_frame_id == -1:
            raise RuntimeError(f"Frame '{ee_frame_name}' not found in model!")

        q = np.asarray(q, dtype=float).flatten()
        if q.size not in (self.arm_model.nq, self.arm_model.nv):
            raise ValueError(f"q has wrong dimension: got {q.size}, expected {self.arm_model.nq} or {self.arm_model.nv}")

        pinocchio.forwardKinematics(self.arm_model, data, q)
        pinocchio.updateFramePlacements(self.arm_model, data)

        oMf = data.oMf[ee_frame_id]  # placement of ee_frame in world
        T_ee_world = np.eye(4, dtype=float)
        T_ee_world[:3, :3] = np.array(oMf.rotation)
        T_ee_world[:3, 3] = np.array(oMf.translation)

        # apply tcp offset expressed in ee_frame: world_tcp = T_ee_world * tcp_transform
        T_tcp_world = T_ee_world @ self.tcp_transform

        return T_tcp_world

    def get_jacobian(self, q, ee_frame_name="ee_link"):
        """
        返回 6 x nv 的雅可比矩阵（线速度在上，角速度在下），对应 tcp 点（如果有 tcp_offset 则为 tcp）。
        对于点偏移 r（在 world 下为 R_ee_world @ r_tcp_local），线速度部分修正为：
            Jv_tcp = Jv_ee - skew(r_world) @ Jw_ee
        角速度部分 Jw_tcp = Jw_ee
        """
        # 创建 Data
        data = pinocchio.Data(self.arm_model)

        # 查找 frame id
        ee_frame_id = self.arm_model.getFrameId(ee_frame_name)
        if ee_frame_id == -1:
            raise RuntimeError(f"Frame '{ee_frame_name}' not found in model!")

        q = np.asarray(q, dtype=float).flatten()
        if q.size not in (self.arm_model.nq, self.arm_model.nv):
            raise ValueError(f"q has wrong dimension: got {q.size}, expected {self.arm_model.nq} or {self.arm_model.nv}")

        pinocchio.forwardKinematics(self.arm_model, data, q)
        pinocchio.updateFramePlacements(self.arm_model, data)

        J = pinocchio.computeFrameJacobian(self.arm_model, data, q, ee_frame_id, pinocchio.LOCAL_WORLD_ALIGNED)
        # J: 6 x nv, 0:3 linear (world), 3:6 angular (world)

        # 如果 tcp_transform 是单位矩阵，则直接返回 J
        if np.allclose(self.tcp_transform, np.eye(4)):
            return J

        # 计算偏移 r 在 world 坐标下（tcp 在 ee_frame 下）
        oMf = data.oMf[ee_frame_id]
        R_ee_world = np.array(oMf.rotation)
        t_tcp_local = np.asarray(self.tcp_transform[:3, 3]).flatten()
        r_world = R_ee_world @ t_tcp_local

        Jv_ee = J[:3, :]
        Jw_ee = J[3:6, :]

        Jv_tcp = Jv_ee - self._skew(r_world) @ Jw_ee
        J_tcp = np.vstack((Jv_tcp, Jw_ee))

        return J_tcp

    def cal(self):
        # get the dof of the arm
        dof = self.arm_model.nv
        num_samples = self.data_qpos_rad.shape[1]
        delta_error = np.zeros(dof)
        result_rad = np.zeros(dof)  # sum of zero position error
        for iteration in range(self.max_iterations):
            # calculate the forward kinematics and Jacobian for each sample
            P_list = []
            J_list = []
            for i in range(num_samples):
                q_corrected = self.data_qpos_rad[:, i] + result_rad
                T = self.get_fk(q_corrected)
                J = self.get_jacobian(q_corrected)
                P_list.append(T[:3, 3])
                J_list.append(J[:3])

            # least squares error
            B = np.zeros((3 * (num_samples - 1), dof))
            Y = np.zeros(3 * (num_samples - 1))

            for i in range(num_samples - 1):
                B[3 * i:3 * i + 3, :] = J_list[i + 1] - J_list[i]
                Y[3 * i:3 * i + 3] = P_list[i] - P_list[i + 1]
            # calculate the delta error
            delta_error = pinv(B.T @ B) @ B.T @ Y
            result_rad += delta_error
            # check for convergence
            if np.all(np.abs(delta_error) < self.tolerance):
                # convert to degrees
                estimated_offsets_deg = np.rad2deg(result_rad)
                print(
                    f"Calibration succeeded: Iteration {iteration + 1}, Tolerance = {self.tolerance}"
                )
                print("Estimated offsets (deg):", estimated_offsets_deg)
                return True, estimated_offsets_deg
        else:
            print(
                f"Calibration failed: exceeded maximum iterations = {iteration + 1} without converging to the tolerance = {self.tolerance}"
            )
            print(
                "Please verify the calibrated arm and the sample data are correct."
            )
            return False, None

def main():
    # 从脚本同级目录读取 input_data.json
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_path = os.path.join(script_dir, "input_data.json")
    if not os.path.exists(input_path):
        raise FileNotFoundError(f"input_data.json not found in {script_dir}")

    with open(input_path, "r") as f:
        data = json.load(f)

    # 在程序开始时打印 ReadMe（如果存在）
    readme = data.get("ReadMe", None)
    if readme:
        print("ReadMe:")
        if isinstance(readme, (list, tuple)):
            for line in readme:
                print(" -", line)
        else:
            print(readme)

    arm = data.get("arm")

    data_qpos = data.get("data_qpos", None)
    if data_qpos is None:
        raise ValueError("input_data.json missing 'data_qpos_deg'")

    if data.get("data_unit", "deg") == "deg":
        data_qpos_rad = np.radians(np.array(data_qpos, dtype=float))
    else:
        data_qpos_rad = np.array(data_qpos, dtype=float)

    tcp_raw = data.get("tcp_frame", None)
    tcp_transform = _parse_tcp_frame(tcp_raw)

    joint_zero_calibrate = JointZeroCalibrate(arm, data_qpos_rad, tcp_transform=tcp_transform)
    res, estimated_offsets_deg = joint_zero_calibrate.cal()
    if res:
        print("Estimated offsets (deg):", estimated_offsets_deg)
        # 命令行确认是否写入电机
        try:
            to_print = np.array(estimated_offsets_deg)
            print("\n即将把以下零位偏差(度)写入对应手臂电机：")
            print(to_print)
            ans = input("确认执行零位校准并写入电机吗? 如需写入请先操作电机下使能op, 如已下OP且确认写入, 请输入 'yes' 或 'y'，其他任意键取消 [y/N]: ").strip().lower()
            if ans in ("y", "yes"):
                set_zero_offset(False, to_print.tolist())
            else:
                print("已取消写入电机。")
        except KeyboardInterrupt:
            print("\n已取消写入电机。")
    else:
        print("Calibration failed.")

if __name__ == "__main__":
    np.set_printoptions(4, suppress=True)
    main()
