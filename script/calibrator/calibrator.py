import os
import json
import numpy as np
from scipy.linalg import pinv
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import pinocchio
from scipy.spatial.transform import Rotation as R

class JointZeroCalibrate:
    def __init__(self, arm, data_qpos_rad):
        # tcp: None 或 7 元素序列 [tx,ty,tz,qx,qy,qz,qw]
        self.data_qpos_rad = data_qpos_rad.T
        self.max_iterations = 100  # maximum number of iterations
        self.tolerance = 1e-5  # convergence tolerance
        self._init_robot_model(arm)

    def _init_robot_model(self, arm):
        package_share_path = get_package_share_directory("w1_server")
        robot_cfg_path = Path(package_share_path) / "robot.cfg"
        with open(robot_cfg_path, 'r') as f:
            cfg = json.load(f)
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
            / (self.robot_name + "_calibration.urdf")
        )
        self.arm_model = pinocchio.buildModelFromUrdf(urdf_path)

    def get_fk(self, q, ee_frame_name="ee_link"):
        """
        返回 4x4 齐次变换矩阵（末端在 base 坐标系下的位姿）。
        tcp: 可选，工具坐标系相对于末端执行器的位姿。
             - 若为 None：不变换
             - 若为 7 元素序列 [tx,ty,tz,qx,qy,qz,qw]：平移+四元数（推荐）
             不再支持仅四元数 4 元素的情况。
        返回 4x4 numpy 矩阵（base -> tcp）。
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

        oMf = data.oMf[ee_frame_id]  # pinocchio.SE3
        T = np.eye(4, dtype=float)
        T[:3, :3] = np.array(oMf.rotation)
        T[:3, 3] = np.array(oMf.translation)

        return T
    
    def get_jacobian(self, q, ee_frame_name="ee_link"):
        """
        返回 6 x nv 的雅可比矩阵（线速度在上，角速度在下）。
        tcp: 同 get_fk，支持 None 或 7 元素 (tx,ty,tz,qx,qy,qz,qw)。
             不再支持仅四元数 4 元素的情况。
        如果 tcp 存在，雅可比会应用工具的平移与旋转： J_tcp = Ad(T_tcp) * J_ee
        """
        # 创建 Data
        data = pinocchio.Data(self.arm_model)

        # 查找 frame id
        ee_frame_id = self.arm_model.getFrameId(ee_frame_name)
        if ee_frame_id == -1:
            raise RuntimeError(f"Frame '{ee_frame_name}' not found in model!")

        # 确保 q 为 numpy 向量且维度匹配
        q = np.asarray(q, dtype=float).flatten()
        if q.size not in (self.arm_model.nq, self.arm_model.nv):
            raise ValueError(f"q has wrong dimension: got {q.size}, expected {self.arm_model.nq} or {self.arm_model.nv}")

        # 前向运动学与 frame placement 更新
        pinocchio.forwardKinematics(self.arm_model, data, q)
        pinocchio.updateFramePlacements(self.arm_model, data)

        # 计算 frame 雅可比（返回 6 x nv 矩阵），在 frame 局部系下
        J = pinocchio.computeFrameJacobian(self.arm_model, data, q, ee_frame_id, pinocchio.LOCAL)
        J = np.asarray(J, dtype=float)  # 6 x nv

        return J

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
                try:
                    # 传入 self.tcp（可能为 None）
                    T = self.get_fk(q_corrected)
                    J = self.get_jacobian(q_corrected)
                    P_list.append(T[:3, 3])
                    J_list.append(J[:3])
                except Exception as e:
                    raise RuntimeError(f"Error in FK or Jacobi calculation: {e}")

            # least squares error
            B = np.zeros((3 * (num_samples - 1), dof))
            Y = np.zeros(3 * (num_samples - 1))

            for i in range(num_samples - 1):
                B[3 * i:3 * i + 3, :] = J_list[i + 1] - J_list[i]
                Y[3 * i:3 * i + 3] = P_list[i] - P_list[i + 1]

            # calculate the delta error
            delta_error = pinv(B.T @ B) @ B.T @ Y
            result_rad += delta_error
            result_rad[0] = 0

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

    arm_key = data.get("arm")

    data_qpos = data.get("data_qpos", None)
    if data_qpos is None:
        raise ValueError("input_data.json missing 'data_qpos_deg'")

    if data.get("data_unit", "deg") == "deg":
        data_qpos_rad = np.radians(np.array(data_qpos, dtype=float))
    else:
        data_qpos_rad = np.array(data_qpos, dtype=float)

    joint_zero_calibrate = JointZeroCalibrate(arm_key, data_qpos_rad)
    res, estimated_offsets_deg = joint_zero_calibrate.cal()
    if res:
        print("Estimated offsets (deg):", estimated_offsets_deg)
    else:
        print("Calibration failed.")


'''关节零位标定说明：
输入数据单位为度
'''
if __name__ == "__main__":
    np.set_printoptions(4, suppress=True)
    main()
