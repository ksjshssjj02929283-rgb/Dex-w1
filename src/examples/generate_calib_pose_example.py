'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''
import numpy as np
from scipy.spatial.transform import Rotation as R


def generate_calib_pose(
    init_pose: np.ndarray,
    col_num: int = 2,
    row_num: int = 2,
    layer_num: int = 2,
    col_size: float = 0.05,
    row_size: float = 0.05,
    layer_size: float = 0.05,
    rot_rad: float = np.deg2rad(10)
) -> tuple[bool, list, list]:

    def generate_trans_pose(init_pose, dx, dy, dz):
        new_pose = init_pose.copy()
        new_pose[0, 3] += dx
        new_pose[1, 3] += dy
        new_pose[2, 3] += dz
        return new_pose

    def generate_rot_pose(init_pose, rx, ry, rz):
        new_pose = init_pose.copy()
        rot = R.from_euler('xyz', [rx, ry, rz]).as_matrix() @ new_pose[:3, :3]
        new_pose[:3, :3] = rot
        return new_pose

    if col_num < 1 or row_num < 1 or layer_num < 1:
        return False, [], []
    if col_size <= 0. or row_size <= 0. or layer_size <= 0.:
        return False, [], []

    col_list = np.linspace(-1., 1., col_num)
    row_list = np.linspace(-1., 1., row_num)
    layer_list = np.linspace(-1., 1., layer_num)

    d_col = 0. if col_num == 1 else col_size * 0.5
    d_row = 0. if row_num == 1 else row_size * 0.5
    d_layer = 0. if layer_num == 1 else layer_size * 0.5

    trans_pose_list = []
    is_col_inverse = True
    is_row_inverse = True
    for dz in range(layer_num - 1, -1, -1):
        is_row_inverse = not is_row_inverse
        for y in range(row_num - 1, -1, -1):
            dy = row_num - y - 1 if is_row_inverse else y
            is_col_inverse = not is_col_inverse
            for x in range(col_num - 1, -1, -1):
                dx = col_num - x - 1 if is_col_inverse else x
                new_pose = generate_trans_pose(
                    init_pose,
                    d_col * col_list[dx],
                    d_row * row_list[dy],
                    d_layer * layer_list[dz],
                )
                trans_pose_list.append(new_pose)

    rotate_pose_list = []
    rot_rad_list = np.array([[-rot_rad, 0.0, 0.0], [rot_rad, 0.0, 0.0],
                             [0.0, -rot_rad, 0.0], [0.0, rot_rad, 0.0],
                             [0.0, 0.0, -rot_rad], [0.0, 0.0, rot_rad]])
    for i in range(rot_rad_list.shape[0]):
        new_rot_pose = generate_rot_pose(
            init_pose,
            rot_rad_list[i, 0],
            rot_rad_list[i, 1],
            rot_rad_list[i, 2],
        )
        rotate_pose_list.append(new_rot_pose)

    return True, trans_pose_list, rotate_pose_list


if __name__ == "__main__":
    from dexe_robot_client.w1 import W1Manager, PathRnBezier, PathType
    from dexe_rp import EndEffectorType, RobotVersion, StatusType

    w1_manager = W1Manager(is_sim=True,
                           robot_version=RobotVersion.W1V024,
                           end_effector_type=EndEffectorType.BrainCo_Revo1_R)

    init_qpos = np.array(
        [-0.5, -0.5, 0.9169, -1.3845, -1.8167, 0.3516, -0.7812])
    init_pose = w1_manager.get_xpos(name="left_arm",
                                    qpos=init_qpos,
                                    is_local=True)[1]

    res, trans_pose, rot_pose = generate_calib_pose(init_pose,
                                                    col_num=2,
                                                    row_num=2,
                                                    layer_num=2,
                                                    col_size=0.05,
                                                    row_size=0.05,
                                                    layer_size=0.05,
                                                    rot_rad=np.deg2rad(10))
    if not res:
        exit(-1)
    total_pose = list(np.concatenate((trans_pose, rot_pose), axis=0))

    # Calc IK
    total_qpos = []
    arm_kine = w1_manager._get_arm_kine("left_arm")
    for pose in total_pose:
        res_ik_status, cur_qpos = arm_kine.get_nearest_ik(pose, init_qpos)
        if res_ik_status != StatusType.Success:
            exit(-2)
        total_qpos.append(cur_qpos)

    path = PathRnBezier(waypoints=np.asarray(total_qpos))
    if not path.calc_path(PathType.Bezier5thJoint, 0.005):
        exit(-3)
    sample_num = 100
    qpos_array = path.get_sample_by_num(sample_num)

    # Move Here
