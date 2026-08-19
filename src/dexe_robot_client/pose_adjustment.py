'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
from scipy.spatial.transform import Rotation

from .common.status import StatusCode

_SE3_ATOL = 1e-5


def _parse_user_pose(
        pose: list[float] | np.ndarray) -> tuple[StatusCode, np.ndarray | None]:
    try:
        pose_array = np.asarray(pose, dtype=float)
    except (TypeError, ValueError):
        return StatusCode.INPUT_INVALID, None

    if pose_array.shape != (6,):
        return StatusCode.DIM_MISMATCH, None
    if not np.all(np.isfinite(pose_array)):
        return StatusCode.INPUT_INVALID, None
    return StatusCode.SUCCESS, pose_array


def _parse_se3(
    transform: list[list[float]] | np.ndarray
) -> tuple[StatusCode, np.ndarray | None]:
    try:
        transform_array = np.asarray(transform, dtype=float)
    except (TypeError, ValueError):
        return StatusCode.INPUT_INVALID, None

    if transform_array.shape != (4, 4):
        return StatusCode.DIM_MISMATCH, None
    if not np.all(np.isfinite(transform_array)):
        return StatusCode.INPUT_INVALID, None
    if not np.allclose(transform_array[3],
                       np.array([0., 0., 0., 1.]),
                       atol=_SE3_ATOL,
                       rtol=0.):
        return StatusCode.INPUT_INVALID, None

    rotation = transform_array[:3, :3]
    if not np.allclose(
            rotation.T @ rotation, np.eye(3), atol=_SE3_ATOL,
            rtol=0.) or not np.isclose(
                np.linalg.det(rotation), 1., atol=_SE3_ATOL, rtol=0.):
        return StatusCode.INPUT_INVALID, None

    # Work with a canonical homogeneous last row after accepting small sensor
    # round-off, and keep the caller's input untouched.
    transform_array = transform_array.copy()
    transform_array[3] = np.array([0., 0., 0., 1.])
    return StatusCode.SUCCESS, transform_array


def _user_pose_to_xpos(user_pose: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", user_pose[3:],
                                            degrees=True).as_matrix()
    transform[:3, 3] = user_pose[:3] / 1000.
    return transform


def _xpos_to_user_pose(transform: np.ndarray) -> list[float]:
    position_mm = transform[:3, 3] * 1000.
    euler_deg = Rotation.from_matrix(transform[:3, :3]).as_euler("xyz",
                                                                 degrees=True)
    return position_mm.tolist() + euler_deg.tolist()


def adjust_dual_arm_taught_tcp1_poses(
    template_box_T_current_box: list[list[float]] | np.ndarray,
    template_left_tcp1_pose: list[float] | np.ndarray,
    template_right_tcp1_pose: list[float] | np.ndarray,
    template_box_T_left_tcp1: list[list[float]] | np.ndarray,
    template_box_T_right_tcp1: list[list[float]] | np.ndarray,
) -> tuple[StatusCode, list[float] | None, list[float] | None]:
    """Calculate corrected dual-arm TCP1 teaching poses.

    The transform convention is ``A_T_B`` and the inputs must satisfy:

    ``reference_T_current_box = reference_T_template_box
    @ template_box_T_current_box``

    ``arm_base_T_tcp1 = arm_base_T_box @ box_T_tcp1``

    The two teaching poses use mm, deg and ``xyz`` Euler angles. The three
    homogeneous transforms use metres.
    """
    pose_inputs = (
        template_left_tcp1_pose,
        template_right_tcp1_pose,
    )
    parsed_poses: list[np.ndarray] = []
    for pose in pose_inputs:
        status, pose_array = _parse_user_pose(pose)
        if status != StatusCode.SUCCESS:
            return status, None, None
        parsed_poses.append(pose_array)

    transform_inputs = (
        template_box_T_current_box,
        template_box_T_left_tcp1,
        template_box_T_right_tcp1,
    )
    parsed_transforms: list[np.ndarray] = []
    for transform in transform_inputs:
        status, transform_array = _parse_se3(transform)
        if status != StatusCode.SUCCESS:
            return status, None, None
        parsed_transforms.append(transform_array)

    left_tcp1_template = _user_pose_to_xpos(parsed_poses[0])
    right_tcp1_template = _user_pose_to_xpos(parsed_poses[1])
    box_delta, box_T_left_tcp1, box_T_right_tcp1 = parsed_transforms

    # First recover each template box pose in its own arm-base frame. Apply
    # the box-local right increment, then reattach the corresponding TCP1.
    left_box_template = left_tcp1_template @ np.linalg.inv(box_T_left_tcp1)
    right_box_template = right_tcp1_template @ np.linalg.inv(box_T_right_tcp1)
    left_box_current = left_box_template @ box_delta
    right_box_current = right_box_template @ box_delta
    left_tcp1_adjusted = left_box_current @ box_T_left_tcp1
    right_tcp1_adjusted = right_box_current @ box_T_right_tcp1

    return (StatusCode.SUCCESS, _xpos_to_user_pose(left_tcp1_adjusted),
            _xpos_to_user_pose(right_tcp1_adjusted))


    @staticmethod
    def adjust_dual_arm_taught_tcp1_poses(
        template_box_T_current_box: list[list[float]] | np.ndarray,
        template_left_tcp1_pose: list[float] | np.ndarray,
        template_right_tcp1_pose: list[float] | np.ndarray,
        template_box_T_left_tcp1: list[list[float]] | np.ndarray,
        template_box_T_right_tcp1: list[list[float]] | np.ndarray,
    ) -> tuple[StatusCode, list[float] | None, list[float] | None]:
        """根据箱体中心的视觉变化量校正双臂 TCP1 示教点。

        该接口只进行刚体位姿计算，不规划或执行机器人运动。它既可用于校正
        双臂接近箱体时的抓取起点，也可用于校正双臂协同搬运时的放置目标点。

        变换采用 ``A_T_B`` 记法，表示坐标系 B 在坐标系 A 下的位姿，并约定
        视觉变化量满足：

        ``reference_T_current_box = reference_T_template_box @ template_box_T_current_box``

        相对抓取位姿的方向定义为：

        ``arm_base_T_tcp1 = arm_base_T_box @ box_T_tcp1``

        左右臂分别在各自 arm-base 坐标系下计算。以左臂为例：

        ``left_base_T_tcp1_new = left_base_T_tcp1_template
        @ inv(template_box_T_left_tcp1)
        @ template_box_T_current_box
        @ template_box_T_left_tcp1``

        Args:
            template_box_T_current_box (list | np.ndarray, shape=(4, 4)):
                模板箱体中心坐标系到当前箱体中心坐标系的右乘变化量，平移单位 m。
            template_left_tcp1_pose (list | np.ndarray, shape=(6,)):
                左臂模板 TCP1 示教点 ``[x, y, z, rx, ry, rz]``，位于左臂
                arm-base 坐标系，单位 mm、deg，欧拉角顺序为 ``xyz``。
            template_right_tcp1_pose (list | np.ndarray, shape=(6,)):
                右臂模板 TCP1 示教点，位于右臂 arm-base 坐标系，格式同上。
            template_box_T_left_tcp1 (list | np.ndarray, shape=(4, 4)):
                模板左 TCP1 在模板箱体中心坐标系下的位姿，平移单位 m。
            template_box_T_right_tcp1 (list | np.ndarray, shape=(4, 4)):
                模板右 TCP1 在模板箱体中心坐标系下的位姿，平移单位 m。

        Returns:
            tuple[StatusCode, list, list]: 状态码，以及校正后的左、右 TCP1
            示教点。输出格式仍为 ``[x, y, z, rx, ry, rz]``（mm、deg）；
            输入无效时两个示教点均为 ``None``。

        Notes:
            - TCP1 就是现有笛卡尔规划接口使用的 TCP，不需要额外引入 TCP2
              或法兰到非标工具接触点的变换。
            - 抓取前若双臂尚未建立目标相对关系，应把两个输出交给支持双臂
              独立目标的规划接口。已经夹持箱体后调用
              ``plan_linear_dual_arm_sync`` 时，只传主臂的校正目标；另一个输出
              可用于校验从臂的期望终点。此时规划起点的双手关系还必须等于
              ``inv(template_box_T_left_tcp1) @ template_box_T_right_tcp1``。
            - 两个输出分别位于左、右 arm-base 坐标系，可直接用于
              ``is_local=True`` 的规划；全局规划前必须先转换到共同坐标系。
            - 每个 ``box_T_tcp1`` 必须描述本次被校正示教点自身与箱体的关系；
              带退让距离的预抓取点不能直接复用实际接触抓取点的相对位姿。
            - 本接口不检查可达性、关节限位或碰撞，输出仍需交给相应规划接口
              完成安全校验。
        """
        return _adjust_dual_arm_taught_tcp1_poses(
            template_box_T_current_box=template_box_T_current_box,
            template_left_tcp1_pose=template_left_tcp1_pose,
            template_right_tcp1_pose=template_right_tcp1_pose,
            template_box_T_left_tcp1=template_box_T_left_tcp1,
            template_box_T_right_tcp1=template_box_T_right_tcp1,
        )