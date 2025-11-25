# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import Rotation as R
from scipy.special import comb
from ...common import get_logger

SMALL_NUM = 1e-6  # 1um


def _bernstein_poly(k, n, s):
    return comb(n, k) * (s**k) * (1 - s)**(n - k)


def _bezier_path(control_points, s):
    """s -> [0, 1]"""
    n = control_points.shape[0] - 1
    B_vec = np.zeros((1, n + 1))  # a vector of each bernstein polynomial
    for k in range(n + 1):
        B_vec[0, k] = _bernstein_poly(k, n, s)
    p = (B_vec @ control_points).ravel()
    return p


def _lerp_path(control_points, s):
    """s -> [0, 1]"""
    return (1 - s) * control_points[0] + s * control_points[1]


def _divide_waypoints(three_waypoints: np.ndarray,
                      blend_tolerance: float,
                      has_start: bool = False,
                      has_end: bool = False) -> tuple[bool, list[list]]:
    if len(three_waypoints) != 3:
        get_logger().error(f"Num of waypoints must = 3.")
        return False, None
    if blend_tolerance < SMALL_NUM:
        get_logger().error(f"blend_tolerance must > 0.")
        return False, None

    p0 = three_waypoints[0]
    p1 = three_waypoints[1]
    p2 = three_waypoints[2]

    len1 = np.linalg.norm(p1 - p0)
    len2 = np.linalg.norm(p2 - p1)
    if len1 < 2 * SMALL_NUM or len2 < 2 * SMALL_NUM:
        get_logger().error(f"len ~= 0.")
        return False, None

    blend1 = blend_tolerance
    blend2 = blend_tolerance
    if len1 < 2 * blend_tolerance:
        blend1 = len1 / 2. - SMALL_NUM  # 避免出现长度为0的path segment
    if len2 < 2 * blend_tolerance:
        blend2 = len2 / 2. - SMALL_NUM
    if has_start and len1 < blend_tolerance:
        blend1 = len1
    if has_end and len2 < blend_tolerance:
        blend2 = len2

    s01 = (len1 - blend1) / len1
    p01 = (1 - s01) * p0 + s01 * p1

    s12 = blend2 / len2
    p12 = (1 - s12) * p1 + s12 * p2

    return True, [[p0, p01], [p01, p1, p12], [p12, p2]]


# TODO: 采样参数是长度还是数量？
# NOTE: coner cases
# 1. 0/180deg √
# 2. repeat points √
# 3. 长度小于blend_tolarence √
# 4. blend_tolarence = 0 √
def interpolate_cartesian_path(waypoint_list: list[np.ndarray],
                               blend_tolerance: float,
                               num_samples: int) -> tuple[bool, np.ndarray]:
    # 输入参数检查
    num_raw_waypoints = len(waypoint_list)
    if num_raw_waypoints < 2:
        get_logger().error(
            f"Number of waypoints must >= 2 rather than {num_raw_waypoints}.")
        return False, None

    if not all(
            isinstance(item, np.ndarray) and item.shape == (4, 4)
            for item in waypoint_list):
        get_logger().error(f"Each waypoints should be 4x4 np.ndarray.")
        return False, None

    # 对重复的waypoint合并
    processed_waypoint_list = []
    for waypoint in waypoint_list:
        if not processed_waypoint_list:
            processed_waypoint_list.append(waypoint)
            continue

        dist = np.linalg.norm(waypoint[:3, 3] -
                              processed_waypoint_list[-1][:3, 3])
        if dist > SMALL_NUM:
            processed_waypoint_list.append(waypoint)
        else:
            # 检查SO3距离
            temp = processed_waypoint_list[-1][:3, :3].T @ waypoint[:3, :3]
            SO3_dist = np.sum(
                R.from_matrix(temp).as_euler(seq="xyz", degrees=False))
            if SO3_dist > np.pi / 2:
                get_logger().error(
                    f"Repeat waypoints with the same R3 and different SO3 are not supported."
                )
                return False, None

    num_waypoints = len(processed_waypoint_list)
    if num_waypoints < 2:
        processed_waypoint_list = waypoint_list[:2]
        num_waypoints = len(processed_waypoint_list)
        # TODO: 避免很短的平移内进行大范围旋转
        # print(f"Only 1 non-repeat waypoint are not supported")
        # return False, None

    # R3插值
    R3_waypoints = np.array(
        [waypoint[:3, 3] for waypoint in processed_waypoint_list])
    # 拆分控制点
    path_seg_points_list = []
    if blend_tolerance < SMALL_NUM or num_waypoints == 2:
        for i in range(num_waypoints - 1):
            path_seg_points_list.append([R3_waypoints[i], R3_waypoints[i + 1]])
    else:
        for i in range(num_waypoints - 2):
            has_start = False
            has_end = False
            if i == 0:
                has_start = True
            if i == num_waypoints - 3:
                has_end = True
            divide_res, sub_path_seg_point_list = _divide_waypoints(
                R3_waypoints[i:i + 3], blend_tolerance, has_start, has_end)
            if not divide_res:
                return False, None
            if len(path_seg_points_list) == 0:
                path_seg_points_list.extend(sub_path_seg_point_list)
                continue
            path_seg_points_list[-1][-1] = sub_path_seg_point_list[0][-1]
            path_seg_points_list.extend(sub_path_seg_point_list[1:])

    # 计算各路径长度
    # NOTE: bezier路径长度用控制点路径长度和来代替
    path_seg_dist_list = []
    for path_seg_points in path_seg_points_list:
        path_seg_dist = 0.
        for i in range(len(path_seg_points) - 1):
            dist = np.linalg.norm(path_seg_points[i + 1] - path_seg_points[i])
            path_seg_dist += dist
        path_seg_dist_list.append(path_seg_dist)

    total_path_length = np.sum(path_seg_dist_list)
    s_list = np.linspace(0., total_path_length, num_samples)

    # R3采样
    R3_sample_list = []
    cur_path_seg_idx = 0
    cur_path_seg_length = 0.
    for i, s in enumerate(s_list):
        # 转换至path segment的normalize s
        if i == len(s_list) - 1:
            cur_path_seg_idx = len(path_seg_dist_list) - 1
            normal_s = 1.
        elif s - cur_path_seg_length < SMALL_NUM:
            normal_s = 0.
        else:
            # 采样点可能越过多段path segment
            while s - cur_path_seg_length > path_seg_dist_list[
                    cur_path_seg_idx]:
                cur_path_seg_length += path_seg_dist_list[cur_path_seg_idx]
                cur_path_seg_idx += 1
                if cur_path_seg_idx == len(path_seg_dist_list) - 1:
                    break
            normal_s = (
                s - cur_path_seg_length) / path_seg_dist_list[cur_path_seg_idx]
        normal_s = np.clip(normal_s, 0., 1.)  # 避免浮点影响
        # 采样path segment
        control_points = np.array(path_seg_points_list[cur_path_seg_idx])
        if control_points.shape[0] == 2:
            R3_sample_list.append(_lerp_path(control_points, normal_s))
        elif control_points.shape[0] == 3:
            R3_sample_list.append(_bezier_path(control_points, normal_s))
    # -------------------------------------------------------
    # SO3插值
    SO3_waypoint_list = [
        waypoint[:3, :3] for waypoint in processed_waypoint_list
    ]
    # 计算各路径长度
    raw_path_prefix_length_list = []
    prefix_length = 0.
    for k in range(len(processed_waypoint_list) - 1):
        p0 = processed_waypoint_list[k][:3, 3]
        p1 = processed_waypoint_list[k + 1][:3, 3]
        prefix_length += np.linalg.norm(p1 - p0)
        raw_path_prefix_length_list.append(prefix_length)
    # SO3采样
    SO3_s_list = [
        temp / raw_path_prefix_length_list[-1]
        for temp in raw_path_prefix_length_list
    ]
    SO3_s_list = [0.] + SO3_s_list[:-1] + [1.]
    SO3_slerp = Slerp(SO3_s_list, R.from_matrix(SO3_waypoint_list))
    SO3_sample_list = []
    for s in np.linspace(0., 1., num_samples):
        SO3_sample_list.append(SO3_slerp(s).as_matrix())

    SE3_sample_list = []
    for R3, SO3 in zip(R3_sample_list, SO3_sample_list):
        SE3 = np.eye(4)
        SE3[:3, 3] = R3
        SE3[:3, :3] = SO3
        SE3_sample_list.append(SE3)

    return True, np.array(SE3_sample_list)  # (N, 4, 4)
