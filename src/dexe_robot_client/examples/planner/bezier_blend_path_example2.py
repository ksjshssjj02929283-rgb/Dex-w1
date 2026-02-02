import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
from IPython import embed

from dexe_robot_client.w1 import PathType, PathRnBezier, PathR3xSO3Bezier


def cal_path_seg_interpolate_num(waypoint_list, cartesian_step):
    interpolate_num_list = []
    for i in range(len(waypoint_list) - 1):
        dist = np.linalg.norm(waypoint_list[i + 1][:3, 3] -
                              waypoint_list[i][:3, 3])
        interpolate_num = max(2, int(np.ceil(dist / cartesian_step)))
        interpolate_num_list.append(interpolate_num)
    return interpolate_num_list


def test():
    waypoint1 = np.eye(4)
    waypoint1[:3, 3] = np.array([0, 0, 0])

    waypoint2 = np.eye(4)
    waypoint2[:3, :3] = R.from_euler(seq="xyz", angles=[30, 0, 0],
                                     degrees=True).as_matrix()
    waypoint2[:3, 3] = np.array([1, 0, 1])

    waypoint3 = np.eye(4)
    waypoint3[:3, :3] = R.from_euler(seq="xyz",
                                     angles=[30, 0, -45],
                                     degrees=True).as_matrix()
    waypoint3[:3, 3] = np.array([0.3, 1, 1])

    waypoint4 = np.eye(4)
    waypoint4[:3, :3] = R.from_euler(seq="xyz",
                                     angles=[30, 20, -45],
                                     degrees=True).as_matrix()
    waypoint4[:3, 3] = np.array([0, 1, 1])

    waypoint5 = np.eye(4)
    waypoint5[:3, 3] = np.array([0.1, 0.5, 0])

    waypoints = np.array(
        [waypoint1, waypoint2, waypoint3, waypoint4, waypoint5])
    path = PathR3xSO3Bezier(waypoints)
    if not path.calc_path(PathType.Bezier5thCartesian, 0.05):
        exit(-1)

    interpolate_num_list = cal_path_seg_interpolate_num(waypoint_list=waypoints,
                                                        cartesian_step=0.005)

    path_seg_list = path.get_path_segments()
    linear_seg_list, blend_seg_list = [], []
    for i in range(len(path_seg_list)):
        if i % 2 == 0:
            linear_seg_list.append(path_seg_list[i])
        else:
            blend_seg_list.append(path_seg_list[i])

    rearrange_seg_list = []
    if len(blend_seg_list) != 0:
        for i in range(len(linear_seg_list)):
            if i == 0:
                rearrange_seg_list.append(
                    ([], linear_seg_list[i], blend_seg_list[i]))
            elif i == len(linear_seg_list) - 1:
                rearrange_seg_list.append(
                    (blend_seg_list[i - 1], linear_seg_list[i], []))
            else:
                rearrange_seg_list.append(
                    (blend_seg_list[i - 1], linear_seg_list[i],
                     blend_seg_list[i]))
    else:
        rearrange_seg_list.append(([], linear_seg_list[0], []))

    result_sample_list = []
    for idx, (pre_blend_seg, linear_seg,
              post_blend_seg) in enumerate(rearrange_seg_list):
        interpolate_num = interpolate_num_list[idx]
        pre_blend_seg_length = pre_blend_seg.get_length(
        ) if pre_blend_seg != [] else 0.0
        linear_seg_length = linear_seg.get_length()
        post_blend_seg_length = post_blend_seg.get_length(
        ) if post_blend_seg != [] else 0.0
        pre_blend_interpolate_num = max(
            0,
            int(
                np.ceil((pre_blend_seg_length / 2) /
                        (pre_blend_seg_length / 2 + linear_seg_length +
                         post_blend_seg_length / 2) *
                        interpolate_num))) if pre_blend_seg != [] else 0
        linear_interpolate_num = max(
            0,
            int(
                np.ceil(linear_seg_length /
                        (pre_blend_seg_length / 2 + linear_seg_length +
                         post_blend_seg_length / 2) * interpolate_num)))
        post_blend_interpolate_num = max(
            0,
            int(
                np.ceil((post_blend_seg_length / 2) /
                        (pre_blend_seg_length / 2 + linear_seg_length +
                         post_blend_seg_length / 2) *
                        interpolate_num))) if post_blend_seg != [] else 0

        sample_seg_list = []
        pre_blend_s = np.linspace(
            pre_blend_seg.get_s0() + pre_blend_seg.get_length() / 2,
            pre_blend_seg.get_s0() + pre_blend_seg.get_length(),
            pre_blend_interpolate_num) if pre_blend_seg != [] else None
        linear_s = np.linspace(linear_seg.get_s0(),
                               linear_seg.get_s0() + linear_seg.get_length(),
                               linear_interpolate_num)
        post_blend_s = np.linspace(
            post_blend_seg.get_s0(),
            post_blend_seg.get_s0() + post_blend_seg.get_length(),
            post_blend_interpolate_num) if post_blend_seg != [] else None

        if pre_blend_s is not None:
            for s in pre_blend_s:
                sample_seg_list.append(pre_blend_seg.get_config(s))
        for s in linear_s:
            sample_seg_list.append(linear_seg.get_config(s))
        if post_blend_s is not None:
            for s in post_blend_s:
                sample_seg_list.append(post_blend_seg.get_config(s))

        result_sample_list.append(sample_seg_list)

    embed()


if __name__ == "__main__":
    test()
