# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
import pinocchio as pin
from enum import Enum, auto
from abc import ABC, abstractmethod

from ...common import get_logger
from .blend_path_segment import (SMALL, EPSILON, weighted_norm,
                                 calc_5th_bezier_Rn_control_points,
                                 calc_5th_bezier_SO3_control_points,
                                 PathSegmentBase, PathSegRnBezier,
                                 PathSegSO3Bezier, PathSegR3xSO3Bezier)


class PathType(Enum):
    Bezier2ndJoint = auto()
    Bezier5thJoint = auto()
    Bezier2ndCartesian = auto()
    Bezier5thCartesian = auto()


class PathBezier(ABC):

    def __init__(self, waypoints: np.ndarray):
        self.logger = get_logger()
        self._waypoints = waypoints
        self._path_seg_list: list[PathSegmentBase] = []
        self._length = 0.
        self.is_init = False

    @abstractmethod
    def _pre_process(self) -> tuple[bool, np.ndarray]:
        pass

    @abstractmethod
    def calc_path(self,
                  path_type: PathType,
                  blend_tolerance: float = 0.005) -> bool:
        pass

    def get_waypoints(self) -> np.ndarray:
        return self._waypoints

    def get_length(self) -> float:
        if not self.is_init:
            self.logger.error(f"Not init, call calc_path() first!")
        return self._length

    def get_path_segments(self) -> list[PathSegmentBase]:
        if not self.is_init:
            self.logger.error(f"Not init, call calc_path() first!")
        return self._path_seg_list

    def _get_path_segment_at_s(self, s: float) -> PathSegmentBase:
        if not self.is_init:
            self.logger.error(f"Not init, call calc_path() first!")
        s_ = max(0, min(s, self._length))
        for path_seg in self._path_seg_list:
            ss = path_seg.get_s0()
            se = ss + path_seg.get_length()
            if s_ <= se:
                return path_seg

    def get_config(self, s: float) -> np.ndarray:
        if not self.is_init:
            self.logger.error(f"Not init, call calc_path() first!")
            return None
        path_seg = self._get_path_segment_at_s(s)
        return path_seg.get_config(s)

    def get_tangent(self, s: float) -> np.ndarray:
        if not self.is_init:
            self.logger.error(f"Not init, call calc_path() first!")
            return None
        path_seg = self._get_path_segment_at_s(s)
        return path_seg.get_tangent(s)

    def get_curvature(self, s: float) -> np.ndarray:
        if not self.is_init:
            self.logger.error(f"Not init, call calc_path() first!")
            return None
        path_seg = self._get_path_segment_at_s(s)
        return path_seg.get_curvature(s)

    def get_torsion(self, s: float) -> np.ndarray:
        if not self.is_init:
            self.logger.error(f"Not init, call calc_path() first!")
            return None
        path_seg = self._get_path_segment_at_s(s)
        return path_seg.get_torsion(s)

    def get_sample_by_step(self, step: float) -> np.ndarray:
        if not self.is_init:
            self.logger.error(f"Not init, call calc_path() first!")
            return None
        if step >= self.get_length():
            self.logger.error(f"step should < {self.get_length()}")
            return None
        if step <= 0.:
            self.logger.error(f"step should > 0")
            return None

        s_vals = np.arange(0, self.get_length(), step)
        if s_vals[-1] < self.get_length():
            s_vals = np.append(s_vals, self.get_length())
        return np.array([self.get_config(s).tolist() for s in s_vals])

    def get_sample_by_num(self, num: int) -> np.ndarray:
        if not self.is_init:
            self.logger.error(f"Not init, call calc_path() first!")
            return None
        if num < 2:
            self.logger.error(f"num should >= 2")
            return None

        s_vals = np.linspace(0, self.get_length(), num)
        return np.array([self.get_config(s).tolist() for s in s_vals])


class PathRnBezier(PathBezier):

    def __init__(self, waypoints: np.ndarray):
        super().__init__(waypoints)

    def _pre_process(self) -> tuple[bool, np.ndarray]:
        if len(self._waypoints) < 2:
            self.logger.error("Num of waypoints should > 2!")
            return False, None

        # Repeat waypoints
        waypoints_1 = [self._waypoints[0]]
        for i in range(1, len(self._waypoints)):
            dist = weighted_norm(self._waypoints[i] - self._waypoints[i - 1])
            if dist < SMALL:
                continue
            waypoints_1.append(self._waypoints[i])
        if len(waypoints_1) < 2:
            self.logger.error(
                "Num of waypoints without repeat points should > 2!")
            return False, None

        # Dense waypoints
        threshold = weighted_norm(np.deg2rad([1.] *
                                             self._waypoints[0].shape[0]))
        for i in range(1, len(waypoints_1)):
            dist = weighted_norm(waypoints_1[i] - waypoints_1[i - 1])
            if dist < threshold:
                self.logger.error("Dense waypoints are not supported!")
                return False, None

        # Colinear waypoints
        waypoints_2 = [waypoints_1[0]]
        for i in range(1, len(waypoints_1) - 1):
            wp_pre = waypoints_2[-1]
            wp_cur = waypoints_1[i]
            wp_nxt = waypoints_1[i + 1]
            v1 = wp_cur - wp_pre
            v2 = wp_nxt - wp_cur
            proj = np.dot(v2, v1) / np.dot(v1, v1) * v1
            if weighted_norm(v2 - proj) <= SMALL:
                if np.dot(v1, v2) > EPSILON:
                    continue
                # else:
                #     self.logger.error(f"Antiparallel are not supported! Please check waypoint[{i-1}:{i+2}]")
                #     return False, None
            waypoints_2.append(wp_cur)
        waypoints_2.append(waypoints_1[-1])

        return True, waypoints_2

    def calc_path(self,
                  path_type: PathType = PathType.Bezier5thJoint,
                  blend_tolerance: float = 0.005) -> bool:
        if path_type not in [PathType.Bezier2ndJoint, PathType.Bezier5thJoint]:
            self.logger.error(
                f"Only support [PathType.Bezier2ndJoint, PathType.Bezier5thJoint]!"
            )
            return False

        self.is_init = False
        self._path_seg_list: list[PathSegRnBezier] = []

        res, waypoints = self._pre_process()
        if not res:
            return False

        if blend_tolerance <= SMALL:
            cur_length = 0.
            wp0 = waypoints[0]
            for wp1 in waypoints[1:]:
                linear_path_seg = PathSegRnBezier(control_points=np.array(
                    [wp0, wp1]),
                                                  s0_global=cur_length)
                self._path_seg_list.append(linear_path_seg)
                cur_length += linear_path_seg.get_length()
                wp0 = wp1
        else:
            cur_length = 0.
            wp0 = waypoints[0]
            wp1 = waypoints[1]
            for wp2 in waypoints[2:]:
                ls = weighted_norm(wp1 - wp0)
                le = weighted_norm(wp2 - wp1)
                dist = 4 * blend_tolerance / weighted_norm(wp2 - wp0)
                dist = min(min(dist, ls / 2.), le / 2.)

                # linear part
                linear_seg_wp0 = wp0
                ts = (wp1 - wp0) / ls
                linear_seg_wp1 = wp1 + (-dist * ts)
                linear_path_seg = PathSegRnBezier(control_points=np.array(
                    [linear_seg_wp0, linear_seg_wp1]),
                                                  s0_global=cur_length)
                self._path_seg_list.append(linear_path_seg)
                cur_length += linear_path_seg.get_length()

                # bezier part
                bezier_seg_wp0 = linear_seg_wp1
                bezier_seg_wp1 = wp1
                te = (wp2 - wp1) / le
                bezier_seg_wp2 = wp1 + dist * te
                sub_waypoints = np.array(
                    [bezier_seg_wp0, bezier_seg_wp1, bezier_seg_wp2])
                if path_type == PathType.Bezier2ndJoint:
                    bezier_path_seg = PathSegRnBezier(
                        control_points=sub_waypoints, s0_global=cur_length)

                elif path_type == PathType.Bezier5thJoint:
                    res, control_points, length = calc_5th_bezier_Rn_control_points(
                        sub_waypoints)
                    if not res:
                        self.logger.error(f"{path_type} path compute failed!")
                        return False
                    bezier_path_seg = PathSegRnBezier(
                        control_points=control_points,
                        length=length,
                        s0_global=cur_length)
                else:
                    self.logger.error(f"{path_type} is not supported!")
                    return False

                self._path_seg_list.append(bezier_path_seg)
                cur_length += bezier_path_seg.get_length()

                wp0 = bezier_seg_wp2
                wp1 = wp2

            # the last linear part
            linear_seg_wp0 = wp0
            linear_seg_wp1 = wp1
            linear_path_seg = PathSegRnBezier(np.array(
                [linear_seg_wp0, linear_seg_wp1]),
                                              s0_global=cur_length)
            self._path_seg_list.append(linear_path_seg)

        self._length = 0.
        for path_seg in self._path_seg_list:
            self._length += path_seg.get_length()

        self.is_init = True
        return True

    def get_sample_by_step(self, step: float) -> np.ndarray:
        if not self.is_init:
            self.logger.error(f"Not init, call calc_path() first!")
            return None
        if step >= self.get_length():
            self.logger.error(f"step should < {self.get_length()}")
            return None
        if step <= 0.:
            self.logger.error(f"step should > 0")
            return None

        s_vals = np.arange(0, self.get_length(), step)
        if s_vals[-1] < self.get_length():
            s_vals = np.append(s_vals, self.get_length())

        sample_list = [self._path_seg_list[0].get_config(0.).tolist()]
        for path_seg in self._path_seg_list:
            s0 = path_seg.get_s0()
            s1 = s0 + path_seg.get_length()
            sub_s_vals = [s for s in s_vals if s > s0 and s < s1]
            if len(sub_s_vals) == 0:
                continue
            sample_list += path_seg.get_config(sub_s_vals).tolist()
        sample_list += [
            self._path_seg_list[-1].get_config(self.get_length()).tolist()
        ]
        return np.array(sample_list)

    def get_sample_by_num(self, num: int) -> np.ndarray:
        if not self.is_init:
            self.logger.error(f"Not init, call calc_path() first!")
            return None
        if num < 2:
            self.logger.error(f"num should >= 2")
            return None

        sample_list = [self._path_seg_list[0].get_config(0.).tolist()]
        s_vals = np.linspace(0, self.get_length(), num)
        for path_seg in self._path_seg_list:
            s0 = path_seg.get_s0()
            s1 = s0 + path_seg.get_length()
            sub_s_vals = [s for s in s_vals if s > s0 and s < s1]
            if len(sub_s_vals) == 0:
                continue
            sample_list += path_seg.get_config(sub_s_vals).tolist()
        sample_list += [
            self._path_seg_list[-1].get_config(self.get_length()).tolist()
        ]
        return np.array(sample_list)


class PathR3xSO3Bezier(PathBezier):

    def __init__(self, waypoints: np.ndarray):
        super().__init__(waypoints)

    def _pre_process(self) -> tuple[bool, np.ndarray]:
        if len(self._waypoints) < 2:
            self.logger.error("Num of waypoints should > 2!")
            return False, None

        # Repeat waypoints
        waypoints_1 = [self._waypoints[0]]
        for i in range(1, len(self._waypoints)):
            dist = weighted_norm(
                pin.log6(
                    np.linalg.inv(self._waypoints[i - 1])
                    @ self._waypoints[i]).vector,
                np.diag([1., 1., 1., 2., 2., 2.]))
            if dist < SMALL:
                continue
            waypoints_1.append(self._waypoints[i])
        if len(waypoints_1) < 2:
            self.logger.error(
                "Num of waypoints without repeat points should > 2!")
            return False, None

        # Dense waypoints
        for i in range(1, len(waypoints_1)):
            dist = weighted_norm(
                pin.log6(
                    np.linalg.inv(waypoints_1[i - 1]) @ waypoints_1[i]).vector,
                np.diag([1., 1., 1., 2., 2., 2.]))
            if dist < 0.03:
                self.logger.error("Dense waypoints are not supported!")
                return False, None

        # Colinear waypoints
        waypoints_2 = [waypoints_1[0]]
        for i in range(1, len(waypoints_1) - 1):
            wp_pre = waypoints_2[-1]
            wp_cur = waypoints_1[i]
            wp_nxt = waypoints_1[i + 1]
            v1 = pin.log6(np.linalg.inv(wp_pre) @ wp_cur)
            v2 = pin.log6(np.linalg.inv(wp_cur) @ wp_nxt)
            v1_R3 = v1.linear
            v2_R3 = v2.linear
            v1_SO3 = v1.angular
            v2_SO3 = v2.angular
            proj = np.dot(v2_R3, v1_R3) / np.dot(v1_R3, v1_R3) * v1_R3
            if weighted_norm(v2_R3 - proj) <= SMALL:
                if np.dot(v1_R3, v2_R3) < EPSILON:
                    self.logger.error("Antiparallel are not supported!")
                    return False, None
                if weighted_norm(v1_SO3) < SMALL and weighted_norm(
                        v2_SO3) < SMALL:
                    continue
            waypoints_2.append(wp_cur)
        waypoints_2.append(waypoints_1[-1])

        return True, waypoints_2

    def calc_path(self,
                  path_type: PathType = PathType.Bezier5thCartesian,
                  blend_tolerance: float = 0.005) -> bool:
        if path_type not in [
                PathType.Bezier2ndCartesian, PathType.Bezier5thCartesian
        ]:
            self.logger.error(
                f"Only support [PathType.Bezier2ndCartesian, PathType.Bezier5thCartesian]!"
            )
            return False

        self.is_init = False
        self._path_seg_list: list[PathSegR3xSO3Bezier] = []

        res, waypoints = self._pre_process()
        if not res:
            return False

        if blend_tolerance <= SMALL:
            cur_length = 0.
            wp0 = waypoints[0]
            for wp1 in waypoints[1:]:
                linear_path_seg = PathSegR3xSO3Bezier(control_points=np.array(
                    [wp0, wp1]),
                                                      s0_global=cur_length)
                self._path_seg_list.append(linear_path_seg)
                cur_length += linear_path_seg.get_length()
                wp0 = wp1
        else:
            cur_length = 0.
            wp0 = waypoints[0]
            wp1 = waypoints[1]
            for wp2 in waypoints[2:]:
                ls = weighted_norm(wp1[:3, 3] - wp0[:3, 3])
                le = weighted_norm(wp2[:3, 3] - wp1[:3, 3])
                dist = 4 * blend_tolerance / weighted_norm(wp2[:3, 3] -
                                                           wp0[:3, 3])
                dist = min(min(dist, ls / 2.), le / 2.)

                # linear part
                linear_seg_wp0 = wp0
                R3_wp0 = wp0[:3, 3]
                SO3_wp0 = wp0[:3, :3]
                R3_wp1 = wp1[:3, 3]
                SO3_wp1 = wp1[:3, :3]
                linear_seg_wp1 = np.eye(4)
                linear_seg_wp1[:3,
                               3] = R3_wp1 + (-dist / ls) * (R3_wp1 - R3_wp0)
                linear_seg_wp1[:3, :3] = SO3_wp1 @ pin.exp3(
                    -dist / ls * pin.log3(SO3_wp0.T @ SO3_wp1))

                linear_path_seg = PathSegR3xSO3Bezier(control_points=np.array(
                    [linear_seg_wp0, linear_seg_wp1]),
                                                      s0_global=cur_length)
                self._path_seg_list.append(linear_path_seg)
                cur_length += linear_path_seg.get_length()

                # bezier part
                bezier_seg_wp0 = linear_seg_wp1
                bezier_seg_wp1 = wp1
                R3_wp1 = wp1[:3, 3]
                SO3_wp1 = wp1[:3, :3]
                R3_wp2 = wp2[:3, 3]
                SO3_wp2 = wp2[:3, :3]
                bezier_seg_wp2 = np.eye(4)
                bezier_seg_wp2[:3, 3] = R3_wp1 + dist / le * (R3_wp2 - R3_wp1)
                bezier_seg_wp2[:3, :3] = SO3_wp1 @ pin.exp3(
                    dist / le * pin.log3(SO3_wp1.T @ SO3_wp2))
                waypoints = np.array(
                    [bezier_seg_wp0, bezier_seg_wp1, bezier_seg_wp2])

                if path_type == PathType.Bezier2ndCartesian:
                    bezier_path_seg = PathSegR3xSO3Bezier(
                        control_points=waypoints, s0_global=cur_length)

                elif path_type == PathType.Bezier5thCartesian:
                    R3_waypoints = np.array([SE3[:3, 3] for SE3 in waypoints])
                    SO3_waypoints = np.array([SE3[:3, :3] for SE3 in waypoints])
                    res1, R3_control_points, length1 = calc_5th_bezier_Rn_control_points(
                        R3_waypoints)
                    res2, SO3_control_points, length2 = calc_5th_bezier_SO3_control_points(
                        SO3_waypoints)
                    if not res1 or not res2:
                        self.logger.error(f"{path_type} path compute failed!")
                        return False
                    control_points = []
                    for R3, SO3 in zip(R3_control_points, SO3_control_points):
                        control_point = np.eye(4)
                        control_point[:3, 3] = R3
                        control_point[:3, :3] = SO3
                        control_points.append(control_point)

                    bezier_path_seg = PathSegR3xSO3Bezier(
                        control_points=np.array(control_points),
                        R3_length=length1,
                        SO3_length=length2,
                        s0_global=cur_length)
                else:
                    self.logger.error(f"{path_type} is not supported!")
                    return False

                self._path_seg_list.append(bezier_path_seg)
                cur_length += bezier_path_seg.get_length()

                wp0 = bezier_seg_wp2
                wp1 = wp2

            # the last linear part
            linear_seg_wp0 = wp0
            linear_seg_wp1 = wp1
            linear_path_seg = PathSegR3xSO3Bezier(np.array(
                [linear_seg_wp0, linear_seg_wp1]),
                                                  s0_global=cur_length)
            self._path_seg_list.append(linear_path_seg)

        self._length = 0.
        for path_seg in self._path_seg_list:
            self._length += path_seg.get_length()

        self.is_init = True
        return True
