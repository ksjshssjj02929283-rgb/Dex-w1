# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from scipy.special import comb
from abc import ABC, abstractmethod
from enum import Enum, auto

from ...common import get_logger

SMALL = 1e-5


def weighted_norm(vec: np.ndarray, weights=None):
    if weights is None:
        weights = np.ones(vec.ndim)
    return np.sqrt(np.sum(weights * vec**2))


def bernstein_basis(n: int, s: float | np.ndarray) -> np.ndarray:
    """(n+1,) or (m, n+1)"""
    k = np.arange(n + 1)
    if isinstance(s, np.ndarray):
        s_ = s.reshape(-1, 1)
        return comb(n, np.arange(n + 1)) * (s_**k) * (1. - s_)**(n - k)
    return comb(n, np.arange(n + 1)) * (s**k) * (1. - s)**(n - k)


class PathType(Enum):
    Bezier2ndJoint = auto()
    Bezier5thJoint = auto()


class PathSegmentBase(ABC):

    def __init__(self, length: float, s0_global=0.):
        self._s0 = s0_global
        self._length = length

    def _s_local(self, s: float | np.ndarray) -> float:
        if self._length < SMALL:
            if isinstance(s, np.ndarray):
                return np.zeros_like(s)
            return 0.
        s_ = np.asarray(s) - self._s0
        s_ = np.clip(s_, 0, self._length)
        return s_ / self._length

    def get_s0(self) -> float:
        return self._s0

    def get_length(self) -> float:
        return self._length

    @abstractmethod
    def get_config(self, s: float | np.ndarray) -> np.ndarray:
        """p(s)"""
        pass

    @abstractmethod
    def get_tangent(self, s: float | np.ndarray) -> np.ndarray:
        """dp(s)"""
        pass

    @abstractmethod
    def get_curvature(self, s: float | np.ndarray) -> np.ndarray:
        """ddp(s)"""
        pass

    @abstractmethod
    def get_torsion(self, s: float | np.ndarray) -> np.ndarray:
        """dddp(s)"""
        pass


class PathSegRnBezier(PathSegmentBase):

    @staticmethod
    def Rn_bezier(control_points: np.ndarray,
                  s: float | np.ndarray) -> np.ndarray:
        """control_points: (n+1, dof)
            -> (dof,) or (m, dof)
        """
        n = len(control_points) - 1
        B = bernstein_basis(n, s)
        return B @ control_points

    @staticmethod
    def Rn_bezier_derivative(control_points: np.ndarray,
                             s: float | np.ndarray,
                             order=1) -> np.ndarray:
        """control_points: (n+1, dof)
            -> (dof,) or (m, dof)
        """
        n = len(control_points) - 1
        cp = control_points.copy()
        coeff = 1.
        for r in range(order):
            cp = np.diff(cp, axis=0)  # forward diff (n-r, dof)
            coeff *= (n - r)
        B = bernstein_basis(n - order, s)
        return coeff * (B @ cp)

    def __init__(self,
                 control_points: np.ndarray,
                 length: float = None,
                 s0_global: float = 0.):
        self._control_points = control_points
        if length is None:
            length = 0.
            for i in range(self._control_points.shape[0] - 1):
                p0 = self._control_points[i]
                p1 = self._control_points[i + 1]
                length += weighted_norm(p1 - p0)
        super().__init__(length, s0_global)

    def get_config(self, s: float | np.ndarray) -> np.ndarray:
        s_local = self._s_local(s)
        return PathSegRnBezier.Rn_bezier(self._control_points, s_local)

    def get_tangent(self, s: float | np.ndarray) -> np.ndarray:
        s_local = self._s_local(s)
        return PathSegRnBezier.Rn_bezier_derivative(
            self._control_points, s_local, order=1) / self._length

    def get_curvature(self, s: float | np.ndarray) -> np.ndarray:
        s_local = self._s_local(s)
        return PathSegRnBezier.Rn_bezier_derivative(
            self._control_points, s_local, order=2) / (self._length**2)

    def get_torsion(self, s: float | np.ndarray) -> np.ndarray:
        s_local = self._s_local(s)
        return PathSegRnBezier.Rn_bezier_derivative(
            self._control_points, s_local, order=3) / (self._length**3)


def calc_5th_bezier_control_points(
        waypoints: np.ndarray,
        ts_norm: float = 1.,
        te_norm: float = 1.,
        cs_norm: float = 0.,
        ce_norm: float = 0.) -> tuple[bool, np.ndarray, float]:
    if len(waypoints) != 3:
        get_logger().error(f"Num of waypoint != 3")
        return False, None, None

    temp0 = waypoints[1] - waypoints[0]
    temp1 = waypoints[2] - waypoints[1]
    ts_normal = temp0 / weighted_norm(temp0)
    te_normal = temp1 / weighted_norm(temp1)
    # 曲率很小时，用单位切线近似
    cs_normal = ts_normal
    ce_normal = te_normal
    ts = ts_norm * ts_normal  # tangent_start
    te = te_norm * te_normal  # tangen_end
    cs = cs_norm * cs_normal  # curvature_start
    ce = ce_norm * ce_normal  # curvature_end

    ps = waypoints[0]
    pe = waypoints[2]
    # Solve constraint
    a = 256 - 49 * weighted_norm(ts + te)**2
    b = 420 * np.dot(pe - ps, ts + te)
    c = -900 * weighted_norm(pe - ps)**2
    delta = b**2 - 4 * a * c
    if delta < 0:
        get_logger().error(f"Calcuate control points failed!")
        return False, None, None
    r = max((-b + np.sqrt(delta)) / (2 * a), (-b - np.sqrt(delta)) / (2 * a))

    p0 = ps
    p1 = p0 + 0.2 * r * ts
    p2 = 2 * p1 - p0 + 0.05 * r * r * cs
    p5 = pe
    p4 = p5 - 0.2 * r * te
    p3 = 2 * p4 - p5 + 0.05 * r * r * ce

    control_points = np.array([p0, p1, p2, p3, p4, p5])
    length = r
    return True, control_points, length


class PathRnBezier:

    def __init__(self, waypoints: np.ndarray, blend_tolerance: float = 0.005):
        self._waypoints = waypoints
        self._blend_tolerance = blend_tolerance
        self._path_seg_list: list[PathSegmentBase] = []
        self._length = 0.
        self.is_init = False

    def calc_path(self, path_type: PathType = PathType.Bezier5thJoint) -> bool:
        self.is_init = False
        self._path_seg_list: list[PathSegmentBase] = []

        cur_length = 0.
        wp0 = self._waypoints[0]
        wp1 = self._waypoints[1]
        for wp2 in self._waypoints[2:]:
            ls = weighted_norm(wp1 - wp0)
            le = weighted_norm(wp2 - wp1)
            dist = 4 * self._blend_tolerance / weighted_norm(wp2 - wp0)
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
            waypoints = np.array(
                [bezier_seg_wp0, bezier_seg_wp1, bezier_seg_wp2])
            if path_type == PathType.Bezier2ndJoint:
                bezier_path_seg = PathSegRnBezier(control_points=waypoints,
                                                  s0_global=cur_length)

            elif path_type == PathType.Bezier5thJoint:
                res, control_points, length = calc_5th_bezier_control_points(
                    waypoints)
                if not res:
                    get_logger().error(f"{path_type} path compute failed!")
                    return False
                bezier_path_seg = PathSegRnBezier(control_points=control_points,
                                                  length=length,
                                                  s0_global=cur_length)
            else:
                get_logger().error(f"{path_type} is not supported!")
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

    def get_waypoints(self) -> np.ndarray:
        return self._waypoints

    def get_blend_tolerance(self) -> float:
        return self._blend_tolerance

    def get_length(self) -> float:
        if not self.is_init:
            get_logger().error(f"Not init, call calc_path() first!")
        return self._length

    def get_path_segments(self) -> list[PathSegmentBase]:
        if not self.is_init:
            get_logger().error(f"Not init, call calc_path() first!")
        return self._path_seg_list

    def _get_path_segment_at_s(self, s: float) -> PathSegmentBase:
        if not self.is_init:
            get_logger().error(f"Not init, call calc_path() first!")
        s_ = max(0, min(s, self._length))
        for path_seg in self._path_seg_list:
            ss = path_seg.get_s0()
            se = ss + path_seg.get_length()
            if s_ <= se:
                return path_seg

    def get_config(self, s: float) -> np.ndarray:
        if not self.is_init:
            get_logger().error(f"Not init, call calc_path() first!")
            return None
        path_seg = self._get_path_segment_at_s(s)
        return path_seg.get_config(s)

    def get_tangent(self, s: float) -> np.ndarray:
        if not self.is_init:
            get_logger().error(f"Not init, call calc_path() first!")
            return None
        path_seg = self._get_path_segment_at_s(s)
        return path_seg.get_tangent(s)

    def get_curvature(self, s: float) -> np.ndarray:
        if not self.is_init:
            get_logger().error(f"Not init, call calc_path() first!")
            return None
        path_seg = self._get_path_segment_at_s(s)
        return path_seg.get_curvature(s)

    def get_torsion(self, s: float) -> np.ndarray:
        if not self.is_init:
            get_logger().error(f"Not init, call calc_path() first!")
            return None
        path_seg = self._get_path_segment_at_s(s)
        return path_seg.get_torsion(s)

    def get_sample_by_step(self, step: float):
        if not self.is_init:
            get_logger().error(f"Not init, call calc_path() first!")
            return None
        if step >= self.get_length():
            get_logger().error(f"step should < {self.get_length()}")
            return None

        sample_list = [self._path_seg_list[0].get_config(0.).tolist()]

        s_vals = np.arange(0, self.get_length(), step)
        if s_vals[-1] < self.get_length():
            s_vals = np.append(s_vals, self.get_length())

        for path_seg in self._path_seg_list:
            s0 = path_seg.get_s0()
            s1 = s0 + path_seg.get_length()
            sub_s_vals = [s for s in s_vals if s > s0 and s < s1]
            sample_list += path_seg.get_config(sub_s_vals).tolist()
        sample_list = [
            self._path_seg_list[-1].get_config(self.get_length()).tolist()
        ]
        return np.array(sample_list)

    def get_sample_by_num(self, num: int):
        if not self.is_init:
            get_logger().error(f"Not init, call calc_path() first!")
            return None

        sample_list = [self._path_seg_list[0].get_config(0.).tolist()]
        s_vals = np.linspace(0, self.get_length(), num)
        for path_seg in self._path_seg_list:
            s0 = path_seg.get_s0()
            s1 = s0 + path_seg.get_length()
            sub_s_vals = [s for s in s_vals if s > s0 and s < s1]
            sample_list += path_seg.get_config(sub_s_vals).tolist()
        sample_list += [
            self._path_seg_list[-1].get_config(self.get_length()).tolist()
        ]
        return np.array(sample_list)
