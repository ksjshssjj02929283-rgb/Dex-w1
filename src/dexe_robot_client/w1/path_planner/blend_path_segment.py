'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
import pinocchio as pin
from scipy.special import comb
from abc import ABC, abstractmethod

from ...common import get_logger

SMALL = 1e-3
EPSILON = 1e-8


def weighted_norm(vec: np.ndarray, weights=None):
    if weights is None:
        weights = np.eye(vec.shape[0])
    return np.sqrt(vec.T @ weights @ vec)


def bernstein_basis(n: int, s: float | np.ndarray) -> np.ndarray:
    """(n+1,) or (m, n+1)"""
    k = np.arange(n + 1)
    if isinstance(s, np.ndarray):
        s_ = s.reshape(-1, 1)
        return comb(n, np.arange(n + 1)) * (s_**k) * (1. - s_)**(n - k)
    return comb(n, np.arange(n + 1)) * (s**k) * (1. - s)**(n - k)


def calc_5th_bezier_Rn_control_points(
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


def calc_5th_bezier_SO3_control_points(
        waypoints: np.ndarray,
        ts_norm: float = 1.,
        te_norm: float = 1.,
        cs_norm: float = 0.,
        ce_norm: float = 0.) -> tuple[bool, np.ndarray, float]:
    if len(waypoints) != 3:
        get_logger().error(f"Num of waypoint != 3")
        return False, None, None

    temp0 = pin.log3(waypoints[0].T @ waypoints[1])
    temp1 = pin.log3(waypoints[1].T @ waypoints[2])
    if weighted_norm(temp0, 2. * np.eye(3)) < EPSILON:
        ts_normal = temp0
    else:
        ts_normal = temp0 / weighted_norm(temp0, 2. * np.eye(3))
    if weighted_norm(temp1, 2. * np.eye(3)) < EPSILON:
        te_normal = temp1
    else:
        te_normal = temp1 / weighted_norm(temp1, 2. * np.eye(3))
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
    a = 256 - 49 * weighted_norm(ts + te, 2. * np.eye(3))**2
    b = 420 * np.dot(pin.log3(ps.T @ pe), ts + te)
    c = -900 * weighted_norm(pin.log3(ps.T @ pe), 2. * np.eye(3))**2
    delta = b**2 - 4 * a * c
    if delta < 0:
        get_logger().error(f"Calcuate control points failed!")
        return False, None, None
    r = max((-b + np.sqrt(delta)) / (2 * a), (-b - np.sqrt(delta)) / (2 * a))

    p0 = ps
    p1 = p0 @ pin.exp3(0.2 * r * ts)
    p2 = p1 @ pin.exp3(pin.log3(p0.T @ p1) + 0.05 * r * r * cs)
    p5 = pe
    p4 = p5 @ pin.exp3(-0.2 * r * te)
    p3 = p4 @ pin.exp3(pin.log3(p5.T @ p4) + 0.05 * r * r * ce)

    control_points = np.array([p0, p1, p2, p3, p4, p5])
    length = r
    return True, control_points, length


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


class PathSegSO3Bezier(PathSegmentBase):

    @staticmethod
    def SO3_bezier(control_points: np.ndarray, s: float) -> np.ndarray:
        """control_points: (n+1, dof)
            -> (dof,) or (m, dof)
        """
        n = len(control_points) - 1
        SO3_list = control_points.copy()
        for k in range(1, n + 1):
            SO3_list = [
                SO3_list[i] @ pin.exp3(
                    s * pin.log3(SO3_list[i].T @ SO3_list[i + 1]))
                for i in range(n + 1 - k)
            ]
        return SO3_list[0]

    @staticmethod
    def SO3_bezier_derivative(control_points: np.ndarray,
                              s: float,
                              order=1) -> np.ndarray:
        SO3_ = np.array([0., 0., 0.])
        n = len(control_points) - 1
        if n + 1 - order < 1:
            return SO3_

        diff = control_points.copy()
        B = bernstein_basis(n - order, s)
        coeff = 1.
        for r in range(order):
            if r == 0:
                diff = [
                    pin.log3(diff[i - 1].T @ diff[i])
                    for i in range(1, len(diff))
                ]
            else:
                diff = [diff[i] - diff[i - 1] for i in range(1, len(diff))]
            coeff *= (n - r)
        for B_k, diff_k in zip(B, diff):
            SO3_ = SO3_ + coeff * B_k * diff_k
        return SO3_

    def __init__(self,
                 control_points: np.ndarray,
                 length: float = None,
                 s0_global: float = 0.):
        """control_points -> (n,3,3)"""
        self._control_points = control_points
        if length is None:
            length = 0.
            for i in range(self._control_points.shape[0] - 1):
                SO3_0 = self._control_points[i]
                SO3_1 = self._control_points[i + 1]
                # align to manif
                length += weighted_norm(pin.log3(SO3_0.T @ SO3_1),
                                        2. * np.eye(3))
        super().__init__(length, s0_global)

    def get_config(self, s: float) -> np.ndarray:
        s_local = self._s_local(s)
        return PathSegSO3Bezier.SO3_bezier(self._control_points, s_local)

    def get_tangent(self, s: float) -> np.ndarray:
        s_local = self._s_local(s)
        if self._length < EPSILON:
            return PathSegSO3Bezier.SO3_bezier_derivative(self._control_points,
                                                          s_local,
                                                          order=1)
        return PathSegSO3Bezier.SO3_bezier_derivative(
            self._control_points, s_local, order=1) / self._length

    def get_curvature(self, s: float) -> np.ndarray:
        s_local = self._s_local(s)
        if self._length < EPSILON:
            return PathSegSO3Bezier.SO3_bezier_derivative(self._control_points,
                                                          s_local,
                                                          order=2)
        return PathSegSO3Bezier.SO3_bezier_derivative(
            self._control_points, s_local, order=2) / (self._length**2)

    def get_torsion(self, s: float) -> np.ndarray:
        s_local = self._s_local(s)
        if self._length < EPSILON:
            return PathSegSO3Bezier.SO3_bezier_derivative(self._control_points,
                                                          s_local,
                                                          order=3)
        return PathSegSO3Bezier.SO3_bezier_derivative(
            self._control_points, s_local, order=3) / (self._length**3)


class PathSegR3xSO3Bezier(PathSegmentBase):

    def __init__(self,
                 control_points: np.ndarray,
                 R3_length: float = None,
                 SO3_length: float = None,
                 s0_global: float = 0.):
        """control_points -> (n,4,4)"""
        self._control_points = control_points
        self._R3_control_points = np.array(
            [SE3[:3, 3] for SE3 in self._control_points])
        self._SO3_control_points = np.array(
            [SE3[:3, :3] for SE3 in self._control_points])
        self._R3_path_seg = PathSegRnBezier(self._R3_control_points, R3_length,
                                            s0_global)
        self._SO3_path_seg = PathSegSO3Bezier(self._SO3_control_points,
                                              SO3_length, s0_global)
        self._R3_major = True
        length = self._R3_path_seg.get_length()
        length2 = self._SO3_path_seg.get_length()
        if length < 1e-4 and length2 > 1e-4:
            length = length2
            self._R3_major = False
        super().__init__(length, s0_global)

    def get_config(self, s: float) -> np.ndarray:
        SE3 = np.eye(4)
        if self._R3_major:
            s_R3 = s
            s_SO3 = self._SO3_path_seg.get_s0(
            ) + self._s_local(s) * self._SO3_path_seg.get_length()
        else:
            s_R3 = self._R3_path_seg.get_s0(
            ) + self._s_local(s) * self._R3_path_seg.get_length()
            s_SO3 = s
        SE3[:3, 3] = self._R3_path_seg.get_config(s_R3)
        SE3[:3, :3] = self._SO3_path_seg.get_config(s_SO3)
        return SE3

    def get_tangent(self, s: float) -> np.ndarray:
        SE3Tangent = np.zeros(6)
        if self._R3_major:
            s_R3Tangent = s
            s_SO3Tangent = self._SO3_path_seg.get_s0(
            ) + self._s_local(s) * self._SO3_path_seg.get_length()
        else:
            s_R3Tangent = self._R3_path_seg.get_s0(
            ) + self._s_local(s) * self._R3_path_seg.get_length()
            s_SO3Tangent = s
        SE3Tangent[:3] = self._R3_path_seg.get_tangent(s_R3Tangent)
        SE3Tangent[3:] = self._SO3_path_seg.get_tangent(s_SO3Tangent)
        return SE3Tangent

    def get_curvature(self, s: float) -> np.ndarray:
        SE3Tangent = np.zeros(6)
        if self._R3_major:
            s_R3Tangent = s
            s_SO3Tangent = self._SO3_path_seg.get_s0(
            ) + self._s_local(s) * self._SO3_path_seg.get_length()
        else:
            s_R3Tangent = self._R3_path_seg.get_s0(
            ) + self._s_local(s) * self._R3_path_seg.get_length()
            s_SO3Tangent = s
        SE3Tangent[:3] = self._R3_path_seg.get_curvature(s_R3Tangent)
        SE3Tangent[3:] = self._SO3_path_seg.get_curvature(s_SO3Tangent)
        return SE3Tangent

    def get_torsion(self, s: float) -> np.ndarray:
        SE3Tangent = np.zeros(6)
        if self._R3_major:
            s_R3Tangent = s
            s_SO3Tangent = self._SO3_path_seg.get_s0(
            ) + self._s_local(s) * self._SO3_path_seg.get_length()
        else:
            s_R3Tangent = self._R3_path_seg.get_s0(
            ) + self._s_local(s) * self._R3_path_seg.get_length()
            s_SO3Tangent = s
        SE3Tangent[:3] = self._R3_path_seg.get_torsion(s_R3Tangent)
        SE3Tangent[3:] = self._SO3_path_seg.get_torsion(s_SO3Tangent)
        return SE3Tangent
