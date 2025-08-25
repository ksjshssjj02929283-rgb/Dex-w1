# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from copy import deepcopy

from ...common import get_logger, W1ArmType
from ..utils.w1_params import W1ArmKineParams


class SRSKinematics:
    def __init__(self, kine_params: W1ArmKineParams):
        self.logger = get_logger()

        self.dofs = 7

        self._kine_params = kine_params
        self._d_bs = self._kine_params.d_bs
        self._d_se = self._kine_params.d_se
        self._d_ew = self._kine_params.d_ew
        self._d_wt = self._kine_params.d_wt
        self._v_0_bs = np.array([0., 0., self._d_bs])
        self._v_6_wt = np.array([0., 0., self._d_wt])

        self._tcp = np.eye(4)
        self._joint_lb = self._kine_params.qpos_limits[:,0]
        self._joint_ub = self._kine_params.qpos_limits[:,1]
        self._user_joint_lb = self._kine_params.qpos_limits[:,0]
        self._user_joint_ub = self._kine_params.qpos_limits[:,1]
        self._joint_weights = np.array([1., 1., 1., 1., 1., 1., 1.])

        self._qpos_lb = None
        self._qpos_ub = None
        self._update_qpos_limits()

    def _dh_transform(self, a, alpha, d, theta) -> np.ndarray:
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([[ct, -st*ca, st*sa, a*ct],
                         [st, ct*ca, -ct*sa, a*st],
                         [0., sa, ca, d],
                         [0., 0., 0., 1.]])
    
    def _skew(self, vec: np.ndarray) -> np.ndarray:
        return np.array([[0., -vec[2], vec[1]],
                         [vec[2], 0., -vec[0]],
                         [-vec[1], vec[0], 0.]])

    def _calc_ref_angle(self, 
                        xpos: np.ndarray, 
                        elbow_config: float) -> np.ndarray:
        """Notes: xpos without tcp"""
        # xpos_ = xpos @ np.linalg.inv(self._tcp) @ np.linalg.inv(self._w1_params.T_origin_tcp) 
        p_d = xpos[:3,3]
        R_d = xpos[:3,:3]

        v_0_sw = p_d - self._v_0_bs - R_d @ self._v_6_wt
        d_sw = np.linalg.norm(v_0_sw)

        # NOTE: 判断共线，避免shoulder关节奇异
        if np.allclose(np.cross(v_0_sw, np.array([0.,0.,1.])), 0., atol=1e-6): 
            shoulder_singular = True
        else:
            shoulder_singular = False

        # FIXME 这里q1_ref,q2_ref应该有4组参考值，但是无论哪种都能计算到最终结果?
        if not shoulder_singular:
            q1_ref = np.arctan2(v_0_sw[1], v_0_sw[0]) # [-oo,+oo] -> [-pi,pi]
        else:
            q1_ref = 0.
        cos_esw = (self._d_se**2 + d_sw**2 - self._d_ew**2) / (2 * self._d_se * d_sw)
        angle_esw = np.arccos(np.clip(cos_esw, -1.0, 1.0))
        angle_wsz = np.arctan2(np.linalg.norm(v_0_sw[:2]), v_0_sw[2])
        q2_ref = angle_wsz - elbow_config * angle_esw
        q3_ref = 0. - self._kine_params.dh_params[2][3] # q3 = 0，臂角 psi = 0
        return np.array([q1_ref, q2_ref, q3_ref])

    def _calc_arm_angle(self, qpos: np.ndarray) -> tuple[bool, float]:
        res_fk, xpos = self.get_fk(qpos)
        if not res_fk:
            return False, None
        xpos_ = xpos @ np.linalg.inv(self._tcp) @ np.linalg.inv(self._kine_params.T_origin_tcp) 
        p0_07 = xpos_[:3,3]
        R0_07 = xpos_[:3,:3]

        v26 = p0_07 - self._v_0_bs - R0_07 @ self._v_6_wt
        uv26 = v26 / np.linalg.norm(v26)

        # 求参考臂角平面的法向
        q4 = qpos[3]
        q_ref = self._calc_ref_angle(xpos_, np.sign(q4))
        T02_ref = None
        T04_ref = np.eye(4)
        for i, q in enumerate(np.append(q_ref, q4)):
            T04_ref = T04_ref @ self._dh_transform(self._kine_params.dh_params[i][0],
                                                self._kine_params.dh_params[i][1],
                                                self._kine_params.dh_params[i][2],
                                                self._kine_params.dh_params[i][3] + q)
            if i == 1:
                T02_ref = deepcopy(T04_ref)
        p02_ref = T02_ref[:3,3]
        p04_ref = T04_ref[:3,3]
        v24_ref = p04_ref - p02_ref
        uv24_ref = v24_ref / np.linalg.norm(v24_ref)
        v_ref_plane = np.cross(uv24_ref, uv26)
        uv_ref_plane = v_ref_plane / np.linalg.norm(v_ref_plane)

        # 求真实臂角平面的法向
        T02 = None
        T04 = np.eye(4)
        for i, q in enumerate(qpos[:4]):
            T04 = T04 @ self._dh_transform(self._kine_params.dh_params[i][0],
                                        self._kine_params.dh_params[i][1],
                                        self._kine_params.dh_params[i][2],
                                        self._kine_params.dh_params[i][3] + q)
            if i == 1:
                T02 = deepcopy(T04)
        p02 = T02[:3,3]
        p04 = T04[:3,3]
        v24 = p04 - p02
        uv24 = v24 / np.linalg.norm(v24)
        v_plane = np.cross(uv24, uv26)
        uv_plane = v_plane / np.linalg.norm(v_plane)

        psi = np.sign(np.cross(uv_ref_plane, uv_plane) @ v26) * \
            np.arccos(np.clip(uv_ref_plane @ uv_plane, -1.0, 1.0))
        return True, psi

    def _check_qpos_limits(self, qpos: np.ndarray) -> bool:
        return np.all((qpos >= self._qpos_lb) & (qpos <= self._qpos_ub))

    def _update_qpos_limits(self):
        self._qpos_lb = np.maximum(self._joint_lb, self._user_joint_lb)
        self._qpos_ub = np.minimum(self._joint_ub, self._user_joint_ub)


    def get_fk(self, qpos: np.ndarray) -> tuple[bool, np.ndarray]:
        qpos_ = np.array(qpos)
        if qpos_.shape[0] != self.dofs:
            self.logger.error(f"dimension != {self.dofs}")
            return False, None
        
        if not self._check_qpos_limits(qpos_):
            self.logger.error(f"qpos out of joint limits.")
            return False, None
        
        T = np.eye(4)
        for i, q in enumerate(qpos_):
            T = T @ self._dh_transform(self._kine_params.dh_params[i][0],
                                    self._kine_params.dh_params[i][1],
                                    self._kine_params.dh_params[i][2],
                                    self._kine_params.dh_params[i][3] + q)
        return True, T @ self._kine_params.T_origin_tcp @ self._tcp


    def get_ik(self, 
               xpos: np.ndarray, 
               arm_angle: float, 
               config: tuple,
               verbose: float = True) -> tuple[bool, np.ndarray]:
        if len(config) != 3:
            self.logger.error(f"len(config) != 3")
            return False, None
        config = np.sign(config)

        xpos_ = xpos @ np.linalg.inv(self._tcp) @ np.linalg.inv(self._kine_params.T_origin_tcp) 
        p_d = xpos_[:3,3]
        R_d = xpos_[:3,:3]

        v_0_sw = p_d - self._v_0_bs - R_d @ self._v_6_wt
        d_sw = np.linalg.norm(v_0_sw)

        # calc elbow q4
        # NOTE: 判断长度，避免elbow关节奇异
        if (self._d_se + self._d_ew) < d_sw:
            self.logger.error(f"xpos out of workspace.")
            return False, None
        cos_q4 = (d_sw**2 - self._d_se**2 - self._d_ew**2) / (2 * self._d_se * self._d_ew)
        q4 = config[1] * np.arccos(np.clip(cos_q4, -1.0, 1.0))
        R_34 = self._dh_transform(self._kine_params.dh_params[3][0], 
                                self._kine_params.dh_params[3][1], 
                                self._kine_params.dh_params[3][2],
                                self._kine_params.dh_params[3][3] + q4)[:3,:3]

        q_ref = self._calc_ref_angle(xpos_, config[1])
        T03_ref = np.eye(4)
        for i, q in enumerate(q_ref):
            T03_ref = T03_ref @ self._dh_transform(self._kine_params.dh_params[i][0],
                                                self._kine_params.dh_params[i][1],
                                                self._kine_params.dh_params[i][2],
                                                self._kine_params.dh_params[i][3] + q)
        R03_ref = T03_ref[:3,:3]

        s_psi = np.sin(arm_angle)
        c_psi = np.cos(arm_angle)

        # Calc shoulder q1 q2 q3
        """R03 
        left                     right
        [[ *    -c1s2  *]         [[ *   -c1s2  *]
        [  *    -s1s2  *]         [  *   -s1s2  *]
        [-s2s3  -c2  -s2c3]]      [s2s3  -c2  s2c3]]
        """
        # TODO: 判断shoulder关节奇异
        uv_0_sw = v_0_sw / d_sw
        skew_v_0_sw = self._skew(uv_0_sw)
        As = skew_v_0_sw @ R03_ref
        Bs = - (skew_v_0_sw @ skew_v_0_sw) @ R03_ref
        Cs = np.outer(uv_0_sw, uv_0_sw) @ R03_ref
        R03_01 = As[0,1]*s_psi + Bs[0,1]*c_psi + Cs[0,1]
        R03_11 = As[1,1]*s_psi + Bs[1,1]*c_psi + Cs[1,1]
        R03_20 = As[2,0]*s_psi + Bs[2,0]*c_psi + Cs[2,0]
        R03_21 = As[2,1]*s_psi + Bs[2,1]*c_psi + Cs[2,1]
        R03_22 = As[2,2]*s_psi + Bs[2,2]*c_psi + Cs[2,2]
        q1 = np.arctan2(config[0] * -R03_11, config[0] * -R03_01)
        q2 = config[0] * np.arccos(np.clip(-R03_21, -1.0, 1.0))
        if self._kine_params.is_left:
            q3 = np.arctan2(config[0] * -R03_20, config[0] * -R03_22)
        else:
            q3 = np.arctan2(config[0] * R03_20, config[0] * R03_22)

        # Calc wrist q5 q6 q7
        """R47 
        INDUSTRIAL
        [[ *    *   c5s6]
        [  *    *   s5s6]
        [-s6c7 s6s7  c6]]

        ANTHROPOMORPHIC 
        left                       right
        [[ *    *   -c5c6]         [[ *    *   c5c6]
        [  *    *   -s5c6]         [  *    *   c5c6]
        [c6c7 -c6s7   s6]]         [-c6c7 c6s7 -s6]]
        """
        # TODO: 判断wrist关节奇异
        Aw = R_34.T @ As.T @ R_d
        Bw = R_34.T @ Bs.T @ R_d
        Cw = R_34.T @ Cs.T @ R_d
        R47_02 = Aw[0,2]*s_psi + Bw[0,2]*c_psi + Cw[0,2]
        R47_12 = Aw[1,2]*s_psi + Bw[1,2]*c_psi + Cw[1,2]
        R47_20 = Aw[2,0]*s_psi + Bw[2,0]*c_psi + Cw[2,0]
        R47_21 = Aw[2,1]*s_psi + Bw[2,1]*c_psi + Cw[2,1]
        R47_22 = Aw[2,2]*s_psi + Bw[2,2]*c_psi + Cw[2,2]
        if self._kine_params.arm_type == W1ArmType.INDUSTRIAL:
            q5 = np.arctan2(config[2] * R47_12, config[2] * R47_02)
            q6 = config[2] * np.arccos(np.clip(R47_22, -1.0, 1.0))
            q7 = np.arctan2(config[2] * R47_21, config[2] * -R47_20)
        else:
            if self._kine_params.is_left:
                q5 = np.arctan2(-R47_12, -R47_02)
                q6 = np.arcsin(np.clip(R47_22, -1.0, 1.0))
                q7 = np.arctan2(-R47_21, R47_20)
            else:
                q5 = np.arctan2(R47_12, R47_02)
                q6 = np.arcsin(np.clip(-R47_22, -1.0, 1.0))
                q7 = np.arctan2(R47_21, -R47_20)

        qpos = np.array([q1, q2, q3, q4, q5, q6, q7])
        if not self._check_qpos_limits(qpos):
            if verbose:
                self.logger.error(f"qpos out of joint limits.")
            return False, None

        return True, qpos


    def get_nearst_ik(self, 
                      xpos: np.ndarray, 
                      qpos_seed: np.ndarray) -> tuple[bool, np.ndarray]:
        # NOTE: dirty implementation!
        qpos_seed_ = np.array(qpos_seed)
        if qpos_seed_.shape[0] != self.dofs:
            self.logger.error(f"dimension != {self.dofs}")
            return False, None

        # TODO: 越过某个config的解如何处理？
        ref_config = np.sign(qpos_seed_[[1,3,5]])
        res, ref_psi = self._calc_arm_angle(qpos_seed_)
        if not res:
            return False, None

        sample_range = np.deg2rad(5.)
        sample_step = np.deg2rad(1.)
        psi_samples = ref_psi + np.arange(-sample_range, sample_range + sample_step, sample_step)

        qpos_list = []
        for psi in psi_samples:
            res, qpos = self.get_ik(xpos, psi, ref_config, verbose = False)
            if res:
                qpos_list.append(qpos)
        if not len(qpos_list):
            self.logger.error(f"Calc nearst ik failed\n xpos: {xpos}\n qpos_seed: {qpos_seed_}")
            return False, None

        dist = [np.sqrt(np.sum(self._joint_weights * (qpos - qpos_seed_) ** 2))
                 for qpos in qpos_list]
        nearst_idx = dist.index(min(dist))
        return True, qpos_list[nearst_idx]


    def set_tcp(self, xpos: np.ndarray) -> bool:
        # TODO: check SE3 and project to nearst SE3
        self._tcp = xpos
        return True

    def get_tcp(self) -> np.ndarray:
        return self._tcp

    def set_joint_limits(self, 
                         lower_bound: np.ndarray, 
                         upper_bound: np.ndarray) -> bool:
        lb = np.array(lower_bound)
        ub = np.array(upper_bound)
        if lb.shape[0] != self.dofs or ub.shape[0] != self.dofs:
            self.logger.error(f"dimension != {self.dofs}")
            return False

        self._joint_lb = lb
        self._joint_ub = ub
        self._update_qpos_limits()

    def get_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return self._joint_lb, self._joint_ub

    def set_user_joint_limits(self, 
                              lower_bound: np.ndarray, 
                              upper_bound: np.ndarray) -> bool:
        lb = np.array(lower_bound)
        ub = np.array(upper_bound)
        if lb.shape[0] != self.dofs or ub.shape[0] != self.dofs:
            self.logger.error(f"dimension != {self.dofs}")
            return False
        
        self._user_joint_lb = lb
        self._user_joint_ub = ub
        self._update_qpos_limits()

    def get_user_joint_limits(self):
        return self._user_joint_lb, self._user_joint_ub

    def set_ik_nearst_weight(self, weights: np.ndarray) -> bool:
        w = np.array(weights)
        if w.shape[0] != self.dofs:
            self.logger.error(f"dimension != {self.dofs}")
            return False
        
        self._joint_weights = w
        return True

    def get_ik_nearst_weight(self) -> np.ndarray:
        return self._joint_weights