# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np

class TorsoKinmatics:
    def __init__(self):
        self.calf_length = 0.34
        self.thigh_length = 0.34
        self.torso_length = 0.385
        self.torso_width = 0.197
        '''MDH
        i: a(i-1)  alpha(i-1)  d(i) theta(i)
        1     0        0        0     q1
        2     L1       0        0     q2
        3     L2       0        0     q3+np.pi/2
        4     0     np.pi/2     0     q4
        '''
        self.mdh_params = [[0., 0., 0., 0.], 
                          [self.calf_length, 0., 0., 0.], 
                          [self.thigh_length, 0., 0., np.pi/2],
                          [0., np.pi/2, 0., 0.]] # a alpha d theta
        self.T_ob = np.array([[0., 1., 0., 0.],
                              [0., 0., 1., 0.],
                              [1., 0., 0., 0.],
                              [0., 0., 0., 1.]]) # origin 2 base
        self.T_el = np.array([[0., 1., 0., 0.],
                              [0., 0., 1., self.torso_width/2],
                              [1., 0., 0., self.torso_length],
                              [0., 0., 0., 1.]]) # end-effector 2 left arm base
        self.T_er = np.array([[0., 1., 0., 0.],
                              [0., 0., -1., -self.torso_width/2],
                              [-1., 0., 0., self.torso_length],
                              [0., 0., 0., 1.]]) # end-effector 2 right arm base
        
    def _mdh(self, a, alpha, d, theta):
        return np.array([[np.cos(theta), -np.sin(theta), 0., a],
                         [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
                         [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), np.cos(alpha)*d],
                         [0., 0., 0., 1.]])

    def get_fk(self, qpos: np.ndarray, is_left_base: bool):
        T = self.T_ob
        for i, q in enumerate(qpos):
            T = T @ self._mdh(self.mdh_params[i][0],
                              self.mdh_params[i][1],
                              self.mdh_params[i][2],
                              self.mdh_params[i][3]+q)
        if is_left_base:
            return T @ self.T_el
        else:
            return T @ self.T_er


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    torso_kine = TorsoKinmatics()
    print(torso_kine.get_fk(np.array([np.pi/4, -np.pi/2, np.pi/4, 0.]), True))