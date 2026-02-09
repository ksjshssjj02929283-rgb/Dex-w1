'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
from scipy.spatial.transform import Rotation as R


# [x,y,z,rx,ry,rz] (unit: mm, deg) <-> 4x4 homogeneous transformation matrix (unit: m, rad)
def user_pose_to_xpos(pos_mm: list,
                      euler_deg: list,
                      rotation_sequence: str = "xyz") -> np.ndarray:
    pos_m = np.array(pos_mm) / 1000.0
    rot = R.from_euler(rotation_sequence, euler_deg, degrees=True)
    xpos = np.eye(4)
    xpos[:3, :3] = rot.as_matrix()
    xpos[:3, 3] = pos_m
    return xpos


def xpos_to_user_pose(xpos: np.ndarray, rotation_sequence: str = "xyz") -> list:
    pos_mm = xpos[:3, 3] * 1000.0
    rot = R.from_matrix(xpos[:3, :3])
    euler_deg = rot.as_euler(rotation_sequence, degrees=True)
    return pos_mm.tolist() + euler_deg.tolist()
