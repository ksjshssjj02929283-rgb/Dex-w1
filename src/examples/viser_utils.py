'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
from dexe_rp import W1Vis, pos_wxyz_to_homogeneous, Box, Sphere, Mesh, Material, rotation_to_wxyz
from loop_rate_limiters import RateLimiter


def _load_viser_environment(vis):
    scene_handle_list = []
    workobjects_handle_list = []

    # Add viser handlers here

    return scene_handle_list, workobjects_handle_list


def load_viser(home_qpos: np.ndarray, left_ee_home_qpos: np.ndarray,
               right_ee_home_qpos: np.ndarray):
    vis = W1Vis()
    vis.set_qpos("robot", home_qpos)
    vis.set_ee_pos("left_ee", left_ee_home_qpos)
    vis.set_ee_pos("right_ee", right_ee_home_qpos)

    scene_handle_list, workobjects_handle_list = _load_viser_environment(vis)

    return vis, scene_handle_list, workobjects_handle_list


def viser_execute_traj(vis,
                       trajectory,
                       armlee_trigger=None,
                       armree_trigger=None):
    rate = RateLimiter(frequency=100, warn=True)
    print(f"trajectory length: {len(trajectory)}")
    idx = 0
    lee_idx = 0
    ree_idx = 0
    while True:
        if idx < len(trajectory):
            full_qpos = trajectory[idx]
            vis.set_qpos("robot", full_qpos)
            if armlee_trigger is not None and lee_idx < len(
                    armlee_trigger) and idx == armlee_trigger[lee_idx][0]:
                vis.set_ee_pos("left_ee",
                               np.asarray(armlee_trigger[lee_idx][1]))
                lee_idx += 1
            if armree_trigger is not None and ree_idx < len(
                    armree_trigger) and idx == armree_trigger[ree_idx][0]:
                vis.set_ee_pos("right_ee",
                               np.asarray(armree_trigger[ree_idx][1]))
                ree_idx += 1
            idx += 1
        else:
            break
        rate.sleep()
