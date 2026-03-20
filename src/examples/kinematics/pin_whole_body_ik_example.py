'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
from dexe_robot_client.w1.kinematics.pin_ik_whole_body import PinIKWholeBody
from dexe_robot_client.w1.utils.unit_conversion import *
from dexe_rp import W1Vis, pos_wxyz_to_homogeneous, Box, Material
from loop_rate_limiters import RateLimiter

if __name__ == "__main__":

    urdf_path = "/home/young/dexe_robot_client_data/assets/Model/W1_V2_A/W1_V2_A.urdf"
    ik_solver = PinIKWholeBody(urdf_path,
                               track_type="standard",
                               cost_type="standard")

    vis = W1Vis()
    h_torso_text = vis.server.gui.add_text(label="Torso (rad)",
                                           initial_value="[]")
    h_head_text = vis.server.gui.add_text(label="Head (rad)",
                                          initial_value="[]")
    h_left_arm_text = vis.server.gui.add_text(label="Left arm (rad)",
                                              initial_value="[]")
    h_right_arm_text = vis.server.gui.add_text(label="Right arm (rad)",
                                               initial_value="[]")

    init_qpos = np.array([
        0.78539816, -1.57079633, 0.78539816, 0.03490659, 0., -0.76794487, -0.98,
        -1.451, 1.622, -0.479, -1.453, -0.513, 0.33, 0.969, 1.416, -1.594,
        0.559, 1.263, 0.53, -0.436
    ])

    vis.set_qpos("robot", init_qpos)

    from IPython import embed
    # embed()

    cur_qpos = vis.get_qpos()
    cur_arml_xpos = vis.w1_kine.get_fk("left_arm", cur_qpos)[1]
    cur_armr_xpos = vis.w1_kine.get_fk("right_arm", cur_qpos)[1]
    center = (cur_arml_xpos[:3, 3] + cur_armr_xpos[:3, 3]) / 2

    box = Box(0.1, 0.1, 0.1, material=Material("purple", (0.5, 0.0, 0.5, 1.0)))
    box.pose = np.eye(4)
    box.pose[:3, 3] = center
    box.pose[0, 3] += 0.1
    h_box = vis.add_body("/box", box)

    h_target = vis.server.scene.add_transform_controls(name="/target",
                                                       scale=0.2,
                                                       position=h_box.position,
                                                       wxyz=h_box.wxyz)

    cur_qpos = vis.get_qpos()
    T_chassis_larmbase = vis.w1_kine.get_fk("left_arm_base", cur_qpos)[1]
    T_chassis_rarmbase = vis.w1_kine.get_fk("right_arm_base", cur_qpos)[1]
    T_chassis_larm = vis.w1_kine.get_fk("left_arm", cur_qpos)[1]
    T_chassis_rarm = vis.w1_kine.get_fk("right_arm", cur_qpos)[1]

    T_world_target = pos_wxyz_to_homogeneous(h_box.position, h_box.wxyz)
    T_target_larm = np.linalg.inv(T_world_target) @ T_chassis_larm
    T_target_rarm = np.linalg.inv(T_world_target) @ T_chassis_rarm

    armlee_T_armltcp = np.identity(4)
    armree_T_armrtcp = np.identity(4)
    armree_T_armrtcp[:3, :3] = R.from_rotvec([0.0, 0.0, 180.],
                                             degrees=True).as_matrix()

    input("Press Enter to start IK...")
    rate = RateLimiter(frequency=100, warn=True)
    while True:
        h_box.position = h_target.position
        h_box.wxyz = h_target.wxyz

        T_world_target = pos_wxyz_to_homogeneous(h_box.position, h_box.wxyz)

        cur_qpos = vis.get_qpos()
        h_torso_text.value = f"{np.round(cur_qpos[:4].tolist(), 2)}"
        h_head_text.value = f"{np.round(cur_qpos[4:6].tolist(), 2)}"
        h_left_arm_text.value = f"{np.round(cur_qpos[6:13].tolist(), 2)}"
        h_right_arm_text.value = f"{np.round(cur_qpos[13:].tolist(), 2)}"

        T_world_larm = T_world_target @ T_target_larm
        T_world_rarm = T_world_target @ T_target_rarm

        world_T_armltcp = T_world_larm @ armlee_T_armltcp
        world_T_armrtcp = T_world_rarm @ armree_T_armrtcp
        arml_tcp_user_pose = xpos_to_user_pose(world_T_armltcp)
        armr_tcp_user_pose = xpos_to_user_pose(world_T_armrtcp)

        res, result_qpos = ik_solver.solve_ik(left_ee_target=T_world_larm,
                                              right_ee_target=T_world_rarm,
                                              cur_qpos=cur_qpos)
        if res:
            vis.set_qpos("robot", result_qpos)
            # print("qpos: ", np.round(result_qpos, 3).tolist())
            # print("left arm tcp pose:", arml_tcp_user_pose)
            # print("right arm tcp pose:", armr_tcp_user_pose)
            # print("-----")

            rate.sleep()
        else:
            continue
