# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------
import time
import copy
import numpy as np
from typing import List, Optional, Dict

import yourdfpy
import concurrent.futures

import viser
from viser.extras import ViserUrdf

from clientsupervisor.move_sync_option import MoveSyncOption
from clientsupervisor.client_supervisior import RobotClientSupervisor
from clientsupervisor.clients.generic_client import GenericClient
from clientsupervisor.clients.modular_reduced_robot_client import (
    SubModuleTask,
    ModularReducedRobotClient,
)

import logging

logger = logging.getLogger(__name__)

all = ["ViserModularReducedRobotClient", "ViserGripperClient"]

class ViserModularReducedRobotClient(ModularReducedRobotClient):
    """
    ViserRobotClient is a placeholder for the actual robot client implementation.
    This class should implement methods to communicate with the Viser robot.
    """

    def __init__(
        self,
        init_full_q: List[float],
        viser_handle_dict: Dict[str, ViserUrdf],
        main_task: SubModuleTask,
        submodule_tasks: List[SubModuleTask],
    ):
        super().__init__(
            init_full_q=init_full_q,
            main_task=main_task,
            submodule_tasks=submodule_tasks,
        )
        # Store the Viser handle dictionary
        self._viser_urdf_dict = viser_handle_dict

    def _on_robot_state_bg_thread(self, jps: List[float],
                                  jps_move_id: int) -> None:
        """
        Update the Viser URDF with the current joint positions.

        Args:
            jps (List[float]): Current joint positions.
            jps_move_id (int): Move ID associated with the joint positions.
        """
        self._viser_urdf_dict["main"].update_cfg(np.array(jps[:6]))
        self._viser_urdf_dict["sub"].update_cfg(np.array(jps[6:]))

    def _on_initial_state_bg_thread(self, jps: List[float]) -> None:
        """
        Update the Viser URDF with the initial joint positions.

        Args:
            jps (List[float]): Initial joint positions.
        """
        self._viser_urdf_dict["main"].update_cfg(np.array(jps[:6]))
        self._viser_urdf_dict["sub"].update_cfg(np.array(jps[6:]))


class ViserGripperClient(GenericClient):
    """
    GripperClient is a placeholder for the actual gripper client implementation.
    This class should implement methods to communicate with the gripper.
    """

    def __init__(self, robot_client: ModularReducedRobotClient):
        self._robot_client = robot_client

    def invoke_async(self,
                     request_id: int,
                     content: Optional[Dict] = None) -> bool:
        if content and "action" in content:
            open_steps = 50
            close_traj = [[(1 - t) * 0.0 + t * 1.0]
                          for t in np.linspace(0, 0.03, open_steps)]
            open_traj = copy.deepcopy(close_traj)
            open_traj = open_traj[::-1]
            action = content["action"]
            if action == "open":
                # Implement gripper open logic
                self._robot_client.move_submodule_async("gripper",
                                                        open_traj,
                                                        interval=0.01)
            elif action == "close":
                # Implement gripper close logic
                self._robot_client.move_submodule_async("gripper",
                                                        close_traj,
                                                        interval=0.01)
            else:
                return False
            return True



if __name__ == "__main__":
    import yourdfpy
    viser_server = viser.ViserServer()
    arm_urdf = yourdfpy.URDF.load(
        "/home/chase/rlia_data/extract/RobotModel/UniversalRobots/UR10/UR10.urdf"
    )

    gripper_urdf = yourdfpy.URDF.load(
        "/home/chase/dexe_robot_client_data/assets/Model/left_pgc/left_hand.urdf"
    )

    init_qpos = np.array([
        np.pi / 2, -np.pi / 4 * 3, np.pi / 4 * 3, -np.pi / 2, -np.pi / 2,
        np.pi / 2
    ])
    target_qpos = np.array(
        [np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2])
    arm_viser_handle = ViserUrdf(target=viser_server, urdf_or_path=arm_urdf)
    arm_viser_handle.update_cfg(init_qpos)

    ee_link_node = arm_viser_handle._joint_frames[-1].name
    gripper_viser_handle = ViserUrdf(target=viser_server,
                                     urdf_or_path=gripper_urdf,
                                     root_node_name=ee_link_node)
    gripper_viser_handle.update_cfg(np.zeros(2))

    viser_handle_dict = {"main": arm_viser_handle, "sub": gripper_viser_handle}

    init_full_q = init_qpos.tolist() + [0.0]
    main_arm_module = SubModuleTask()
    main_arm_module.name = "arm"
    main_arm_module.dof_begin = 0
    main_arm_module.n_dof = 6
    sub_gripper_module = SubModuleTask()
    sub_gripper_module.name = "gripper"
    sub_gripper_module.dof_begin = 6
    sub_gripper_module.n_dof = 1

    client = ViserModularReducedRobotClient(
        init_full_q=init_full_q,
        viser_handle_dict=viser_handle_dict,
        main_task=main_arm_module,
        submodule_tasks=[sub_gripper_module],
    )

    gripper_client = ViserGripperClient(robot_client=client)

    executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)

    supervisior = RobotClientSupervisor(
        robot_client=client,
        client_map={"gripper": gripper_client},
        executor_owned=executor,
    )
    supervisior.initialize()

    # Generate trajectory from init_qpos to target_qpos
    steps = 200
    pick_traj = [((1 - t) * init_qpos + t * target_qpos).tolist()
                 for t in np.linspace(0, 1, steps)]
    pick_back_traj = pick_traj[::-1]

    place_traj = [q.copy() for q in pick_traj]
    for q in place_traj:
        q[0] += np.deg2rad(30)

    # Genera init_qpos to place_traj[0] trajectory
    init_to_place_traj = [
        ((1 - t) * init_qpos + t * np.array(place_traj[0])).tolist()
        for t in np.linspace(0, 1, steps)
    ]
    place_back_traj = [q.copy() for q in place_traj[::-1]]
    time.sleep(1)

    move_id, end_id = supervisior.move_joint_trajectory_async(pick_traj,
                                                              interval=0.005)
    res_id = supervisior.call_srv_async("gripper", {"action": "close"},
                                        MoveSyncOption.sync_w_latest_move())
    move_id, end_id = supervisior.move_joint_trajectory_async(pick_back_traj,
                                                              interval=0.0025)
    move_id, end_id = supervisior.move_joint_trajectory_async(
        init_to_place_traj, interval=0.0025)
    move_id, end_id = supervisior.move_joint_trajectory_async(place_traj,
                                                              interval=0.0025)
    res_id = supervisior.call_srv_async("gripper", {"action": "open"},
                                        MoveSyncOption.sync_w_latest_move())
    supervisior.wait_move()

    # Wait for execution to complete
    time.sleep(0.5)

    # Query final state
    state = client.query_state()
    print(f"Final robot state: {state}")

    # Stop the client
    supervisior.stop()
