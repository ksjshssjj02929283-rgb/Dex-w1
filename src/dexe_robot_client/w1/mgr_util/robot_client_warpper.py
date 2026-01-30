# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------
import time
import queue
import threading
import numpy as np
from queue import SimpleQueue
from typing import List, Optional, Dict

from clientsupervisor.clients.generic_client import GenericClient
from clientsupervisor.clients.reduced_robot_client_interface import ReducedRobotClientInterface
from clientsupervisor.clients.modular_reduced_robot_client import ModularReducedRobotClient as RobotClient  # pyright: ignore[reportMissingImports]
from clientsupervisor.move_sync_option import MoveSyncOption  # pyright: ignore[reportMissingImports]
import logging
from ...robot_client.data_structure import MotionType, MotionGroup

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class RobotClientWarpper(ReducedRobotClientInterface):
    def __init__(self, client: "RobotClient"):
        self._client = client

    def query_state(self) -> ReducedRobotClientInterface.ReducedRobotState:
        try:
            info = self._client.get_info()
            state = ReducedRobotClientInterface.ReducedRobotState(
                jps=info.joint_position,
                latest_sent_id=info.latest_sent_id,
                latest_finished_id=info.latest_finished_id,
            )
            return state
        except Exception as e:
            logger.error(f"Failed to query state: {e}")
            raise

    def move_joint_trajectory_async(self, motion_target: List[List[float]], interval: float = 0.01, endpoint_index: Optional[List[int]] = None) -> int:
        """
        Move the robot along a joint trajectory asynchronously.
            
        Args:
            motion_target: List[List[float]]: motion joint trajectory
            interval: float: each motion point interval time
            endpoint_index: Optional[List[int]]: segment endpoints index for synchronized tasks

        Returns:
            int: move id
        """
        flattened_motion_target = [angle for point in motion_target for angle in point]

        finished_id = self._client.move_traj_async(flattened_motion_target, motion_group=MotionGroup.WholeBody, interval=interval, endpoint_index=endpoint_index)

        return finished_id - len(endpoint_index if endpoint_index else []) + 1


    def get_info(self) -> Dict:
        return self._client.get_info()


class GripperClientWarpper(GenericClient):
    def __init__(self, client: "EndEffectorClient"):
        self._client = client

    def peek_status(self) -> Dict:
        action = self._client.get_pos()
        return {'action': action}

    def invoke_async(self, request_id: int, content: Dict) -> bool:
        action = content['action']
        print(f"GripperClientWarpper invoke_async action: {action}")
        self._client.set_pos(action)
        return True

    def get_pos(self) -> Optional[List[float]]:
        return self._client.get_pos()
