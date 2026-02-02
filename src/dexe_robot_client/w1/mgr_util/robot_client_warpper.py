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

from ...client_supervisor.clients.generic_client import GenericClient
from ...client_supervisor.clients.reduced_robot_client_interface import ReducedRobotClientInterface
from ...client_supervisor.utils.move_sync_option import MoveSyncOption
from ...robot_client.data_structure import MotionType, MotionGroup


class RobotClientWrapper(ReducedRobotClientInterface):

    def __init__(self, client: "RobotClient"):
        self._client = client
        self.finished_id = -1
        self.traj_num = -1
        self.endpoint_index = []

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

    def move_joint_trajectory_async(
            self,
            motion_target: List[List[float]],
            interval: float = 0.01,
            endpoint_index: Optional[List[int]] = None) -> int:
        """
        Move the robot along a joint trajectory asynchronously.
            
        Args:
            motion_target: List[List[float]]: motion joint trajectory
            interval: float: each motion point interval time
            endpoint_index: Optional[List[int]]: segment endpoints index for synchronized tasks

        Returns:
            int: movement beginning ID
        """
        self.traj_num = len(motion_target)
        flattened_motion_target = [
            angle for point in motion_target for angle in point
        ]

        finished_id = self._client.move_traj_async(
            flattened_motion_target,
            motion_group=MotionGroup.WholeBody,
            interval=interval,
            endpoint_index=endpoint_index)
        self.finished_id = finished_id
        self.endpoint_index = endpoint_index if endpoint_index else []

        return finished_id - len(endpoint_index if endpoint_index else []) + 1

    def wait_move(self, time_out: float = -1, interval: float = 0.01) -> bool:
        traj_duration = (self.traj_num - 1) * interval
        last_finished_id = self.finished_id - len(self.endpoint_index)
        for next_finished_id in range(last_finished_id + 1,
                                      self.finished_id + 1):
            self._client.wait_request_finish(request_id=next_finished_id,
                                             timeout_sec=max(5., traj_duration))

        return True

    def get_info(self) -> Dict:
        return self._client.get_info()


class GripperClientWrapper(GenericClient):

    def __init__(self, client: "EndEffectorClient"):
        self._client = client

    def peek_status(self) -> Dict:
        action = self._client.get_pos()
        return {'action': action}

    def invoke_async(self, request_id: int, content: Dict) -> bool:
        action = content['action']
        self._client.set_pos(action)
        return True

    def get_pos(self) -> Optional[List[float]]:
        return self._client.get_pos()
