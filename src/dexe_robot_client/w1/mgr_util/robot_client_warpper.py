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
        """
        Initialize the RobotClientWrapper with a RobotClient instance.
        """
        super().__init__()
        self._client = client
        self._latest_kill_id = -1

    def query_state(self) -> ReducedRobotClientInterface.ReducedRobotState:
        """
        Query the current state of the robot.

        Returns:
            ReducedRobotClientInterface.ReducedRobotState: current robot state
        """
        info = self._client.get_info()
        state = ReducedRobotClientInterface.ReducedRobotState(
            jps=info.joint_position,
            latest_sent_id=info.latest_sent_id,
            latest_finished_id=info.latest_finished_id,
            latest_kill_id=self._latest_kill_id)
        return state

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
        flattened_motion_target = [
            angle for point in motion_target for angle in point
        ]

        finished_id = self._client.move_traj_async(
            flattened_motion_target,
            motion_group=MotionGroup.WholeBody,
            endpoint_index=endpoint_index,
            interval=interval)
        endpoint_index = endpoint_index if endpoint_index else []

        return finished_id - len(endpoint_index) + 1

    def wait_move(self, time_out: float = -1, interval: float = 0.01) -> bool:
        """
        Wait for the robot to finish the movement.

        Args:
            time_out: float: maximum wait time in seconds, -1 means wait indefinitely
            interval: float: polling interval time in seconds

        Returns:
            bool: True if the movement is finished within the timeout, False otherwise
        """
        return self._client.wait_request_finish(request_id=None,
                                                timeout_sec=time_out)

    def get_info(self) -> Dict:
        return self._client.get_info()

    def stop_move(self):
        new_kill_id = self._client.stop_move()
        new_kill_id = max(self._latest_kill_id, new_kill_id)
        self._latest_kill_id = new_kill_id


class GripperClientWrapper(GenericClient):

    def __init__(self, client: "EndEffectorClient"):
        """
        Initialize the GripperClientWrapper with an EndEffectorClient instance.
        """
        self._client = client

    def peek_status(self) -> Dict:
        """
        Peek the current status of the gripper.

        Returns:
            Dict: current status of the gripper
        """
        res, action = self._client.get_pos()
        return {"res": res, "action": action}

    def invoke_async(self, request_id: int, content: Dict) -> bool:
        """
        Invoke an asynchronous command to the gripper.

        Args:
            request_id: int: unique identifier for the request
            content: Dict: command content, must include 'action' key

        Returns:
            bool: True if the command is successfully sent, False otherwise
        """
        action = content['action']
        self._client.set_pos(action)
        return True
