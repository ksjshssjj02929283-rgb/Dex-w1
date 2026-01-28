# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from typing import List, Optional, Dict

from clientsupervisor.clients.generic_client import GenericClient
from clientsupervisor.clients.reduced_robot_client_interface import ReducedRobotClientInterface
from clientsupervisor.move_sync_option import MoveSyncOption  # pyright: ignore[reportMissingImports]

class RobotClientWarpper(ReducedRobotClientInterface):
    def __init__(self, client: "RobotClient"):
        self._client = client

    def query_state(self) -> ReducedRobotClientInterface.ReducedRobotState:
        info = self._client.get_info()
        state = ReducedRobotClientInterface.ReducedRobotState(jps=info.joint_position, latest_sent_id=info.latest_sent_id, latest_finished_id=info.latest_finished_id)
        return state

    def move_joint_trajectory_async(self):
        self._client.move_joint_trajectory_async()

class GripperClientWarpper(GenericClient):
    def __init__(self, client: "EndEffectorClient"):
        self._client = client

    def peek_status(self) -> Dict:
        qpos = self._client.get_qpos()
        return {'qpos': qpos}

    def invoke_async(self, content: Dict):
        qpos = content['qpos']
        self._client.set_qpos(qpos)

