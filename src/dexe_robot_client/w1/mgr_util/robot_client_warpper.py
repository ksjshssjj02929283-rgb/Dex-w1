# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from typing import List, Optional, Dict
import time

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

    def move_joint_trajectory_async(self, motion_target: List[List[float]], interval: float = 0.01, endpoint_index: Optional[List[int]] = None):
        """
        异步接口 - 多点位轨迹运动
        Args:
        - motion_target: List[List[float]]: 多点位的关节角度轨迹
        - interval: float: 每个点之间的时间间隔（秒）
        - endpoint_index: Optional[List[int]]: 指定每段末尾的点位索引
        """
        self._client.start_joint_streaming_request()

        for i, target in enumerate(motion_target):
            self._client.send_streaming_command(position=target, streaming_finished=(i == len(motion_target) - 1))
            if endpoint_index and i in endpoint_index:
                self._client.get_info()["latest_finished_id"] = i

            time.sleep(interval)

    def get_info(self) -> Dict:
        return self._client.get_info()

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

    # def _on_initial_state_bg_thread(self, jps: List[float]) -> None:
    #     """
    #     Update the Viser URDF with the initial joint positions.

    #     Args:
    #         jps (List[float]): Initial joint positions.
    #     """
    #     pass

class GripperClientWarpper(GenericClient):
    def __init__(self, client: "EndEffectorClient"):
        self._client = client

    def peek_status(self) -> Dict:
        qpos = self._client.get_qpos()
        return {'qpos': qpos}

    def invoke_async(self, content: Dict):
        qpos = content['qpos']
        self._client.set_qpos(qpos)

    def get_pos(self) -> Optional[List[float]]:
        return self._client.get_pos()
