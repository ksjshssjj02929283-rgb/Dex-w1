# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import time
import queue
import threading
import concurrent.futures
from collections import deque
from dataclasses import dataclass
from typing import Dict, Optional, Tuple, List

from .clients.generic_client import GenericClient
from .clients.reduced_robot_client_interface import ReducedRobotClientInterface
from .clients.modular_reduced_robot_client import ModularReducedRobotClient
from .utils.move_sync_option import MoveSyncOption

from ..common import get_logger


class RobotClientSupervisor(object):
    """
    Supervisor class for managing robot client and peripheral device clients.
    Handles background task scheduling, synchronization, and coordination.
    """

    @dataclass
    class BackgroundRequest(object):
        """
        Data class for internal background task requests.

        Attributes:
            srv_name (str): Service/client name.
            associated_move_id (int): Associated move ID for synchronization.
            request_id (int): Unique request ID.
            content (Optional[Dict]): Optional request content.
        """

        srv_name: str
        associated_move_id: int = -1
        request_id: int = -1
        content: Optional[Dict] = None

    def __init__(
        self,
        robot_client: ModularReducedRobotClient,
        client_map: Optional[Dict[str, GenericClient]] = None,
        executor_owned: Optional[concurrent.futures.ThreadPoolExecutor] = None,
    ):
        """
        Initialize the RobotClientSupervisor.

        Args:
            robot_client (ModularReducedRobotClient): The main robot client.
            client_map (Dict[str, GenericClient]): Peripheral device client map.
            executor_owned (Optional[ThreadPoolExecutor]): Shared thread pool executor.
        """
        self.logger = get_logger()
        self._robot_client = robot_client  # Robot main client
        self._client_map = (client_map if client_map is not None else {}
                           )  # Peripheral device client map
        self._task_queue = queue.SimpleQueue(
        )  # Task queue for background requests
        self._next_request_id: int = 0  # Next request ID
        self._bg_thread: Optional[threading.Thread] = None  # Background thread
        self._executor = executor_owned  # Shared thread pool
        self._stop = False  # Stop flag

    @property
    def client_map(self):
        """
        Get the peripheral device client map.
        Returns:
            Dict[str, GenericClient]: The client map.
        """
        return self._client_map

    @property
    def robot_client(self):
        """
        Get the main robot client.
        Returns:
            ModularReducedRobotClient: The robot client instance.
        """
        return self._robot_client

    def initialize(self):
        """
        Initialize the supervisor, robot client, and all peripheral clients. 
        Starts the background thread.
        """
        if self._bg_thread is not None and self._bg_thread.is_alive():
            self.logger.warning("Supervisor is already initialized.")
            return

        # Init robot client
        self._robot_client.initialize()

        # Init other clients
        for _, v in self._client_map.items():
            v.initialize(executor=self._executor)

        # Start background thread for supervision
        self._bg_thread = threading.Thread(target=self._bg_thread_loop)
        self._bg_thread.start()

    def stop(self):
        """
        Stop the supervisor, background thread, all clients, and the robot client.
        """
        if self._bg_thread is None or not self._bg_thread.is_alive():
            self.logger.warning("Supervisor is not running or already stopped.")
            return

        self._stop = True
        self._bg_thread.join()

        # Stop all the clients
        for _, v in self._client_map.items():
            v.stop()
        self._executor.shutdown(cancel_futures=True)

        # Stop the robot client
        self._robot_client.stop()

    def call_srv_async(
        self,
        srv_name: str,
        content: Optional[Dict] = None,
        sync_option: MoveSyncOption = MoveSyncOption.sync_w_latest_move(),
    ) -> int:
        """
        Asynchronously call a peripheral service/client, optionally synchronized with a robot move.

        Args:
            srv_name (str): Name of the service/client.
            content (Optional[Dict]): Optional request content.
            sync_option (MoveSyncOption): Synchronization option.

        Returns:
            int: The request ID assigned to this call.
        """
        # Make request id
        request_id = self._next_request_id
        self._next_request_id += 1

        # Make move id
        associated_move_id: int = -1
        if sync_option.need_sync:
            if sync_option.associated_move_id >= 0:
                associated_move_id = sync_option.associated_move_id
            else:
                # Query from latest move id
                local_robot_state: Optional[
                    ReducedRobotClientInterface.ReducedRobotState] = None
                while local_robot_state is None:
                    local_robot_state = self._robot_client.query_state()
                    # Do NOT need to sleep here, as this is likely due to lock contention

                # Get the id
                assert local_robot_state is not None
                associated_move_id = local_robot_state.latest_sent_id

        # Make task
        task = RobotClientSupervisor.BackgroundRequest(
            srv_name=srv_name,
            associated_move_id=associated_move_id,
            request_id=request_id,
            content=content,
        )
        self._task_queue.put(task)

        # Done
        return request_id

    def move_joint_trajectory_async(
        self,
        motion_target: List[List[float]],
        interval: float = 0.01,
        endpoint_index: Optional[List[int]] = None,
    ) -> Tuple[int, int]:
        """
        Execute a robot joint trajectory movement asynchronously.

        Args:
            motion_target (List[List[float]]): List of joint positions for the trajectory.
            interval (float): Time interval between points.
            endpoint_index (Optional[List[int]]): Segment endpoints.

        Returns:
            Tuple[int, int]: The range of movement IDs [begin_id, end_id). 
                             end_id is NOT included.
        """
        n_segments = 1 if endpoint_index is None else len(endpoint_index)
        move_id_begin = self._robot_client.move_joint_trajectory_async(
            motion_target=motion_target,
            interval=interval,
            endpoint_index=endpoint_index,
        )
        if move_id_begin < 0:
            return move_id_begin, move_id_begin

        # Correct
        return move_id_begin, move_id_begin + n_segments

    def query_robot_state(self):
        """
        Query the current robot state.

        Returns:
            ReducedRobotClientInterface.ReducedRobotState: The current robot state.
        """
        return self._robot_client.query_state()

    def wait_move(self, time_out: float = -1, interval: float = 0.05) -> bool:
        """
        Wait for all robot movements to finish or until timeout.

        Args:
            time_out (float): Maximum time to wait in seconds. 
                              If negative, wait indefinitely. Default is -1.
            interval (float): Polling interval in seconds. Default is 0.05s.

        Returns:
            bool: True if movements finished before timeout, False otherwise.
        """
        return self._robot_client.wait_move(time_out=time_out,
                                            interval=interval)

    def _bg_thread_loop(self,
                        sleep_interval: float = 0.05,
                        idle_interval: float = 0.1):
        """
        Background thread loop for task scheduling and synchronization.
        Handles both synchronized and non-synchronized tasks, and executes them when ready.

        Args:
            sleep_interval (float): Sleep interval between checks when active.
            idle_interval (float): Sleep interval when idle.
        """
        local_tasks_no_sync = deque()
        local_tasks_sync = deque()
        while not self._stop:
            # Pop task to local stack
            while not self._stop:
                try:
                    elem: RobotClientSupervisor.BackgroundRequest = (
                        self._task_queue.get_nowait())
                    if elem.associated_move_id >= 0:
                        local_tasks_sync.append(elem)
                    else:
                        local_tasks_no_sync.append(elem)
                except queue.Empty:
                    break

            if self._stop:
                break

            # Run all no sync tasks immediately
            while local_tasks_no_sync:
                task: RobotClientSupervisor.BackgroundRequest = (
                    local_tasks_no_sync.popleft())
                assert task.associated_move_id < 0
                if task.srv_name in self._client_map:
                    client: GenericClient = self._client_map[task.srv_name]
                    client.invoke_async(task.request_id, task.content)

            if not local_tasks_sync:
                # No sync tasks, sleep for a while
                time.sleep(idle_interval)
                continue

            # Query robot status and check for synced tasks
            robot_state: Optional[
                ReducedRobotClientInterface.
                ReducedRobotState] = self._robot_client.query_state()
            if robot_state is None:
                # To next loop
                time.sleep(idle_interval)
                continue

            # Execute the tasks that depend on robot state
            remaining_tasks = deque()
            for task in local_tasks_sync:
                # Directly drop this one without invoking or put into buffer
                assert task.associated_move_id >= 0
                if task.associated_move_id <= robot_state.latest_kill_id:
                    continue

                # Not killed
                if task.associated_move_id > robot_state.latest_finished_id:
                    remaining_tasks.append(task)
                    continue

                # Else execute this
                if task.srv_name in self._client_map:
                    client: GenericClient = self._client_map[task.srv_name]
                    # TODO: Handle exceptions here
                    client.invoke_async(task.request_id, task.content)

            # Update the tasks
            local_tasks_sync = remaining_tasks
            # To next loop
            time.sleep(sleep_interval)
