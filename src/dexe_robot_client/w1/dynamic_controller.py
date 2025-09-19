# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
import time, threading
import pathlib, json
from copy import deepcopy
import atexit
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from abc import ABC, abstractmethod

from ..common import time_consuming, get_logger, DATA_DIR, W1ArmType, W1Version
from ..end_effector_client import EndEffectorType, create_end_effector_client
from ..robot_client import RobotClientRos2 as RobotClient
from ..robot_client import MotionGroup, Status
from .kinemtaics.torso_kinematics import TorsoKinematics
from .kinemtaics.arm_kinematics import SRSKinematics
from .path_planner.bezier_blend_path import interpolate_cartesian_path
from .trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner
from .utils.w1_params import W1Params
from .utils.unit_conversion import user_pose_to_xpos, xpos_to_user_pose
from .path_planner.utils.robot_type import RobotType
from .path_planner.fcl_checker import FCLChecker
from .path_planner.planner_pinocchio import Planner_Pin

from dexe_robot_client.w1.path_planner.utils.mapping_index import *
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.w1.path_planner.fcl_checker import FCLChecker
from dexe_robot_client.w1.path_planner.single_planner import SinglePlanner
import pinocchio

from dexe_robot_client.trajectory_client import TrajectoryPublisher, TrajectorySubscriber

np.set_printoptions(precision=4, suppress=True)

class DynamicController():
    def __init__(self,
                 is_sim: bool,
                 arm_type: W1ArmType,
                 w1_version: W1Version = W1Version.V2,
                end_effector_type: EndEffectorType = None):
        self.logger = get_logger()
        self.TIME_STEP = 0.01 # 100Hz

        self.robot_client = None
        self._robot_client_sim = None
        self._robot_client_real = None

        self._executor = None
        self._node_thread = None

        self._is_sim = None
        self._arm_type = arm_type
        self._w1_version = w1_version
        self._end_effector_type = end_effector_type

        self.w1_params = W1Params(self._arm_type, 
                                  self._w1_version, 
                                  self._end_effector_type)
        self._clients = []
        
        try:
            self._init_clients()
        except Exception:
            self._destroy_clients()
            raise RuntimeError("Ros2 node create failed ")
        
        self.set_sim_mode(is_sim)

        self._stop_move_event = threading.Event()
        self._stop_move_event.clear()

        # 在主线程中正常退出时自动调用
        atexit.register(self._destroy_clients)
        

        
    def _init_clients(self):
        if not rclpy.ok():
            rclpy.init()
        
        self._robot_client_sim = RobotClient(is_sim = True)
        self._robot_client_real = RobotClient(is_sim = False)
        self.robot_client = self._robot_client_sim

        # trajectory subscriber client
        self.trajectory_subscriber = TrajectorySubscriber()

        self._clients = [self._robot_client_sim, self._robot_client_real]
        self._is_sim = True
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._robot_client_sim)
        self._executor.add_node(self._robot_client_real)
        self._executor.add_node(self.trajectory_subscriber)

        self._node_thread = threading.Thread(target=self._executor.spin, 
                                            daemon=True)
        self._node_thread.start()
        self.logger.info("Ros2 client initialized.")

    def _destroy_clients(self):
        try:
            if rclpy.ok():
                rclpy.shutdown() # 停止ros触发spin

            if self._node_thread is not None:
                self._node_thread.join()

            if self._executor is not None:
                self._executor.shutdown()

            for node in self._clients:
                node.destroy_node()

            self.logger.debug(f"Ros2 node destroyed.")
        except Exception as e:
            self.logger.error(f"Ros2 node destroy failed: {e}.")

    def set_sim_mode(self, enable_sim: bool) -> bool:
        if self._is_sim == enable_sim:
            if self._is_sim:
                self.logger.info("W1 manager already in sim mode.")
            else:
                self.logger.info("W1 manager already in real mode.")
            return True

        if enable_sim:
            target_name = "sim"
            target_robot_client = self._robot_client_sim
        else:
            target_name = "real"
            target_robot_client = self._robot_client_real

        # 检查target client
        # if target_robot_client is None:
        #     self.logger.error(f"Robot_client_{target_name} is not initialized.")
        #     return False
        if not target_robot_client.is_connected(timeout_sec=0.1):
            self.logger.error(f"{target_robot_client.node_name} is not connected!")
            return False
        if target_robot_client.status not in [Status.Init, Status.Idle]:
            self.logger.error(f"{target_robot_client.node_name} status: {target_robot_client.status}, can not switch mode!")
            return False


        # 检查current client
        if not self.robot_client.is_connected():
            self.logger.error(f"{self.robot_client.node_name} is not connected!")
            return False
        if self.robot_client.status not in [Status.Init, Status.Idle]:
            self.logger.error(f"{self.robot_client.node_name} status: {self.robot_client.status}, can not switch mode!")
            return False

        self.robot_client = target_robot_client
        self._is_sim = enable_sim
        self.logger.info(f"W1 manager set to {target_name} mode.")
        return True
    
    def execute_joint_traj(self, 
                           qpos_array: np.ndarray,
                           reach_timeout: float = 1.0) -> bool:
        """按10ms间隔执行关节轨迹

        Args:
            qpos_array (np.ndarray): 关节轨迹
            reach_timeout (float, optional): 发送完指令后的到位检测timeout. Defaults to 2.0.

        Returns:
            bool: 
        """
        # check enable
        if self.robot_client.status in (Status.Init, Status.Running, Status.Error):
            self.logger.error(f"Robot client not in idle. ({self.robot_client.status})")
            return False

        # reset stop move flag
        if self._stop_move_event.is_set():
            self._stop_move_event.clear()

        # check current qpos and start qpos
        res, cur_qpos = self.get_current_qpos()
        if not res: 
            return False
        if not np.allclose(cur_qpos, qpos_array[0], atol=np.deg2rad(1)):
            self.logger.error(f"cur_qpos ({cur_qpos}) is not equal to start_qpos ({qpos_array[0]}).")
            return False
        
        count = 0

        # streaming
        end_iter = qpos_array.shape[0] - 1
        self.robot_client.start_joint_streaming_request(MotionGroup.WholeBody)
        self.logger.info("Start streaming.")
        t0 = time.time()
        last_qpos = cur_qpos
        for qpos in qpos_array:
            if self._stop_move_event.is_set():
                self.robot_client.send_streaming_command(position = last_qpos.tolist(), 
                                                         streaming_finished = True)
                self.logger.info(f"Stop sending streaming command")
                stop_ee_thread = True
                self._stop_move_event.clear()
                return False

            if count == end_iter + 1:
                streaming_finished = True
            else:
                streaming_finished = False

            # self.logger.debug(f"planner_cnt: {self.planner_cnt}")
            if not self.robot_client.send_streaming_command(qpos.tolist(), 
                                                            streaming_finished):
                self.logger.critical("Streaming command send failed.")
                return False
            self.robot_client.wait_second(self.TIME_STEP)
            count += 1
            last_qpos = qpos
        self.robot_client.wait_request_finish()

        self.logger.info("Finish streaming. " +
                        f"Ideal duration should be {(count-1) * self.TIME_STEP}s, " +
                        f"and actual duration is {time.time() - t0}s.")
        
        stop_ee_thread = True

        # check reach within timeout
        targt_qpos = qpos_array[-1]
        t1 = time.time()
        while time.time() - t1 < reach_timeout:
            res, cur_qpos = self.get_current_qpos()
            if not res: 
                return False
            if np.allclose(cur_qpos, targt_qpos, atol=np.deg2rad(1)):
                return True
            time.sleep(5*self.TIME_STEP)

        self.logger.error(f"Current qpos does NOT reach the target qpos until {reach_timeout}s, " +
                          f"qpos error(deg): {np.rad2deg(cur_qpos - targt_qpos)}.")
        return False
    

    def get_current_qpos(self, name: str = "full_body") -> tuple[bool, np.ndarray]:
        """获取特定部件(支持ee)的当前关节位置

        Args:
            name (str, optional): ["full_body", "torso", "head", "left_arm", "right_arm", 
                                    "left_arm_ee", "right_arm_ee"]. Defaults to "full_body".

        Returns:
            tuple[bool, np.ndarray]
        """
        support_names = list(self.w1_params.names) + self.w1_params.arm_ee_names
        if name not in support_names:
            self.logger.error(f"{support_names} is supported, except {name}.")
            return False, None
        
        # if name in self.w1_params.arm_ee_names:
        #     if not self._has_arm_ee_client(name, False):
        #         return False, None
        #     return self._get_arm_ee_client(name).get_qpos()
        
        robot_info = self.robot_client.get_info()
        if robot_info.joint_position is None:
            self.logger.error(f"Get current robot info failed, status: {robot_info.status}.")
            return False, None
        
        cur_qpos = np.array(robot_info.joint_position)

        if name == "full_body":
            return True, cur_qpos
        return True, cur_qpos[self.w1_params.index_infos[name]["index"]]
    
    def start_moving(self):
        if self.robot_client.status in (Status.Init, Status.Running, Status.Error):
            self.logger.error(f"Robot client not in idle. ({self.robot_client.status})")
            return False
        self.robot_client.start_joint_streaming_request(MotionGroup.WholeBody)
        record_new_traj_time = None
        current_traj_finish = False
        # input("press enter to continue")
        while True:
            start_time = time.time()
            received_traj = self.trajectory_subscriber.get_trajectory_numpy()
            
            if received_traj is None:
                current_traj_finish = False
                continue

            if current_traj_finish:
                continue

            if self._stop_move_event.is_set():
                self._stop_move_event.clear()

            res, cur_qpos = self.get_current_qpos()
            if not res:
                continue

            

            # TODO: check first waypoint is close to current qpos
            # if not np.allclose(cur_qpos, received_traj[0], atol=np.deg2rad(1)):
            #     self.logger.error(f"cur_qpos ({cur_qpos}) is not equal to start_qpos ({received_traj[0]}).")
            #     continue


            count = 1
            end_iter = received_traj.shape[0] - 1
            # self.robot_client.start_joint_streaming_request(MotionGroup.WholeBody)
            self.logger.info("Start streaming.")
            t0 = time.time()
            last_qpos = cur_qpos
            self.logger.debug(f"preparing time: {time.time() - start_time}s")
            if record_new_traj_time is not None:
                self.logger.debug(f"wait new traj time: {time.time() - record_new_traj_time}s")
                record_new_traj_time = None
            for qpos in received_traj:
                # self.logger.debug("enter the received traj loop and publish point")
                is_new_traj = self.trajectory_subscriber.get_is_new_trajectory()
                if is_new_traj:
                    # self.logger.info("Received new trajectory, stop current trajectory execution.")
                    record_new_traj_time = time.time()
                    # self.robot_client.send_streaming_command(qpos.tolist(),
                    #                                             streaming_finished = False)
                    # # self.robot_client.send_streaming_command(position = last_qpos.tolist(),
                    # #                                          streaming_finished = True)
                    # self.robot_client.wait_second(self.TIME_STEP)
                    # self.robot_client.wait_request_finish()
                    break
                if self._stop_move_event.is_set():
                    self.robot_client.send_streaming_command(position = last_qpos.tolist(),
                                                             streaming_finished = True)
                    self.logger.info(f"Stop sending streaming command")
                    self.robot_client.wait_request_finish()
                    # self.robot_client.wait_second(self.TIME_STEP)
                    # self._stop_move_event.clear()
                    break

                if count == end_iter + 1:
                    streaming_finished = True
                else:
                    streaming_finished = False

                if not self.robot_client.send_streaming_command(qpos.tolist(),
                                                                streaming_finished):
                    self.logger.critical("Streaming command send failed.")
                    current_traj_finish = True
                    self.robot_client.start_joint_streaming_request(MotionGroup.WholeBody)
                    break
                if streaming_finished:
                    current_traj_finish = True
                    self.robot_client.start_joint_streaming_request(MotionGroup.WholeBody)
                    self.logger.info("Finish current trajectory streaming, wait for new trajectory.")
                self.robot_client.wait_second(self.TIME_STEP)
                count += 1
                last_qpos = qpos




if __name__ == "__main__":
    dynamic_controller = DynamicController(is_sim=True,
                                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                                           w1_version=W1Version.V2,
                                           end_effector_type=None)

    dynamic_controller.start_moving()
    
