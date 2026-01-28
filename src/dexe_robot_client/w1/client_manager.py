import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from dexe_robot_client.robot_client import RobotClientRos2 as RobotClient
from dexe_robot_client.navigation_client import ChassisClient
from dexe_robot_client.end_effector_client import create_end_effector_client_real_and_sim
from dexe_robot_client.common import get_logger

class ClientManager:
    def __init__(self, use_nav: bool = False, end_effector_type=None):
        self.logger = get_logger()
        self._use_nav = use_nav
        self._end_effector_type = end_effector_type

        self._robot_client_sim = None
        self._robot_client_real = None
        self._chassis_client_sim = None
        self._chassis_client_real = None

        self._left_ee_client_sim = None
        self._left_ee_client_real = None
        self._right_ee_client_sim = None
        self._right_ee_client_real = None

        self._clients = []
        self._executor = None
        self._node_thread = None

    def init_clients(self):
        if not rclpy.ok():
            rclpy.init()

        self._robot_client_sim = RobotClient(is_sim=True)
        self._robot_client_real = RobotClient(is_sim=False)
        self._chassis_client_real = ChassisClient(is_sim=False)

        if self._end_effector_type is not None:
            self._left_ee_client_sim, self._left_ee_client_real = create_end_effector_client_real_and_sim(
                is_left=True, end_effector_type=self._end_effector_type
            )
            self._right_ee_client_sim, self._right_ee_client_real = create_end_effector_client_real_and_sim(
                is_left=False, end_effector_type=self._end_effector_type
            )

        self._clients = [
            self._robot_client_sim,
            self._robot_client_real,
            self._chassis_client_real,
        ]

        if self._end_effector_type is not None:
            self._clients.extend([
                self._left_ee_client_sim,
                self._left_ee_client_real,
                self._right_ee_client_sim,
                self._right_ee_client_real,
            ])

        self._executor = MultiThreadedExecutor()
        for node in self._clients:
            self._executor.add_node(node)
        self._node_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._node_thread.start()
        self.logger.info("Clients initialized.")

    def destroy_clients(self):
        try:
            self.logger.info("Destroying clients...")
            if self._executor:
                self._executor.shutdown()
                self.logger.info("Executor shut down.")
            if self._node_thread and self._node_thread.is_alive():
                self._node_thread.join()
                self.logger.info("Node thread joined.")
            self.logger.info("All clients destroyed.")
        except Exception as e:
            self.logger.error(f"Error while destroying clients: {e}")

    def set_mode(self, is_sim: bool):
        if is_sim:
            self.logger.info("Switching to simulation mode.")
            return self._robot_client_sim
        else:
            self.logger.info("Switching to real mode.")
            return self._robot_client_real

    def check_client_status(self, client):
        if client and client.is_connected():
            return True
        self.logger.error(f"Client {client.node_name} is not connected.")
        return False
