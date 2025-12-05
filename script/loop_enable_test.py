import json
import threading
import time
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SERVICE_NAME = 'dexe-system.service'
SUDO_PASSWORD = '123456'
MAX_RESTARTS = 30
TOPIC_NAME = '/feedback/robot_server_state'
DO_INITIAL_RESTART = True


def restart_service_with_sudo(password: str, service_name: str) -> bool:
    cmd = ['sudo', '-S', 'systemctl', 'restart', service_name]
    try:
        proc = subprocess.run(
            cmd,
            input=(password + '\n').encode('utf-8'),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=60,
            check=False,
        )
        if proc.returncode != 0:
            err = proc.stderr.decode(errors='ignore').strip()
            print(f'[ERROR] systemctl failed (rc={proc.returncode}): {err}')
            return False
        return True
    except Exception as e:
        print(f'[EXCEPTION] restart_service_with_sudo: {e}')
        return False


class RobotStateListener(Node):
    def __init__(self):
        super().__init__('robot_state_listener')
        self.subscription = self.create_subscription(
            String, TOPIC_NAME, self._on_msg, 10
        )
        self.restart_count = 0
        self.triggering = False

        self.enabled_event = None  # type: threading.Event | None

        self.get_logger().info(
            f'Listening on {TOPIC_NAME}; will restart {SERVICE_NAME} '
            f'after 5s when robot_statusword == "ENABLED" (max {MAX_RESTARTS} times).'
        )

        if DO_INITIAL_RESTART:
            threading.Thread(target=self._initial_restart, daemon=True).start()

    def _start_enabled_watchdog(self, timeout_sec: int = 300):
        try:
            if self.enabled_event:
                self.enabled_event.set()
        except Exception:
            pass

        self.enabled_event = threading.Event()

        def _watch():
            ok = self.enabled_event.wait(timeout_sec)
            if not ok:
                try:
                    self.get_logger().error('测试失败：重启后5分钟未收到 ENABLED，结束测试。')
                except Exception:
                    print('测试失败：重启后5分钟未收到 ENABLED，结束测试。')
                rclpy.shutdown()

        threading.Thread(target=_watch, daemon=True).start()

    def _initial_restart(self):
        if self.restart_count >= MAX_RESTARTS:
            self.get_logger().info('Max restarts already reached before initial action.')
            rclpy.shutdown()
            return
        try:
            self.triggering = True
            self.get_logger().info('Initial restart at startup...')
            ok = restart_service_with_sudo(SUDO_PASSWORD, SERVICE_NAME)
            if ok:
                self.get_logger().info(
                    f'Initial restart OK ({self.restart_count}/{MAX_RESTARTS}).'
                )
                self._start_enabled_watchdog(timeout_sec=300)
            else:
                self.get_logger().error('Initial restart failed.')
        finally:
            if self.restart_count >= MAX_RESTARTS:
                self.get_logger().info('Reached max restarts after initial. Stopping node.')
                rclpy.shutdown()
            else:
                time.sleep(1)
                self.triggering = False

    def _on_msg(self, msg: String):
        if self.restart_count >= MAX_RESTARTS:
            self.get_logger().info('Max restarts reached. Shutting down node.')
            rclpy.shutdown()
            return

        try:
            payload_str = msg.data if hasattr(msg, 'data') else str(msg)
            data = json.loads(payload_str)
        except Exception as e:
            self.get_logger().warn(f'JSON parse failed: {e}')
            return

        status = data.get('robot_statusword', None)

        if status == 'ENABLED' and self.enabled_event:
            self.enabled_event.set()

        if status == 'ENABLED' and not self.triggering:
            self.triggering = True
            self.get_logger().info('Detected robot_statusword == ENABLED. Waiting 5s before restart...')
            threading.Thread(target=self._delayed_restart, daemon=True).start()

    def _delayed_restart(self):
        try:
            time.sleep(5)
            ok = restart_service_with_sudo(SUDO_PASSWORD, SERVICE_NAME)
            if ok:
                self.restart_count += 1
                self.get_logger().info(
                    f'Restarted {SERVICE_NAME} successfully ({self.restart_count}/{MAX_RESTARTS}).'
                )
                self._start_enabled_watchdog(timeout_sec=300)
            else:
                self.get_logger().error('Restart failed.')
        finally:
            if self.restart_count >= MAX_RESTARTS:
                self.get_logger().info('Reached max restarts. Stopping node.')
                rclpy.shutdown()
            else:
                time.sleep(1)
                self.triggering = False


def main():
    rclpy.init()
    node = RobotStateListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()