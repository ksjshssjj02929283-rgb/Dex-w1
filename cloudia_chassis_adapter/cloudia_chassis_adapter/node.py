"""迦智底盘长连接适配器。

Cloudia 模式下本节点是底盘 WebSocket 的唯一所有者：接收 upstream_state，
发布现有 ROS 2 状态话题，并把 /control/cmd_vel 以固定周期转换成
downstream_command/speedCommand。默认后端为 dexe 时不建立连接、不发布状态。
"""

from __future__ import annotations

import json
import math
import os
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool

from .protocol import build_speed_command, parse_upstream_state

try:
    import websocket
except ImportError:  # pragma: no cover - deployment dependency
    websocket = None


class CloudiaChassisAdapter(Node):

    POWER_STATUS_NAME = "/Devices/System/PowerBoard/collector_node: PowerBoard"
    CONNECTION_STATUS_NAME = "Cloudia WebSocket"

    def __init__(self):
        super().__init__("cloudia_chassis_adapter")
        default_backend = os.environ.get("DEXE_CHASSIS_BACKEND", "dexe")
        self.declare_parameter("backend", default_backend)
        self.declare_parameter("websocket_url", "")
        self.declare_parameter("robot_host", "")
        self.declare_parameter("state_port", 8852)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("pose_topic", "/feedback/chassis_pose")
        self.declare_parameter("battery_topic", "/diagnostics_agg")
        self.declare_parameter("obstacle_topic", "obstacle_detector/has_obstacle")
        self.declare_parameter("velocity_topic", "/cloudia/chassis_velocity")
        self.declare_parameter("cmd_vel_topic", "/control/cmd_vel")
        self.declare_parameter("stale_timeout_s", 0.5)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("command_rate_hz", 10.0)
        self.declare_parameter("reconnect_min_s", 1.0)
        self.declare_parameter("reconnect_max_s", 10.0)
        self.declare_parameter("connect_timeout_s", 5.0)
        self.declare_parameter("use_safe", False)
        self.declare_parameter("serial_number", "")

        self._backend = str(self.get_parameter("backend").value).lower()
        self._enabled = self._backend == "cloudia"
        self._url = self._resolve_url()
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._stale_timeout = float(self.get_parameter("stale_timeout_s").value)
        self._command_timeout = float(
            self.get_parameter("command_timeout_s").value)
        self._command_rate = max(
            1.0, float(self.get_parameter("command_rate_hz").value))
        self._reconnect_min = max(
            0.1, float(self.get_parameter("reconnect_min_s").value))
        self._reconnect_max = max(
            self._reconnect_min,
            float(self.get_parameter("reconnect_max_s").value))
        self._connect_timeout = float(
            self.get_parameter("connect_timeout_s").value)
        self._use_safe = bool(self.get_parameter("use_safe").value)
        self._serial_number = str(self.get_parameter("serial_number").value)

        self._pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("pose_topic").value), 10)
        self._battery_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("battery_topic").value), 10)
        self._obstacle_pub = self.create_publisher(
            Bool, str(self.get_parameter("obstacle_topic").value), 10)
        velocity_topic = str(self.get_parameter("velocity_topic").value)
        self._velocity_pub = (self.create_publisher(TwistStamped, velocity_topic, 10)
                              if velocity_topic else None)
        self._cmd_sub = self.create_subscription(
            Twist, str(self.get_parameter("cmd_vel_topic").value),
            self._on_cmd_vel, 10)

        self._lock = threading.Lock()
        self._ws = None
        self._stop = threading.Event()
        self._last_state_time = None
        self._last_cmd_time = None
        self._last_cmd = (0.0, 0.0, 0.0)
        self._connected = False
        self._stale_reported = False
        self._last_diag_time = 0.0
        self._thread = None

        self._watchdog_timer = self.create_timer(0.1, self._watchdog)
        self._command_timer = self.create_timer(
            1.0 / self._command_rate, self._send_current_command)

        if not self._enabled:
            self.get_logger().info(
                f"backend={self._backend}，Cloudia 适配器已禁用；保持自研底盘链路。")
            return
        if websocket is None or not hasattr(websocket, "WebSocket"):
            raise RuntimeError(
                "缺少 websocket-client；请安装 python3-websocket 或 websocket-client")
        if not self._url:
            raise RuntimeError("Cloudia WebSocket 地址未配置")
        self.get_logger().info(
            f"Cloudia WebSocket adapter connecting to {self._url}")
        self._thread = threading.Thread(
            target=self._websocket_loop, name="cloudia-websocket", daemon=True)
        self._thread.start()

    def _resolve_url(self):
        configured = str(self.get_parameter("websocket_url").value).strip()
        if configured:
            return configured
        host = str(self.get_parameter("robot_host").value).strip()
        if not host:
            host = os.environ.get("CLOUDIA_HOST", "").strip()
        if not host:
            return ""
        port = int(self.get_parameter("state_port").value)
        return f"ws://{host}:{port}/"

    def _on_cmd_vel(self, msg):
        with self._lock:
            self._last_cmd = (float(msg.linear.x), float(msg.linear.y),
                              float(msg.angular.z))
            self._last_cmd_time = time.monotonic()

    def _websocket_loop(self):
        reconnect_delay = self._reconnect_min
        while not self._stop.is_set():
            ws = None
            try:
                ws = websocket.WebSocket()
                ws.connect(self._url, timeout=self._connect_timeout)
                ws.settimeout(1.0)
                with self._lock:
                    self._ws = ws
                    self._connected = True
                reconnect_delay = self._reconnect_min
                self._publish_connection_diagnostic(True, "connected")
                self.get_logger().info("Cloudia WebSocket connected")
                while not self._stop.is_set():
                    try:
                        raw = ws.recv()
                    except websocket.WebSocketTimeoutException:
                        continue
                    if raw is None:
                        break
                    self._handle_frame(raw)
            except Exception as exc:
                self.get_logger().warning(f"Cloudia WebSocket error: {exc}")
            finally:
                self._send_zero(ws)
                with self._lock:
                    if self._ws is ws:
                        self._ws = None
                    self._connected = False
                self._publish_connection_diagnostic(False, "disconnected")
                if ws is not None:
                    try:
                        ws.close()
                    except Exception:
                        pass
            if not self._stop.is_set():
                self._stop.wait(reconnect_delay)
                reconnect_delay = min(self._reconnect_max, reconnect_delay * 2.0)

    def _handle_frame(self, raw):
        try:
            payload = json.loads(raw.decode("utf-8") if isinstance(raw, bytes) else raw)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"忽略非 JSON WebSocket 帧: {exc}")
            return
        state = parse_upstream_state(payload)
        if state is None:
            return
        self._last_state_time = time.monotonic()
        self._stale_reported = False
        self._publish_state(state)

    def _publish_state(self, state):
        now = self.get_clock().now().to_msg()
        pose = state.get("pose")
        if pose is not None:
            msg = PoseStamped()
            msg.header.stamp = now
            msg.header.frame_id = self._frame_id
            msg.pose.position.x = pose["x"]
            msg.pose.position.y = pose["y"]
            msg.pose.orientation.z = math.sin(pose["theta"] / 2.0)
            msg.pose.orientation.w = math.cos(pose["theta"] / 2.0)
            self._pose_pub.publish(msg)

        velocity = state.get("velocity") or {}
        if self._velocity_pub is not None:
            msg = TwistStamped()
            msg.header.stamp = now
            msg.header.frame_id = self._frame_id
            msg.twist.linear.x = velocity.get("vx", 0.0)
            msg.twist.linear.y = velocity.get("vy", 0.0)
            msg.twist.angular.z = velocity.get("omega", 0.0)
            self._velocity_pub.publish(msg)

        obstacle = Bool()
        obstacle.data = bool(state.get("obstacle_stop", False))
        self._obstacle_pub.publish(obstacle)

        battery = state.get("battery_percentage")
        if battery is not None:
            self._publish_battery_diagnostic(battery)

    def _publish_battery_diagnostic(self, percentage):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = self.POWER_STATUS_NAME
        status.message = "Cloudia state connected"
        status.values = [KeyValue(key="battery_percentage", value=f"{percentage:.3f}")]
        array.status = [status]
        self._battery_pub.publish(array)

    def _publish_connection_diagnostic(self, connected, message):
        now = time.monotonic()
        if now - self._last_diag_time < 0.2 and connected:
            return
        self._last_diag_time = now
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK if connected else DiagnosticStatus.ERROR
        status.name = self.CONNECTION_STATUS_NAME
        status.message = message
        array.status = [status]
        self._battery_pub.publish(array)

    def _watchdog(self):
        if not self._enabled:
            return
        last_state = self._last_state_time
        if last_state is not None and time.monotonic() - last_state > self._stale_timeout:
            if not self._stale_reported:
                self._stale_reported = True
                self.get_logger().warning(
                    f"Cloudia 状态超过 {self._stale_timeout:.3f}s 未更新，停止发布过期状态。")
                self._publish_connection_diagnostic(False, "state stale")

    def _send_current_command(self):
        if not self._enabled:
            return
        now = time.monotonic()
        with self._lock:
            command = self._last_cmd
            command_time = self._last_cmd_time
            connected = self._connected
            state_time = self._last_state_time
        state_fresh = state_time is not None and now - state_time <= self._stale_timeout
        if (not state_fresh or command_time is None
                or now - command_time > self._command_timeout):
            command = (0.0, 0.0, 0.0)
        if connected:
            self._send_command(*command)

    def _send_command(self, vx, vy, omega):
        payload = build_speed_command(
            vx,
            vy,
            omega,
            use_safe=self._use_safe,
            serial_number=self._serial_number,
        )
        raw = json.dumps(payload, separators=(",", ":"))
        with self._lock:
            ws = self._ws
            if ws is None:
                return
            try:
                ws.send(raw)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"Cloudia speedCommand 发送失败: {exc}")

    def _send_zero(self, ws=None):
        payload = build_speed_command(
            0.0,
            0.0,
            0.0,
            use_safe=self._use_safe,
            serial_number=self._serial_number,
        )
        raw = json.dumps(payload, separators=(",", ":"))
        with self._lock:
            target = ws or self._ws
            if target is None:
                return
            try:
                target.send(raw)
            except Exception:  # noqa: BLE001
                pass

    def close(self):
        self._send_zero()
        self._stop.set()
        with self._lock:
            ws = self._ws
        if ws is not None:
            try:
                ws.close()
            except Exception:  # noqa: BLE001
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CloudiaChassisAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
