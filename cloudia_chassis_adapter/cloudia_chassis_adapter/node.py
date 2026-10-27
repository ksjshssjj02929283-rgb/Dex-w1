"""迦智底盘长连接适配器。

Cloudia 模式下本节点是底盘 WebSocket 的唯一所有者：接收 upstream_state，
发布位姿与完整原始状态 JSON，并把 /control/cmd_vel 以固定周期转换成
downstream_command/speedCommand；无输入时保持静默，超时停车后释放速度通道。
链路检测：状态帧中断超过 link_timeout_s 即断开重连；按周期发协议心跳，
网关回过心跳后，心跳回包超时同样断开重连。
默认后端为 dexe 时不建立连接、不发布状态。
"""

from __future__ import annotations

import json
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String

from .protocol import (build_heartbeat_command, build_speed_command,
                       is_heartbeat_response, parse_upstream_state)

try:
    import websocket
except ImportError:  # pragma: no cover - deployment dependency
    websocket = None


class CloudiaChassisAdapter(Node):

    def __init__(self):
        super().__init__("cloudia_chassis_adapter")
        default_backend = os.environ.get("DEXE_CHASSIS_BACKEND", "dexe")
        self.declare_parameter("backend", default_backend)
        self.declare_parameter("websocket_url", "")
        self.declare_parameter("robot_host", "")
        self.declare_parameter("state_port", 8852)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("pose_topic", "/feedback/chassis_pose")
        self.declare_parameter("raw_state_topic", "/feedback/cloudia_state_raw")
        self.declare_parameter("cmd_vel_topic", "/control/cmd_vel")
        self.declare_parameter("stale_timeout_s", 0.5)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("command_rate_hz", 10.0)
        self.declare_parameter("reconnect_min_s", 1.0)
        self.declare_parameter("reconnect_max_s", 10.0)
        self.declare_parameter("connect_timeout_s", 5.0)
        self.declare_parameter("use_safe", False)
        self.declare_parameter("serial_number", "")
        self.declare_parameter("link_timeout_s", 1.0)
        self.declare_parameter("heartbeat_period_s", 1.0)
        self.declare_parameter("heartbeat_timeout_s", 3.0)
        self.declare_parameter("client_id", "dexe-robot-server")

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
        self._link_timeout = max(
            0.2, float(self.get_parameter("link_timeout_s").value))
        self._heartbeat_period = max(
            0.1, float(self.get_parameter("heartbeat_period_s").value))
        self._heartbeat_timeout = max(
            self._heartbeat_period,
            float(self.get_parameter("heartbeat_timeout_s").value))
        self._client_id = str(self.get_parameter("client_id").value)

        self._pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("pose_topic").value), 10)
        self._raw_state_pub = self.create_publisher(
            String, str(self.get_parameter("raw_state_topic").value), 10)
        self._cmd_sub = self.create_subscription(
            Twist, str(self.get_parameter("cmd_vel_topic").value),
            self._on_cmd_vel, 10)

        # 发送决策与 socket 写入共用锁，避免退出/重连时重放旧速度。
        self._lock = threading.RLock()
        self._ws = None
        self._stop = threading.Event()
        self._last_state_time = None
        self._last_cmd_time = None
        self._last_cmd = (0.0, 0.0, 0.0)
        self._stop_frames_remaining = 0
        self._connected = False
        self._stale_reported = False
        self._thread = None
        self._connected_at = None
        self._last_heartbeat_sent = None
        self._last_heartbeat_ack = None
        self._heartbeat_seen = False
        self._heartbeat_missing_reported = False

        self._watchdog_timer = self.create_timer(0.1, self._watchdog)
        self._command_timer = self.create_timer(1.0 / self._command_rate,
                                                self._send_current_command)

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
        self._thread = threading.Thread(target=self._websocket_loop,
                                        name="cloudia-websocket",
                                        daemon=True)
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
            if self._stop.is_set():
                return
            self._last_cmd = (float(msg.linear.x), float(msg.linear.y),
                              float(msg.angular.z))
            self._last_cmd_time = time.monotonic()
            # 包括新鲜零速：故障停车必须持续转发，不能按速度是否为零释放。
            self._stop_frames_remaining = 3

    def _websocket_loop(self):
        reconnect_delay = self._reconnect_min
        while not self._stop.is_set():
            ws = None
            try:
                ws = websocket.WebSocket()
                ws.connect(self._url, timeout=self._connect_timeout)
                # 短接收超时：没有帧时也能及时做链路检测和心跳。
                ws.settimeout(0.2)
                with self._lock:
                    self._ws = ws
                    # 上次断开时停车发送失败，先补一次停车尝试再恢复发送。
                    self._send_zero(ws)
                    self._connected = True
                    # 连接恢复后只接受新的 ROS 指令与状态，不重放断线前缓存。
                    self._last_cmd_time = None
                    self._last_state_time = None
                    self._connected_at = time.monotonic()
                    self._last_heartbeat_sent = None
                    self._last_heartbeat_ack = None
                    self._heartbeat_seen = False
                    self._heartbeat_missing_reported = False
                reconnect_delay = self._reconnect_min
                self.get_logger().info("Cloudia WebSocket connected")
                while not self._stop.is_set():
                    now = time.monotonic()
                    reason = self._link_lost_reason(now)
                    if reason:
                        self.get_logger().warning(
                            f"Cloudia WebSocket link lost: {reason}; reconnecting"
                        )
                        break
                    self._send_heartbeat_if_due(ws, now)
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
                with self._lock:
                    self._connected = False
                    self._last_cmd_time = None
                    self._last_state_time = None
                    self._send_zero(ws)
                    if self._ws is ws:
                        self._ws = None
                if ws is not None:
                    try:
                        ws.close()
                    except Exception:
                        pass
            if not self._stop.is_set():
                self._stop.wait(reconnect_delay)
                reconnect_delay = min(self._reconnect_max,
                                      reconnect_delay * 2.0)

    def _handle_frame(self, raw):
        if not self._enabled:
            return
        try:
            text = raw.decode("utf-8") if isinstance(raw, bytes) else raw
            payload = json.loads(text)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"忽略非 JSON WebSocket 帧: {exc}")
            return
        if is_heartbeat_response(payload):
            with self._lock:
                self._last_heartbeat_ack = time.monotonic()
                self._heartbeat_seen = True
            return
        if (not isinstance(payload, dict) or
                payload.get("communicationType") != "upstream_state"):
            return

        # 转发原始文本，保留 header、位姿和全部未知字段，不重新序列化或补默认值。
        # 位姿提取失败不能阻断其它状态转发；异常状态也留给消费方检查。
        self._raw_state_pub.publish(String(data=text))
        state = parse_upstream_state(payload)
        if state is None:
            return
        with self._lock:
            self._last_state_time = time.monotonic()
        self._stale_reported = False
        self._publish_pose(state["pose"])

    def _publish_pose(self, pose):
        now = self.get_clock().now().to_msg()
        if pose is not None:
            msg = PoseStamped()
            msg.header.stamp = now
            msg.header.frame_id = self._frame_id
            msg.pose.position.x = pose["x"]
            msg.pose.position.y = pose["y"]
            msg.pose.orientation.z = math.sin(pose["theta"] / 2.0)
            msg.pose.orientation.w = math.cos(pose["theta"] / 2.0)
            self._pose_pub.publish(msg)

    def _link_lost_reason(self, now):
        """链路判死条件；返回原因字符串，链路正常返回 None。"""
        with self._lock:
            last_state = self._last_state_time
            connected_at = self._connected_at
            heartbeat_seen = self._heartbeat_seen
            last_ack = self._last_heartbeat_ack
            last_sent = self._last_heartbeat_sent
        reference = last_state if last_state is not None else connected_at
        if reference is not None and now - reference > self._link_timeout:
            return f"no upstream_state for {now - reference:.2f}s"
        if heartbeat_seen:
            if now - last_ack > self._heartbeat_timeout:
                return f"no heartbeatResponse for {now - last_ack:.2f}s"
        elif (last_sent is not None and connected_at is not None and
              now - connected_at > self._heartbeat_timeout and
              not self._heartbeat_missing_reported):
            # 网关从未回过心跳：只告警一次，链路仍按状态帧判断，避免反复重连。
            self._heartbeat_missing_reported = True
            self.get_logger().warning(
                "Cloudia gateway has not answered heartbeatCommand; "
                "link check falls back to upstream_state only")
        return None

    def _send_heartbeat_if_due(self, ws, now):
        """到期发一次心跳；发送失败抛异常，由外层断开重连。"""
        with self._lock:
            last_sent = self._last_heartbeat_sent
            if last_sent is not None and now - last_sent < self._heartbeat_period:
                return
            if self._ws is not ws:
                return
            payload = build_heartbeat_command(self._client_id,
                                              serial_number=self._serial_number)
            ws.send(json.dumps(payload, separators=(",", ":")))
            self._last_heartbeat_sent = now

    def _watchdog(self):
        if not self._enabled:
            return
        last_state = self._last_state_time
        if last_state is not None and time.monotonic(
        ) - last_state > self._stale_timeout:
            if not self._stale_reported:
                self._stale_reported = True
                self.get_logger().warning(
                    f"Cloudia 状态超过 {self._stale_timeout:.3f}s 未更新，停止发布过期状态。")

    def _send_current_command(self):
        if not self._enabled:
            return
        with self._lock:
            if self._stop.is_set() or not self._connected:
                return
            now = time.monotonic()
            command_fresh = (self._last_cmd_time is not None and
                             now - self._last_cmd_time <= self._command_timeout)
            if not command_fresh and self._stop_frames_remaining == 0:
                # 刚启动或已完成停车：只接收状态，不占用迦智速度仲裁通道。
                return
            state_fresh = (self._last_state_time is not None and
                           now - self._last_state_time <= self._stale_timeout)
            command = self._last_cmd if command_fresh and state_fresh else (0.0,
                                                                            0.0,
                                                                            0.0)
            if self._send_command(*command) and not command_fresh:
                # 仅成功写入 socket 才计数；这是发送完成，不是底盘停车 ACK。
                self._stop_frames_remaining -= 1

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
                return False
            try:
                ws.send(raw)
                return True
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"Cloudia speedCommand 发送失败: {exc}")
                return False

    def _send_zero(self, ws=None):
        with self._lock:
            if self._stop_frames_remaining == 0:
                # 从未控制或已经释放，断开/退出也不抢占正在运行的导航。
                return
            target = ws if ws is not None else self._ws
            if target is None:
                return
            payload = build_speed_command(
                0.0,
                0.0,
                0.0,
                use_safe=self._use_safe,
                serial_number=self._serial_number,
            )
            try:
                target.send(json.dumps(payload, separators=(",", ":")))
                self._stop_frames_remaining = 0
            except Exception:  # noqa: BLE001
                # 保留待停车状态；重连后先发零速，不恢复旧的非零速度。
                pass

    def close(self):
        self._stop.set()
        with self._lock:
            self._send_zero()
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
