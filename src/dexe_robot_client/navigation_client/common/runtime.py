"""执行器感知的 spin / future 等待工具 + standalone 后台执行器。

规则（源自"多 Executor 索引冲突"修复）：
- 节点被外部 Executor（如 W1Manager）管理时，绝不在节点内再 spin；
- 需要同步等待 future 时，独立模式用 spin_until_future_complete，
  executor 模式用 busy-wait 轮询（若调用线程本身是 executor 回调线程，
  future 无法被调度完成，只会超时——调用方需避免在回调里同步等待）。
"""

import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor


def is_managed_by_executor(node) -> bool:
    executor = node.executor
    return executor is not None and node in executor.get_nodes()


def spin_once(node, timeout_sec: float = 0.1):
    """仅独立运行时处理一次回调；被外部 Executor 管理时为空操作。"""
    if is_managed_by_executor(node):
        return
    rclpy.spin_once(node, timeout_sec=timeout_sec)


def spin(node):
    """独立运行时的阻塞 spin；被外部 Executor 管理时为空操作。"""
    if is_managed_by_executor(node):
        return
    while rclpy.ok():
        spin_once(node, timeout_sec=0.1)


def wait_for_future(node, future, timeout_sec: float) -> bool:
    """等待 future 完成，返回是否完成（不抛异常）。"""
    if not is_managed_by_executor(node):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        return future.done()

    deadline = time.monotonic() + timeout_sec
    while not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            node.get_logger().warn(
                f"wait_for_future 超时（executor 模式，timeout={timeout_sec}s）。"
                "可能原因：服务端未响应，或当前线程就是 executor 回调线程。")
            return False
        time.sleep(min(0.01, remaining))
    return True


class StandaloneRunner:
    """standalone=True 时为节点自建后台执行器线程（用完调 close()）。"""

    def __init__(self, node):
        self._node = node
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def close(self, join_timeout: float = 3.0):
        self._executor.shutdown()
        self._thread.join(timeout=join_timeout)
