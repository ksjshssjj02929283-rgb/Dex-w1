"""统一的 service 同步调用封装：未初始化守卫 + 可用性检查 + 执行器感知等待。"""

from . import runtime


def call_service(node, client, request, *, timeout_sec: float = 1.0,
                 wait_service_sec: float = 0.5, label: str = ""):
    """同步调用 service。失败返回 None（并记日志），成功返回 response。"""
    tag = f"[{label}] " if label else ""
    if client is None:
        node.get_logger().warn(f"{tag}服务客户端未初始化（当前模式不支持），忽略调用。")
        return None
    if not client.wait_for_service(timeout_sec=wait_service_sec):
        node.get_logger().error(f"{tag}服务 {client.srv_name} 不可用。")
        return None
    future = client.call_async(request)
    if not runtime.wait_for_future(node, future, timeout_sec):
        node.get_logger().error(f"{tag}服务 {client.srv_name} 调用超时。")
        return None
    try:
        return future.result()
    except Exception as e:  # noqa: BLE001 —— 失败必须体现在返回值
        node.get_logger().error(f"{tag}服务 {client.srv_name} 响应异常：{e}")
        return None
