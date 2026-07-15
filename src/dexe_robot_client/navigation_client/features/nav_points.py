"""导航点查询：/map_asset_manager/get_all_nav_points（同步 service）。

返回 dict[点位名称] -> (x, y, theta_rad)，theta 单位与路网导航 waypoints 一致（弧度）。
"""

from nav_interfaces.msg import NavPoint
from nav_interfaces.srv import GetAllNavPoints

from ..common import call_service


class NavPointsFeature:

    def __init__(self, node, cfg):
        self._node = node
        self._client = (node.create_client(GetAllNavPoints, cfg.nav_points_service)
                        if cfg.nav_points_service else None)

    def get_all(self) -> dict:
        response = call_service(
            self._node, self._client, GetAllNavPoints.Request(),
            timeout_sec=10.0, wait_service_sec=1.0, label="获取导航点")
        if response is None or not response.success:
            return {}

        points = {}
        for point in response.points:
            if not isinstance(point, NavPoint) or not point.name:
                self._node.get_logger().warning(
                    "[导航点跳过] 类型异常或名称为空。")
                continue
            points[str(point.name)] = (
                float(point.x), float(point.y), float(point.theta))
        return points
