"""迦智底盘适配特性。

协议细节只允许停留在本子包内；门面继续使用现有底盘接口名字。
"""

from .http import CloudiaApiError, CloudiaHttpClient
from .nav_points import CloudiaNavPointsFeature
from .navigation import CloudiaNavigationFeature, CloudiaTaskClient
from .precise_control import CloudiaPreciseControlFeature

__all__ = [
    "CloudiaApiError",
    "CloudiaHttpClient",
    "CloudiaNavPointsFeature",
    "CloudiaNavigationFeature",
    "CloudiaPreciseControlFeature",
    "CloudiaTaskClient",
]
