"""W1Manager 回充 Action 接口单元测试。

W1Manager 模块导入时会加载实机运动学依赖。本测试只提取并执行回充相关方法，
避免单元测试因未安装 Pinocchio 等非回充依赖而失去覆盖。
"""

import ast
from enum import Enum
from pathlib import Path
from unittest.mock import Mock

import pytest


class StatusCode(Enum):
    SUCCESS = 1
    CHASSIS_NOT_USE = -500
    CHASSIS_NOT_CONNECT = -501
    CHASSIS_SET_AUTO_CHARGING_FAIL = -508
    CHASSIS_CANCEL_AUTO_CHARGING_FAIL = -509


def _load_w1_manager_charging_methods():
    """从源码加载回充方法，隔离与本次测试无关的硬件依赖。"""
    source_path = (Path(__file__).parents[1] / "src" / "dexe_robot_client" /
                   "w1" / "w1_manager.py")
    module = ast.parse(source_path.read_text(encoding="utf-8"))
    class_node = next(
        node for node in module.body
        if isinstance(node, ast.ClassDef) and node.name == "W1Manager")
    method_names = {
        "_has_chassis_client_initialized",
        "set_auto_charging",
        "get_auto_charging_status",
        "get_auto_charging_result",
        "cancel_auto_charging",
    }
    methods = [
        node for node in class_node.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and
        node.name in method_names
    ]
    isolated_class = ast.ClassDef(name="W1Manager",
                                  bases=[],
                                  keywords=[],
                                  body=methods,
                                  decorator_list=[])
    isolated_module = ast.Module(body=[isolated_class], type_ignores=[])
    ast.fix_missing_locations(isolated_module)
    namespace = {"StatusCode": StatusCode}
    exec(compile(isolated_module, source_path, "exec"), namespace)
    return namespace["W1Manager"]


W1Manager = _load_w1_manager_charging_methods()


@pytest.fixture
def manager():
    """绕过硬件初始化，只验证 W1Manager 的回充接口转换逻辑。"""
    instance = W1Manager()
    instance._use_nav = True
    instance.logger = Mock()
    instance.chassis_client = Mock()
    return instance


def test_set_auto_charging_does_not_require_pose_connection(manager):
    """位姿反馈断流不应阻止向可用的回充 Action Server 发送 Goal。"""
    manager.chassis_client.is_connected.return_value = False
    manager.chassis_client.set_chassis_auto_charging.return_value = True

    status = manager.set_auto_charging(force=False, timeout_sec=2.5)

    assert status == StatusCode.SUCCESS
    manager.chassis_client.is_connected.assert_not_called()
    manager.chassis_client.set_chassis_auto_charging.assert_called_once_with(
        force=False, timeout_sec=2.5)


@pytest.mark.parametrize(
    ("use_nav", "client", "expected"),
    [
        (False, Mock(), StatusCode.CHASSIS_NOT_USE),
        (True, None, StatusCode.CHASSIS_NOT_CONNECT),
    ],
)
def test_set_auto_charging_checks_client_initialization(manager, use_nav,
                                                        client, expected):
    manager._use_nav = use_nav
    manager.chassis_client = client

    assert manager.set_auto_charging() == expected


def test_set_auto_charging_maps_send_failure(manager):
    manager.chassis_client.set_chassis_auto_charging.return_value = False

    assert (manager.set_auto_charging() ==
            StatusCode.CHASSIS_SET_AUTO_CHARGING_FAIL)


def test_get_auto_charging_status_and_result(manager):
    manager.chassis_client.get_auto_charging_status.return_value = "SUCCEEDED"
    manager.chassis_client.get_auto_charging_result.return_value = (True,
                                                                    "回充成功")

    assert manager.get_auto_charging_status() == (StatusCode.SUCCESS,
                                                  "SUCCEEDED")
    assert manager.get_auto_charging_result() == (StatusCode.SUCCESS, True,
                                                  "回充成功")


def test_cancel_auto_charging_maps_send_failure(manager):
    manager.chassis_client.cancel_auto_charging.return_value = False

    assert (manager.cancel_auto_charging() ==
            StatusCode.CHASSIS_CANCEL_AUTO_CHARGING_FAIL)
