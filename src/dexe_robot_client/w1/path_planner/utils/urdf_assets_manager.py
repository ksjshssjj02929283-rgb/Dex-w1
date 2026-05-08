# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, List


@dataclass
class FilePath:
    """文件路径配置"""
    robot_urdf: str
    robot_with_ee_urdf: str
    rvis: Optional[str] = None
    mjcf: Optional[str] = None

    @classmethod
    def from_dict(cls, data: dict) -> "FilePath":
        """从字典创建 FilePath"""
        return cls(
            robot_urdf=data.get("robot_urdf"),
            robot_with_ee_urdf=data.get("robot_with_ee_urdf"),
            rvis=data.get("rvis"),
            mjcf=data.get("mjcf"),
        )

    def to_dict(self) -> dict:
        """转换为字典"""
        return {
            "robot_urdf": self.robot_urdf,
            "robot_with_ee_urdf": self.robot_with_ee_urdf,
            "rvis": self.rvis,
            "mjcf": self.mjcf,
        }


@dataclass
class URDFConfig:
    """URDF 配置"""
    robot_type: str
    version: str
    file_path: FilePath
    ros2_to_urdf_map: Dict[str, str]
    left_hand_type: Optional[str] = None
    right_hand_type: Optional[str] = None

    @classmethod
    def from_dict(cls, data: dict) -> "URDFConfig":
        """从字典创建 URDFConfig"""
        return cls(
            robot_type=data.get("robot_type"),
            version=data.get("version"),
            left_hand_type=data.get("left_hand_type"),
            right_hand_type=data.get("right_hand_type"),
            file_path=FilePath.from_dict(data.get("file_path", {})),
            ros2_to_urdf_map=data.get("ros2_to_urdf_map", {}),
        )

    def to_dict(self) -> dict:
        """转换为字典"""
        return {
            "robot_type": self.robot_type,
            "version": self.version,
            "left_hand_type": self.left_hand_type,
            "right_hand_type": self.right_hand_type,
            "file_path": self.file_path.to_dict(),
            "ros2_to_urdf_map": self.ros2_to_urdf_map,
        }


class URDFConfigLoader:
    """URDF 配置文件加载器"""

    DEFAULT_CONFIG_PATH = Path.home() / ".cache/dexe_data/config.json"

    def __init__(self, config_path: Optional[str | Path] = None):
        """使用配置文件路径初始化加载器

        Args:
            config_path: config.json 的路径。若为 None，则使用默认路径。
        """
        if config_path is None:
            config_path = self.DEFAULT_CONFIG_PATH
        self.config_path = Path(config_path)
        self.config: Optional[URDFConfig] = None

        # ROS2 控制部位定义
        self.ros2_control_parts = {
            "torso": ["ANKLE", "KNEE", "BUTTOCK", "WAIST"],
            "head": ["NECK1", "NECK2"],
            "left_arm": [
                "LEFT_J1", "LEFT_J2", "LEFT_J3", "LEFT_J4", "LEFT_J5",
                "LEFT_J6", "LEFT_J7"
            ],
            "right_arm": [
                "RIGHT_J1", "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5",
                "RIGHT_J6", "RIGHT_J7"
            ],
        }

    def load(self) -> URDFConfig:
        """从文件加载配置

        Returns:
            URDFConfig: 已加载的配置对象

        Raises:
            FileNotFoundError: 配置文件不存在
            json.JSONDecodeError: 配置文件 JSON 格式非法
        """
        if not self.config_path.exists():
            raise FileNotFoundError(
                f"Config file not found: {self.config_path}")

        with open(self.config_path, "r") as f:
            data = json.load(f)

        self.config = URDFConfig.from_dict(data)
        return self.config

    def save(self, config: Optional[URDFConfig] = None) -> None:
        """将配置保存到文件

        Args:
            config: 要保存的配置。若为 None，则使用当前已加载配置。

        Raises:
            ValueError: 没有可保存的配置
        """
        if config is None:
            config = self.config

        if config is None:
            raise ValueError("No configuration to save")

        self.config_path.parent.mkdir(parents=True, exist_ok=True)

        with open(self.config_path, "w") as f:
            json.dump(config.to_dict(), f, indent=2)

    def get_robot_urdf(self) -> str:
        """获取机器人 URDF 路径"""
        if self.config is None:
            self.load()
        return self.config.file_path.robot_urdf

    def get_robot_with_ee_urdf(self) -> str:
        """获取带末端执行器的机器人 URDF 路径"""
        if self.config is None:
            self.load()
        return self.config.file_path.robot_with_ee_urdf

    def get_ros2_to_urdf_map(self) -> Dict[str, str]:
        """获取 ROS2 到 URDF 的关节名映射"""
        if self.config is None:
            self.load()
        return self.config.ros2_to_urdf_map

    def get_urdf_control_joints(self) -> List[str]:
        """获取需要控制的 URDF 关节名列表

        Returns:
            List[str]: URDF 关节名列表
        """
        if self.config is None:
            self.load()
        return list(self.config.ros2_to_urdf_map.values())

    def get_ros2_control_joints(self) -> List[str]:
        """获取需要控制的 ROS2 关节名列表

        Returns:
            List[str]: ROS2 关节名列表
        """
        if self.config is None:
            self.load()
        return list(self.config.ros2_to_urdf_map.keys())

    def get_urdf_control_parts(self) -> Dict[str, List[str]]:
        """获取按部位划分的 URDF 控制关节

        Returns:
            Dict[str, List[str]]: 身体部位到 URDF 关节名列表的映射，例如：
                {
                    "torso": ["ANKLE", "KNEE", "BUTTOCK", "WAIST"]（已转换为 URDF 名称）, 
                    "head": ["NECK1", "NECK2"],
                    "left_arm": ["LEFT_J1", ...]（已转换为 URDF 名称）, 
                    "right_arm": ["RIGHT_J1", ...]（已转换为 URDF 名称）
                }
        """
        if self.config is None:
            self.load()

        # 将 ROS2 关节名转换为 URDF 关节名
        urdf_control_parts = {
            part: [self.config.ros2_to_urdf_map[joint] for joint in joints]
            for part, joints in self.ros2_control_parts.items()
        }
        return urdf_control_parts

    def ros2_to_urdf(self, ros2_joint_name: str) -> str:
        """将 ROS2 关节名转换为 URDF 关节名

        Args:
            ros2_joint_name: ROS2 关节名

        Returns:
            str: 对应的 URDF 关节名

        Raises:
            KeyError: 映射中不存在该关节名
        """
        if self.config is None:
            self.load()
        if ros2_joint_name not in self.config.ros2_to_urdf_map:
            raise KeyError(
                f"Joint '{ros2_joint_name}' not found in ROS2 to URDF mapping")
        return self.config.ros2_to_urdf_map[ros2_joint_name]

    def is_valid(self) -> bool:
        """校验配置是否合法

        Returns:
            bool: 若配置合法返回 True，否则返回 False
        """
        if self.config is None:
            return False

        # 检查必填字段
        if not self.config.robot_type:
            return False
        if not self.config.version:
            return False
        if not self.config.file_path.robot_urdf:
            return False
        if not self.config.file_path.robot_with_ee_urdf:
            return False

        # 检查文件是否存在
        if not Path(self.config.file_path.robot_urdf).exists():
            return False
        if not Path(self.config.file_path.robot_with_ee_urdf).exists():
            return False

        return True

    def get_urdf_base_dir(self) -> Path:
        """获取机器人 URDF 的基础目录（robot.urdf 的父目录）

        Returns:
            Path: robot_urdf 文件的父目录
        """
        if self.config is None:
            self.load()
        return Path(self.config.file_path.robot_urdf).parent

    @classmethod
    def from_default_path(cls) -> "URDFConfigLoader":
        """使用默认配置路径创建并立即加载

        Returns:
            URDFConfigLoader: 已初始化且已加载的加载器
        """
        loader = cls()
        loader.load()
        return loader


class URDFAssetsManager:
    """基于配置路径的 URDF 资源管理器"""

    def __init__(self, config_loader: Optional[URDFConfigLoader] = None):
        """使用配置加载器初始化资源管理器

        Args:
            config_loader: URDFConfigLoader 实例。若为 None，则使用默认路径创建。
        """
        if config_loader is None:
            config_loader = URDFConfigLoader.from_default_path()

        self.config_loader = config_loader
        self.base_dir = config_loader.get_urdf_base_dir()

    def get_urdf_dir(self) -> str:
        """获取 URDF 目录（机器人 URDF 的父目录）

        Returns:
            str: URDF 目录路径
        """
        return str(self.base_dir)

    def get_urdf_path(self) -> str:
        """获取机器人 URDF 路径

        Returns:
            str: robot.urdf 完整路径
        """
        return self.config_loader.get_robot_urdf()

    def get_ee_urdf_path(self) -> str:
        """获取带末端执行器的机器人 URDF 路径

        Returns:
            str: robot_with_ee.urdf 完整路径
        """
        return self.config_loader.get_robot_with_ee_urdf()

    def list_urdf_files(self) -> list[str]:
        """列出基础目录下所有 URDF 文件

        Returns:
            list[str]: URDF 文件路径列表
        """
        if not self.base_dir.exists():
            return []
        return sorted([str(f) for f in self.base_dir.glob("*.urdf")])

    def get_mesh_files(self, subdirs: bool = True) -> list[str]:
        """获取基础目录下的网格文件

        Args:
            subdirs: 是否包含子目录中的文件

        Returns:
            list[str]: 网格文件路径列表
        """
        if not self.base_dir.exists():
            return []

        pattern = "**/*.*" if subdirs else "*.*"
        mesh_exts = {".dae", ".obj", ".stl", ".ply"}

        files = []
        for f in self.base_dir.glob(pattern):
            if f.suffix.lower() in mesh_exts:
                files.append(str(f))
        return sorted(files)

    def get_parent_dir(self) -> str:
        """获取 URDF 基础目录的父目录

        Returns:
            str: 父目录（例如 assembled/w1 的父目录 assembled）
        """
        return str(self.base_dir.parent)

    def get_cache_dir(self) -> str:
        """获取缓存目录（base_dir 的上上级目录）

        Returns:
            str: 缓存根目录
        """
        return str(self.base_dir.parent.parent)

    def get_urdf_control_joints(self) -> List[str]:
        """获取需要控制的 URDF 关节名列表

        Returns:
            List[str]: URDF 关节名列表
        """
        return self.config_loader.get_urdf_control_joints()

    def get_ros2_control_joints(self) -> List[str]:
        """获取需要控制的 ROS2 关节名列表

        Returns:
            List[str]: ROS2 关节名列表
        """
        return self.config_loader.get_ros2_control_joints()

    def get_urdf_control_parts(self) -> Dict[str, List[str]]:
        """获取按部位划分的 URDF 控制关节

        Returns:
            Dict[str, List[str]]: 身体部位到 URDF 关节名列表的映射
        """
        return self.config_loader.get_urdf_control_parts()


if __name__ == "__main__":
    # 方式1：最简洁 - 直接加载
    loader = URDFConfigLoader.from_default_path()
    print(f"Robot URDF: {loader.get_robot_urdf()}")
    print(f"Robot with EE: {loader.get_robot_with_ee_urdf()}")

    # 方式2：使用资源管理器
    assets = URDFAssetsManager()
    print(f"URDF Base Dir: {assets.get_urdf_dir()}")
    print(f"Robot URDF Path: {assets.get_urdf_path()}")
    print(f"Robot with EE URDF Path: {assets.get_ee_urdf_path()}")
    # print(f"URDF Files: {assets.list_urdf_files()}")
    # print(f"Mesh Files: {assets.get_mesh_files()}")

    # 方式3：自定义路径
    # custom_loader = URDFConfigLoader("/custom/path/config.json")
    # custom_loader.load()
    # print(f"Custom config loaded: {custom_loader.config.robot_type}")

    from IPython import embed
    embed(header="URDF Config Loader")
