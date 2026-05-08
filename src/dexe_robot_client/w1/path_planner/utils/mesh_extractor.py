# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from pathlib import Path
import pinocchio as pin
import os
from typing import List, Optional, Set, Dict, Iterable
from collections import deque


class PinocchioMeshExtractor:
    """
    Pinocchio Mesh 文件路径提取器
    支持通过 joint 或 link 名称，获取该关节/连杆及其所有子连杆的 mesh 文件路径列表
    """

    def __init__(self,
                 urdf_path: str,
                 package_dirs: Optional[List[str]] = None,
                 verbose: bool = False):
        """
        初始化 Mesh 路径提取器
        
        Args:
            urdf_path: URDF 文件的路径
            package_dirs: 包含 mesh 文件的包目录列表，用于解析 package:// 路径
            verbose: 是否打印详细信息
        """
        self.urdf_path = urdf_path
        self.package_dirs = package_dirs or []
        self.verbose = verbose

        # 模型对象
        self.model: Optional[pin.Model] = None
        self.collision_model: Optional[pin.GeometryModel] = None
        self.visual_model: Optional[pin.GeometryModel] = None
        self.data: Optional[pin.Data] = None

        # 缓存提取的 mesh 路径
        self._visual_mesh_paths: Optional[List[str]] = None
        self._collision_mesh_paths: Optional[List[str]] = None

        # 缓存 link 到 mesh 路径的映射
        self._link_to_visual_mesh_paths: Optional[Dict[str, List[str]]] = None
        self._link_to_collision_mesh_paths: Optional[Dict[str,
                                                          List[str]]] = None

        # 加载模型
        self._load_model()

        # 构建映射关系
        self._build_link_mesh_maps()

    def _load_model(self):
        """加载 URDF 模型并构建几何模型"""
        try:
            if self.verbose:
                print(f"正在加载 URDF 文件: {self.urdf_path}")

            # 构建模型
            self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
                self.urdf_path, self.package_dirs, pin.JointModelFreeFlyer())

            # 创建 data 对象
            self.data = self.model.createData()

            if self.verbose:
                print(f"模型加载成功!")
                print(f"  - 关节数量: {self.model.njoints}")
                print(f"  - 连杆数量: {self.model.nframes}")
                print(f"  - 视觉几何体数量: {len(self.visual_model.geometryObjects)}")
                print(
                    f"  - 碰撞几何体数量: {len(self.collision_model.geometryObjects)}")

        except Exception as e:
            raise RuntimeError(f"模型加载失败: {str(e)}")

    def _build_link_mesh_maps(self):
        """构建连杆到 mesh 文件路径的映射"""
        self._link_to_visual_mesh_paths = {}
        self._link_to_collision_mesh_paths = {}

        # 构建视觉映射
        for geom in self.visual_model.geometryObjects:
            link_name = self.model.frames[geom.parentFrame].name
            mesh_path = self._get_mesh_file_path(geom)
            if mesh_path:
                if link_name not in self._link_to_visual_mesh_paths:
                    self._link_to_visual_mesh_paths[link_name] = []
                self._link_to_visual_mesh_paths[link_name].append(mesh_path)

        # 构建碰撞映射
        for geom in self.collision_model.geometryObjects:
            link_name = self.model.frames[geom.parentFrame].name
            mesh_path = self._get_mesh_file_path(geom)
            if mesh_path:
                if link_name not in self._link_to_collision_mesh_paths:
                    self._link_to_collision_mesh_paths[link_name] = []
                self._link_to_collision_mesh_paths[link_name].append(mesh_path)

    def _get_mesh_file_path(self, geom) -> Optional[str]:
        """
        从几何对象中提取 mesh 文件的绝对路径
        
        Returns:
            mesh 文件的绝对路径，如果没有 mesh 文件则返回 None
        """
        mesh_path = None
        if hasattr(geom, 'meshPath') and geom.meshPath:
            mesh_path = geom.meshPath
        elif hasattr(geom, 'filename') and geom.filename:
            mesh_path = geom.filename

        if mesh_path:
            # 转换为绝对路径
            if not os.path.isabs(mesh_path):
                # 尝试在 package_dirs 中查找
                for package_dir in self.package_dirs:
                    potential_path = os.path.join(package_dir, mesh_path)
                    if os.path.exists(potential_path):
                        mesh_path = os.path.abspath(potential_path)
                        break
                else:
                    # 如果找不到，保持原样但记录警告
                    if self.verbose:
                        print(f"警告: 找不到 mesh 文件 {mesh_path}")
            else:
                mesh_path = os.path.abspath(mesh_path)

        return mesh_path

    def _get_link_name_from_joint(self, joint_name: str) -> Optional[str]:
        """
        根据关节名称获取对应的子连杆名称
        在 URDF 中，关节通常连接到其子连杆
        
        Returns:
            连杆名称，如果找不到则返回 None
        """
        try:
            joint_id = self.model.getJointId(joint_name)
            if joint_id > 0:  # 跳过 universe 关节
                for frame in self.model.frames:
                    if frame.parentJoint == joint_id and frame.type == pin.FrameType.BODY:
                        return frame.name
        except Exception as e:
            if self.verbose:
                print(f"警告: 找不到关节 '{joint_name}' - {str(e)}")
        return None

    def _get_all_child_links(self, root_link_name: str) -> Set[str]:
        """
        获取某个连杆下的所有子连杆（包括自身）
        使用 BFS 遍历框架树
        
        Args:
            root_link_name: 根连杆名称
            
        Returns:
            所有子连杆名称的集合（包括自身）
        """
        return set(self._get_all_child_links_ordered(root_link_name))

    def _get_all_child_links_ordered(self, root_link_name: str) -> List[str]:
        """
        获取某个连杆下的所有子连杆（包括自身），按 BFS 顺序返回
        """
        child_links = [root_link_name]
        seen = {root_link_name}

        queue = deque([root_link_name])

        while queue:
            current_link = queue.popleft()

            try:
                current_frame_id = self.model.getFrameId(current_link)
            except Exception:
                continue

            for frame in self.model.frames:
                if frame.parentFrame == current_frame_id and frame.name != current_link:
                    if frame.name not in seen:
                        seen.add(frame.name)
                        child_links.append(frame.name)
                        queue.append(frame.name)

        return child_links

    def _get_frame_id(self, link_name: str) -> Optional[int]:
        """获取连杆对应的 frame ID，不存在则返回 None"""
        try:
            return self.model.getFrameId(link_name)
        except Exception:
            return None

    def _get_path_to_root(self, frame_id: int) -> List[int]:
        """返回从指定 frame 到根的路径（包含自身）"""
        path = []
        visited = set()
        current = frame_id
        while current is not None and current not in visited:
            visited.add(current)
            path.append(current)
            parent = self.model.frames[current].parentFrame
            if parent == current:
                break
            current = parent
        return path

    def _get_link_names_between_links(self, link_name_a: str,
                                      link_name_b: str) -> Set[str]:
        """获取两连杆之间路径上的所有连杆名称（包含两端）"""
        frame_a = self._get_frame_id(link_name_a)
        frame_b = self._get_frame_id(link_name_b)
        if frame_a is None or frame_b is None:
            return set()

        path_a = self._get_path_to_root(frame_a)
        path_b = self._get_path_to_root(frame_b)

        path_a_set = set(path_a)
        lca = None
        for frame_id in path_b:
            if frame_id in path_a_set:
                lca = frame_id
                break
        if lca is None:
            return set()

        up_a = path_a[:path_a.index(lca) + 1]
        up_b = path_b[:path_b.index(lca)]
        path_between = up_a + up_b

        return {self.model.frames[frame_id].name for frame_id in path_between}

    def _get_link_names_between_links_ordered(self, link_name_a: str,
                                              link_name_b: str) -> List[str]:
        """获取两连杆之间路径上的连杆名称（包含两端），按 A->B 顺序返回"""
        frame_a = self._get_frame_id(link_name_a)
        frame_b = self._get_frame_id(link_name_b)
        if frame_a is None or frame_b is None:
            return []

        path_a = self._get_path_to_root(frame_a)
        path_b = self._get_path_to_root(frame_b)

        path_a_set = set(path_a)
        lca = None
        for frame_id in path_b:
            if frame_id in path_a_set:
                lca = frame_id
                break
        if lca is None:
            return []

        up_a = path_a[:path_a.index(lca) + 1]
        down_b = list(reversed(path_b[:path_b.index(lca)]))
        path_between = up_a + down_b

        return [self.model.frames[frame_id].name for frame_id in path_between]

    def _get_mesh_paths_for_links(self,
                                  link_names: Iterable[str],
                                  include_visual: bool = True,
                                  include_collision: bool = True) -> List[str]:
        """
        获取一组连杆的所有 mesh 文件路径
        
        Args:
            link_names: 连杆名称集合
            include_visual: 是否包含视觉模型
            include_collision: 是否包含碰撞模型
            
        Returns:
            mesh 文件路径列表
        """
        mesh_paths = []

        for link_name in link_names:
            if include_visual and link_name in self._link_to_visual_mesh_paths:
                mesh_paths.extend(self._link_to_visual_mesh_paths[link_name])
            if include_collision and link_name in self._link_to_collision_mesh_paths:
                mesh_paths.extend(self._link_to_collision_mesh_paths[link_name])

        return mesh_paths

    # ========== 主要公共接口 ==========

    def _get_mesh_paths_by_joint(self,
                                 joint_name: str,
                                 include_self: bool = True,
                                 include_visual: bool = True,
                                 include_collision: bool = True,
                                 unique: bool = True) -> List[str]:
        """
        根据关节名称，获取该关节对应连杆自身的 mesh 文件路径列表
        
        Args:
            joint_name: 关节名称
            include_self: 是否包含关节自身的连杆（False 时返回空列表）
            include_visual: 是否包含视觉模型
            include_collision: 是否包含碰撞模型
            unique: 是否去重
            
        Returns:
            mesh 文件路径列表（List[str]）
            
        Example:
            >>> paths = extractor.get_mesh_paths_by_joints(["shoulder_joint"])[0]
            >>> print(paths)
            ['/path/to/shoulder.stl', '/path/to/upper_arm.obj', '/path/to/forearm.stl']
        """
        # 获取关节对应的连杆
        link_name = self._get_link_name_from_joint(joint_name)
        if link_name is None:
            if self.verbose:
                print(f"警告: 无法找到关节 '{joint_name}' 对应的连杆")
            return []

        if include_self:
            child_links = [link_name]
        else:
            child_links = []

        # 获取 mesh 路径
        mesh_paths = self._get_mesh_paths_for_links(child_links, include_visual,
                                                    include_collision)

        if unique:
            mesh_paths = list(dict.fromkeys(mesh_paths))

        return mesh_paths

    def get_mesh_paths_by_joints(self,
                                 joint_names: List[str],
                                 include_self: bool = True,
                                 include_visual: bool = True,
                                 include_collision: bool = True,
                                 unique: bool = True) -> List[List[str]]:
        """
        根据多个关节名称，按输入顺序返回每个关节对应的 mesh 文件路径列表

        Args:
            joint_names: 关节名称列表
            include_self: 是否包含关节自身的连杆
            include_visual: 是否包含视觉模型
            include_collision: 是否包含碰撞模型
            unique: 是否去重

        Returns:
            二维列表，外层顺序与 joint_names 一致
        """
        return [
            self._get_mesh_paths_by_joint(
                joint_name,
                include_self=include_self,
                include_visual=include_visual,
                include_collision=include_collision,
                unique=unique,
            ) for joint_name in joint_names
        ]

    def get_mesh_paths_by_link(self,
                               link_name: str,
                               other_link_name: Optional[str] = None,
                               include_self: bool = True,
                               include_visual: bool = True,
                               include_collision: bool = True,
                               unique: bool = True) -> List[str]:
        """
        根据连杆名称，获取该连杆及其所有子连杆的 mesh 文件路径列表
        或者输入两个连杆，获取两者之间路径上的 mesh 文件路径列表
        返回顺序稳定：子树按 BFS 顺序，路径按 link_name -> other_link_name 顺序
        
        Args:
            link_name: 连杆名称
            other_link_name: 另一个连杆名称（可选）
            include_self: 是否包含当前连杆自身
            include_visual: 是否包含视觉模型
            include_collision: 是否包含碰撞模型
            unique: 是否去重
            
        Returns:
            mesh 文件路径列表（List[str]）
            
        Example:
            >>> paths = extractor.get_mesh_paths_by_link("base_link")
            >>> print(paths)
            ['/path/to/base.stl', '/path/to/base_collision.obj']
        """
        # 检查连杆是否存在
        if self._get_frame_id(link_name) is None:
            if self.verbose:
                print(f"警告: 找不到连杆 '{link_name}'")
            return []

        if other_link_name:
            child_links = self._get_link_names_between_links_ordered(
                link_name, other_link_name)
            if not child_links and self.verbose:
                print(f"警告: 无法找到连杆 '{link_name}' 与 '{other_link_name}' 之间的路径")
        else:
            # 获取所有子连杆
            if include_self:
                child_links = self._get_all_child_links_ordered(link_name)
            else:
                all_children = self._get_all_child_links_ordered(link_name)
                child_links = [
                    name for name in all_children if name != link_name
                ]

        # 获取 mesh 路径
        mesh_paths = self._get_mesh_paths_for_links(child_links, include_visual,
                                                    include_collision)

        if unique:
            mesh_paths = list(dict.fromkeys(mesh_paths))

        return mesh_paths

    def _get_all_joints_mesh_paths(
            self,
            include_visual: bool = True,
            include_collision: bool = True) -> Dict[str, List[str]]:
        """
        获取所有关节及其控制的 mesh 文件路径映射
        
        Args:
            include_visual: 是否包含视觉模型
            include_collision: 是否包含碰撞模型
            
        Returns:
            字典，键为关节名称，值为该关节控制的 mesh 文件路径列表
            
        Example:
            >>> all_paths = extractor._get_all_joints_mesh_paths()
            >>> for joint, paths in all_paths.items():
            ...     print(f"{joint}: {len(paths)} files")
        """
        joint_mesh_paths = {}

        # 遍历所有关节（跳过 universe 关节）
        for joint_id in range(1, self.model.njoints):
            joint_name = self.model.names[joint_id]
            mesh_paths = self._get_mesh_paths_by_joint(
                joint_name,
                include_visual=include_visual,
                include_collision=include_collision)
            if mesh_paths:
                joint_mesh_paths[joint_name] = mesh_paths

        return joint_mesh_paths

    # ========== 辅助方法 ==========

    def _get_visual_mesh_paths(self) -> List[str]:
        """
        获取所有视觉模型的 mesh 文件路径列表
        
        Returns:
            所有视觉 mesh 文件路径列表
        """
        if self._visual_mesh_paths is None:
            self._visual_mesh_paths = []
            for geom in self.visual_model.geometryObjects:
                path = self._get_mesh_file_path(geom)
                if path:
                    self._visual_mesh_paths.append(path)
        return self._visual_mesh_paths.copy()

    def _get_collision_mesh_paths(self) -> List[str]:
        """
        获取所有碰撞模型的 mesh 文件路径列表
        
        Returns:
            所有碰撞 mesh 文件路径列表
        """
        if self._collision_mesh_paths is None:
            self._collision_mesh_paths = []
            for geom in self.collision_model.geometryObjects:
                path = self._get_mesh_file_path(geom)
                if path:
                    self._collision_mesh_paths.append(path)
        return self._collision_mesh_paths.copy()

    def _get_all_mesh_paths(self, unique: bool = True) -> List[str]:
        """
        获取所有 mesh 文件路径（视觉+碰撞）
        
        Args:
            unique: 是否去重
            
        Returns:
            所有 mesh 文件路径列表
        """
        all_paths = self._get_visual_mesh_paths(
        ) + self._get_collision_mesh_paths()
        if unique:
            return list(dict.fromkeys(all_paths))
        return all_paths

    def _get_mesh_paths_for_single_link(
            self,
            link_name: str,
            include_visual: bool = True,
            include_collision: bool = True) -> Dict[str, List[str]]:
        """
        获取单个连杆的 mesh 文件路径（仅自身，不包括子连杆）
        
        Args:
            link_name: 连杆名称
            include_visual: 是否包含视觉模型
            include_collision: 是否包含碰撞模型
            
        Returns:
            包含视觉和碰撞 mesh 路径的字典
            
        Example:
            >>> paths = extractor._get_mesh_paths_for_single_link("base_link")
            >>> print(paths['visual'])
            >>> print(paths['collision'])
        """
        result = {"visual": [], "collision": []}

        if include_visual and link_name in self._link_to_visual_mesh_paths:
            result["visual"] = self._link_to_visual_mesh_paths[link_name].copy()

        if include_collision and link_name in self._link_to_collision_mesh_paths:
            result["collision"] = self._link_to_collision_mesh_paths[
                link_name].copy()

        return result


# # ========== 使用示例 ==========
# if __name__ == "__main__":
#     extractor = PinocchioMeshExtractor(
#         urdf_path=Path.home() / ".cache/dexe_data/assembled/w1/robot_with_ee.urdf",
#         package_dirs=[Path.home() / ".cache/dexe_data/assembled/w1/"],
#         verbose=True)

#     collision_between = extractor.get_mesh_paths_by_link("left_j7", include_visual=False, include_collision=True)
#     print(collision_between)

#     joint_name = ["LEFT_J1", "LEFT_J2", "LEFT_J3", "LEFT_J4", "LEFT_J5"]
#     collision_between = extractor.get_mesh_paths_by_joints(joint_name, include_visual=False, include_collision=True)
#     print(joint_name)
#     print(collision_between)

# ========== 使用示例 ==========
if __name__ == "__main__":
    extractor = PinocchioMeshExtractor(
        urdf_path=Path.home() / ".cache/dexe_data/assembled/w1/robot.urdf",
        package_dirs=[Path.home() / ".cache/dexe_data/assembled/w1/"],
        verbose=True)

    collision_between = extractor.get_mesh_paths_by_link("left_j7",
                                                         include_visual=False,
                                                         include_collision=True)
    print(collision_between)

    joint_name = ["LEFT_J1", "LEFT_J2", "LEFT_J3", "LEFT_J4", "LEFT_J5"]
    collision_between = extractor.get_mesh_paths_by_joints(
        joint_name, include_visual=False, include_collision=True)
    print(joint_name)
    print(collision_between)
    from IPython import embed
    embed(header="Mesh Extractor Demo")  # noqa: F401 --- IGNORE ---
