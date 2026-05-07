#!/bin/bash

# 设置 ROS 环境变量
source /opt/ros/humble/setup.bash

# 仓根 dexe_interfaces 是 ROS2 metapackage(package.xml 用 <metapackage/>
# 标记 + 简单 ament_cmake/ament_package() CMakeLists.txt),colcon 从根递归
# 扫到 metapackage + 12 个接口子包,按依赖顺序 build。
echo "Running colcon build..."
colcon build
