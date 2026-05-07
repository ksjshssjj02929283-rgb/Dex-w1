#!/bin/bash

# 设置 ROS 环境变量
source /opt/ros/humble/setup.bash

# 仓根无 package.xml,colcon 默认从 . 递归扫,识别
# dexe_interfaces_meta/(metapackage,name=dexe_interfaces)+ 11 个接口子包。
echo "Running colcon build..."
colcon build
