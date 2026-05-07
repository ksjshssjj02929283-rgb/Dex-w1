#!/bin/bash

# 设置 ROS 环境变量
source /opt/ros/humble/setup.bash

# 打印并执行 colcon build 命令进行构建
echo "Running colcon build..."
# 仓根 package.xml 是供 CI 读版本号用的元数据,不参与构建。
# colcon 默认会在根目录识别到 package.xml 后停止下钻,从而屏蔽全部子包,
# 因此用 --base-paths 显式列出含子包的子目录(动态扫描)。
SUBPATHS=$(find . -mindepth 2 -name package.xml -printf '%h\n' | awk -F/ '{print $2}' | sort -u | tr '\n' ' ')
echo "📂 子包搜索路径: ${SUBPATHS}"
colcon build --base-paths ${SUBPATHS}
