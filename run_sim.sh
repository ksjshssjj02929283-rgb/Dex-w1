source install/setup.bash

cd "$(dirname "$0")"
# 启动sim_server和仿真可视化
/bin/python3 sim_server/visualization/visual_ros2_node.py &
ros2 run sim_server sim_server