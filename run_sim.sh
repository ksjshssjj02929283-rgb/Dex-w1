SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

source $SCRIPT_DIR/../install/setup.bash

# 启动sim_server和仿真可视化
# /bin/python3 $SCRIPT_DIR/sim_server/python/pgc_gripper_sim_server.py &
/bin/python3 $SCRIPT_DIR/sim_server/python/brainco_hand_sim_server.py &
/bin/python3 $SCRIPT_DIR/sim_server/python/visual_ros2_node.py &
ros2 run sim_server sim_server