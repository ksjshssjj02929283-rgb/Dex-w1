SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

source $SCRIPT_DIR/../install/setup.bash

# Function to kill process by name
kill_process() {
    local name=$1
    pids=$(pgrep -f "$name")
    if [ -n "$pids" ]; then
        echo "Killing existing $name processes: $pids"
        kill -9 $pids
    fi
}

# 如果已经运行了则kill掉
kill_process "pgc_gripper_sim_server.py"
kill_process "brainco_hand_sim_server.py"
kill_process "visual_ros2_node.py"
kill_process "sim_server"

# 启动sim_server和仿真可视化
# /bin/python3 $SCRIPT_DIR/sim_server/python/pgc_gripper_sim_server.py &
/bin/python3 $SCRIPT_DIR/sim_server/python/brainco_hand_sim_server.py &
/bin/python3 $SCRIPT_DIR/sim_server/python/visual_ros2_node.py &
ros2 run sim_server sim_server