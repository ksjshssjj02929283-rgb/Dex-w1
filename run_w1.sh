# 切换到脚本所在目录
cd "$(dirname "$0")"

source ../install/setup.bash

# Function to kill process by name
kill_process() {
    local name=$1
    pids=$(pgrep -f "$name")
    if [ -n "$pids" ]; then
        kill -9 $pids
    fi
}

# 如果已经运行了则kill掉
kill_process "visual_ros2_node.py"
kill_process "sim_server"
kill_process "w1_server"

# 启动sim_server和仿真可视化
# /bin/python3 sim_server/python/visual_ros2_node.py &
ros2 run sim_server sim_server > /dev/null 2>&1 &
ros2 run w1_server w1_server