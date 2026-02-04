# 切换到脚本所在目录
cd "$(dirname "$0")"

source ../install/setup.bash

# Function to kill process by pattern (full command line)
kill_process_pattern() {
    local name=$1
    pids=$(pgrep -f "$name")
    if [ -n "$pids" ]; then
        kill -9 $pids
    fi
}

# Function to kill process by exact name
kill_process_exact() {
    local name=$1
    pids=$(pgrep -x "$name")
    if [ -n "$pids" ]; then
        kill -9 $pids
    fi
}

# 如果已经运行了则kill掉
kill_process_pattern "visual_ros2_node.py"
kill_process_exact "sim_server"

# 启动sim_server和仿真可视化
/bin/python3 script/visual_ros2_node.py &
ros2 run sim_server sim_server
