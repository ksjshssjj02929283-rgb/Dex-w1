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

# 启动sim_server和仿真可视化
/bin/python3 script/visual_ros2_node.py &
ros2 run sim_server sim_server