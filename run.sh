# 切换到脚本所在目录
cd "$(dirname "$0")"

# 解析参数
A1_MODE=false
if [[ "$1" == "--a1" ]] || [[ "$1" == "a1" ]]; then
    A1_MODE=true
fi

SUDO_PASS=123456
echo "$SUDO_PASS" | sudo -S setcap 'cap_sys_nice=eip cap_ipc_lock=ep' ../install/L1_server/lib/L1_server/L1_server

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
kill_process "sim_server"
kill_process "L1_server"

taskset -c 5 ros2 run sim_server sim_server > /dev/null 2>&1 &
taskset -c 6 script/set_max_position_following_error_window.sh > /dev/null 2>&1 &
taskset -c 6 ros2 run L1_server L1_server
