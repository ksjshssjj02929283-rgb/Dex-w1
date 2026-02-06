# 切换到脚本所在目录
cd "$(dirname "$0")"

# 解析参数
A1_MODE=false
if [[ "$1" == "--a1" ]] || [[ "$1" == "a1" ]]; then
    A1_MODE=true
fi

SUDO_PASS=123456
echo "$SUDO_PASS" | sudo -S setcap 'cap_sys_nice=eip cap_ipc_lock=ep' ../install/w1_server/lib/w1_server/w1_server

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
kill_process "w1_server"

if [ "$A1_MODE" = true ]; then
    echo "📄 使用配置: robot.cfg (与 W1 共用)"
    taskset -c 6 ../install/w1_server/lib/w1_server/w1_server --a1
else
    taskset -c 5 ros2 run sim_server sim_server > /dev/null 2>&1 &
    taskset -c 6 script/set_max_position_following_error_window.sh > /dev/null 2>&1 &
    taskset -c 6 ros2 run w1_server w1_server
fi