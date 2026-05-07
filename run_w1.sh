#!/bin/bash

cd "$(dirname "$0")"

SUDO_PASS=123456
echo "$SUDO_PASS" | sudo -S setcap 'cap_sys_nice=eip cap_ipc_lock=ep' /opt/w1_server/w1_server/lib/w1_server/w1_server

source /opt/w1_server/setup.bash

# Function to kill process by name
kill_process() {
    local name=$1
    pids=$(pgrep -x "$name")
    if [ -n "$pids" ]; then
        kill -9 $pids
    fi
}

# 如果已经运行了则kill掉
kill_process "sim_server"
kill_process "w1_server"

taskset -c 7 ros2 run sim_server sim_server > /dev/null 2>&1 &
taskset -c 6 script/set_max_position_following_error_window.sh > /dev/null 2>&1 &
taskset -c 6 ros2 run w1_server w1_server 
