#!/bin/bash
# 切换到脚本所在目录
cd "$(dirname "$0")"

source ../install/setup.bash

# Function to kill process by exact name
kill_process_exact() {
    local name=$1
    pids=$(pgrep -x "$name")
    if [ -n "$pids" ]; then
        kill -9 $pids
    fi
}

# 如果已经运行了则kill掉
kill_process_exact "joint_control"

# 启动 joint_control (交互模式，直接运行以保持实时权限)
/opt/w1_server/w1_server/lib/w1_server/joint_control
