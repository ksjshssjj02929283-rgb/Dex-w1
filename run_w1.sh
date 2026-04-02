#!/usr/bin/env bash

#   ./run_w1.sh          # 默认启动 W1 模式 (20关节)
#   ./run_w1.sh a1       # 启动 A1 模式 (14关节)
#   ./run_w1.sh --a1     # 同上
#   ./run_w1.sh arm      # 启动 Arm 模式 (仅机械臂, 7关节)
#   ./run_w1.sh --arm    # 同上

# 切换到脚本所在目录
cd "$(dirname "$0")"

# 解析参数
A1_MODE=false
ARM_MODE=false
if [[ "$1" == "--a1" ]] || [[ "$1" == "a1" ]]; then
    A1_MODE=true
elif [[ "$1" == "--arm" ]] || [[ "$1" == "arm" ]]; then
    ARM_MODE=true
fi

W1_BIN="../install/w1_server/lib/w1_server/w1_server"
CAP_MODE=${CAP_MODE:-workspace_safe}

if [ "$CAP_MODE" = "enable" ]; then
    echo "🔐 CAP_MODE=enable：将为 w1_server 设置 setcap（需要 sudo 密码）"
    sudo -v
    sudo setcap 'cap_sys_nice=eip cap_ipc_lock=ep' "$W1_BIN"
else
    # 对工作区 install 目录下的二进制，默认移除 capability，避免触发 secure-exec 后忽略 LD_LIBRARY_PATH。
    if getcap "$W1_BIN" 2>/dev/null | grep -q 'cap_'; then
        echo "🔐 检测到 $W1_BIN 已设置 capability；为避免动态库加载失败，将移除 capability（需要 sudo 密码）"
        sudo -v
        sudo setcap -r "$W1_BIN"
    fi
fi

source ../install/setup.bash

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

if [ "$A1_MODE" = true ]; then
    taskset -c 6 ../install/w1_server/lib/w1_server/w1_server --a1
elif [ "$ARM_MODE" = true ]; then
    echo "📄 使用配置: robot.cfg (Arm 模式)"
    taskset -c 6 ../install/w1_server/lib/w1_server/w1_server --arm
else
    taskset -c 6 ../install/w1_server/lib/w1_server/w1_server
fi
