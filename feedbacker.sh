#!/bin/bash
#   ./feedbacker.sh              # 默认启动 W1 模式 (20关节)
#   ./feedbacker.sh --a1         # 启动 A1 模式 (16关节)
#   ./feedbacker.sh --single     # 启动 Single Arm 模式 (7关节)
#   ./feedbacker.sh --dual       # 启动 Dual Arm 模式 (14关节)

# 切换到脚本所在目录
cd "$(dirname "$0")"

# 解析参数

MODE=""
FEEDBACK_ARGS=""

case "$1" in
    --a1|a1)
        MODE="a1"
        FEEDBACK_ARGS="--a1"
        shift
        ;;
    --single|single)
        MODE="single"
        FEEDBACK_ARGS="--single"
        shift
        ;;
    --dual|dual)
        MODE="dual"
        FEEDBACK_ARGS="--dual"
        shift
        ;;
    --arm|arm)
        MODE="single"
        FEEDBACK_ARGS="--single"
        shift
        ;;
    *)
        MODE="w1"
        ;;
esac

echo "Starting feedback in $MODE mode..."

# 剩余参数传递给 feedback
if [ $# -gt 0 ]; then
    FEEDBACK_ARGS="$FEEDBACK_ARGS $@"
fi

# Function to kill process by exact name
kill_process_exact() {
    local name=$1
    pids=$(pgrep -x "$name")
    if [ -n "$pids" ]; then
        kill -9 $pids
    fi
}

# 如果已经运行了则kill掉
kill_process_exact "feedback"

# 启动 feedback（默认不记录日志，使用 --record 开启日志记录）
../install/w1_server/lib/w1_server/feedback $FEEDBACK_ARGS
