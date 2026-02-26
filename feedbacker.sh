#   ./feedbacker.sh          # 默认启动 W1 模式 (20关节)
#   ./feedbacker.sh --a1     # 启动 A1 模式 (14关节)
#   ./feedbacker.sh --arm    # 启动 Arm 模式 (仅机械臂, 7关节)

# 切换到脚本所在目录
cd "$(dirname "$0")"

# 解析参数

A1_MODE=false
ARM_MODE=false
FEEDBACK_ARGS=""

if [[ "$1" == "--a1" ]] || [[ "$1" == "a1" ]]; then
    A1_MODE=true
    FEEDBACK_ARGS="--a1"
    shift
elif [[ "$1" == "--arm" ]] || [[ "$1" == "arm" ]]; then
    ARM_MODE=true
    FEEDBACK_ARGS="--arm"
    shift
fi

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
/opt/w1_server/w1_server/lib/w1_server/feedback $FEEDBACK_ARGS
