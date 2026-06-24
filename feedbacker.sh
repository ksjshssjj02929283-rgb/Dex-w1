#!/bin/bash
#   ./feedbacker.sh              # 默认启动
#   ./feedbacker.sh --record     # 启动并记录日志
#
# L1 的 feedback 为固定 30 自由度（18 上半身 + 12 腿），无需指定关节模式。

# 切换到脚本所在目录
cd "$(dirname "$0")"
WORKSPACE_DIR=$(pwd)
INSTALL_DIR="$(cd "${WORKSPACE_DIR}/../install" && pwd 2>/dev/null || echo "${WORKSPACE_DIR}/../install")"

# Function to kill process by exact name
kill_process_exact() {
    local name=$1
    pids=$(pgrep -x "$name")
    if [ -n "$pids" ]; then
        kill -9 $pids
    fi
}

# 如果已经运行了则 kill 掉
kill_process_exact "feedback"

# 选择可执行文件路径
BIN="$INSTALL_DIR/l1_server/lib/l1_server/feedback"

if [ ! -f "$BIN" ]; then
    echo "❌ 找不到 feedback 可执行文件: $BIN"
    echo "   请先运行 ./build.sh"
    exit 1
fi

echo "Starting feedback: $BIN"

# 启动 feedback（剩余参数透传，如 --record）
exec "$BIN" "$@"
