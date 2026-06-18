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

# 选择可执行文件路径：优先 install（开发），回退 /opt（部署）
BIN=""
for candidate in "$INSTALL_DIR/L1_server/lib/L1_server/feedback" "/opt/L1_server/L1_server/lib/L1_server/feedback"; do
    if [ -f "$candidate" ]; then
        BIN="$candidate"
        break
    fi
done

if [ -z "$BIN" ]; then
    echo "❌ 找不到 feedback 可执行文件"
    echo "   请先运行 ./build.sh，或检查 /opt/L1_server 是否已部署"
    exit 1
fi

echo "Starting feedback: $BIN"

# 启动 feedback（剩余参数透传，如 --record）
exec "$BIN" "$@"
