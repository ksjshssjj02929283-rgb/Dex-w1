# 切换到脚本所在目录
cd "$(dirname "$0")"

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
/opt/w1_server/w1_server/lib/w1_server/feedback "$@"
