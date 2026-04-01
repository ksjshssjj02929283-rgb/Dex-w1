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

/opt/L1_server/L1_server/lib/L1_server/feedback
