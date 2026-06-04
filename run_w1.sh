#!/bin/bash

cd "$(dirname "$0")"
WORKSPACE_DIR=$(pwd)
INSTALL_DIR="$(cd "${WORKSPACE_DIR}/../install" && pwd 2>/dev/null || echo "${WORKSPACE_DIR}/../install")"

source ../install/setup.bash

# Function to kill process by name
kill_process() {
    local name=$1
    pids=$(pgrep -x "$name")
    if [ -n "$pids" ]; then
        kill -9 $pids
    fi
}

# === 检查 Sudo 权限 ===
# 尝试使用默认密码验证，如果失败则提示用户手动输入
if echo "123456" | sudo -S -v >/dev/null 2>&1; then
    SUDO_PREFIX="echo 123456 | sudo -S"
else
    sudo -v
    SUDO_PREFIX="sudo"
fi

# 在 install 目录中对二进制提权
sudo_setcap() {
  eval "$SUDO_PREFIX setcap \"$1\" \"$2\" 2>/dev/null"
  eval "$SUDO_PREFIX getcap -v \"$2\" 2>/dev/null"
}
if [ -f "$INSTALL_DIR/w1_server/lib/w1_server/w1_server" ]; then
  sudo_setcap 'cap_sys_nice,cap_ipc_lock=+ep' "$INSTALL_DIR/w1_server/lib/w1_server/w1_server" || true
  echo "setcap applied to $INSTALL_DIR/w1_server/lib/w1_server/w1_server:"; getcap -v "$INSTALL_DIR/w1_server/lib/w1_server/w1_server" || true
fi
if [ -f "$INSTALL_DIR/w1_server/lib/w1_server/feedback" ]; then
  sudo_setcap 'cap_sys_nice,cap_ipc_lock=+ep' "$INSTALL_DIR/w1_server/lib/w1_server/feedback" || true
  echo "setcap applied to $INSTALL_DIR/w1_server/lib/w1_server/feedback:"; getcap -v "$INSTALL_DIR/w1_server/lib/w1_server/feedback" || true
fi

# 如果已经运行了则kill掉
kill_process "sim_server"
kill_process "w1_server"

taskset -c 7 ros2 run sim_server sim_server > /dev/null 2>&1 &
taskset -c 6 script/set_max_position_following_error_window.sh > /dev/null 2>&1 &
taskset -c 6 ros2 run w1_server w1_server 
