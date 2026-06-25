#!/bin/bash

cd "$(dirname "$0")"
WORKSPACE_DIR=$(pwd)
INSTALL_DIR="$(cd "${WORKSPACE_DIR}/../install" && pwd 2>/dev/null || echo "${WORKSPACE_DIR}/../install")"

source ../install/setup.bash

# Function to kill process by exact name
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
  eval "$SUDO_PREFIX setcap \"$1\" \"$2\"" 2>/dev/null
  eval "$SUDO_PREFIX getcap -v \"$2\"" 2>/dev/null
}
if [ -f "$INSTALL_DIR/l1_server/lib/l1_server/l1_server" ]; then
  sudo_setcap 'cap_sys_nice,cap_ipc_lock=+ep' "$INSTALL_DIR/l1_server/lib/l1_server/l1_server" || true
fi
if [ -f "$INSTALL_DIR/l1_server/lib/l1_server/feedback" ]; then
  sudo_setcap 'cap_sys_nice,cap_ipc_lock=+ep' "$INSTALL_DIR/l1_server/lib/l1_server/feedback" || true
fi

# 如果已经运行了则 kill 掉
kill_process "l1_server"

# 配置 EtherCAT 伺服的位置跟随误差窗口
# taskset -c 6 script/set_max_position_following_error_window.sh > /dev/null 2>&1 &

# 启动 l1_server
taskset -c 6 ./../install/l1_server/lib/l1_server/l1_server 
