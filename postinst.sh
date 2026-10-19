#!/bin/bash
set -e

# 安装时由 dpkg 执行，不能依赖打包时的变量或当前工作目录。
INSTALL_PATH="/home/dexforce/w1/install"
SCRIPTS_PATH="/home/dexforce/w1/dexe_robot_server"

if [ -d "$SCRIPTS_PATH" ]; then
    chown -R dexforce:dexforce "$SCRIPTS_PATH"
    chmod +x "$SCRIPTS_PATH"/*.sh
fi

# 设置实时任务权限 (Set capabilities for real-time tasks)
BINS=(
    "$INSTALL_PATH/w1_server/lib/w1_server/w1_server"
    "$INSTALL_PATH/w1_server/lib/w1_server/feedback"
)
for BIN in "${BINS[@]}"; do
    if [ -f "$BIN" ]; then
        if setcap 'cap_sys_nice,cap_ipc_lock=+ep' "$BIN"; then
            echo "Successfully set capabilities for $BIN"
        else
            echo "Failed to set capabilities for $BIN"
        fi
    fi
done

cat > /tmp/w1_server.conf << LDCONF
/opt/ros/humble/lib
/opt/ros/humble/lib/aarch64-linux-gnu
/opt/ros/humble/lib/x86_64-linux-gnu
$INSTALL_PATH/chassis_interfaces/lib
$INSTALL_PATH/joint_interfaces/lib
$INSTALL_PATH/robot_server_interfaces/lib
$INSTALL_PATH/robot_server_lib/lib
$INSTALL_PATH/nav_interfaces/lib
LDCONF
cp /tmp/w1_server.conf /etc/ld.so.conf.d/w1_server.conf
ldconfig

exit 0
