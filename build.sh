# 切换到包含所有包的根目录
WORKSPACE_DIR=$(dirname "$0")
cd "$WORKSPACE_DIR"/..
source /opt/ros/humble/setup.bash

rm -rf build/L1_server  # 清理L1_server的构建目录重新cmake以避免覆盖robot.cfg

PACKAGES=$(find dexe_interfaces dexe_robot_server -name "package.xml" -exec sh -c 'dirname {} | xargs basename' \; | sort -u | tr '\n' ' ')
colcon build --cmake-args --packages-select $PACKAGES

BIN=install/L1_server/lib/L1_server/L1_server
if [ -f "$BIN" ]; then
  sudo setcap 'cap_sys_nice,cap_ipc_lock=+ep' "$BIN" || true
  echo "setcap applied to $BIN:"; getcap -v "$BIN" || true
else
  echo "ERROR: $BIN 不存在，检查构建是否成功以及目标名是否为 L1_server"
fi

# 设置 feedback 权限（复制后设置）
if [ -f "install/L1_server/lib/L1_server/feedback" ]; then
  sudo setcap 'cap_sys_nice,cap_ipc_lock=+ep' "install/L1_server/lib/L1_server/feedback" || true
  echo "setcap applied to install/L1_server/lib/L1_server/feedback:"; getcap -v "install/L1_server/lib/L1_server/feedback" || true
else
  echo "ERROR: install/L1_server/lib/L1_server/feedback 不存在，检查构建是否成功以及目标名是否为 feedback"
fi
# 复制到 /opt 并配置运行时库路径
sudo rm -rf /opt/L1_server
sudo cp -r install /opt/L1_server

{
    echo "/opt/ros/humble/lib"
    echo "/opt/ros/humble/lib/aarch64-linux-gnu"
    echo "/opt/ros/humble/lib/x86_64-linux-gnu"
    echo "/opt/L1_server/chassis_interfaces/lib"
    echo "/opt/L1_server/joint_interfaces/lib"
    echo "/opt/L1_server/robot_server_interfaces/lib"
    echo "/opt/L1_server/robot_server_lib/lib"
} | sudo tee /etc/ld.so.conf.d/L1_server.conf
sudo ldconfig