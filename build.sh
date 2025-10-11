# 切换到包含所有包的根目录
cd "$(dirname "$0")/.."

PACKAGES=$(find dexe_interfaces dexe_robot_server -name "package.xml" -exec sh -c 'dirname {} | xargs basename' \; | sort -u | tr '\n' ' ')
colcon build --cmake-args -DENABLE_TORQUE_CONTROL_SIMULATION=ON --packages-select $PACKAGES

BIN=install/w1_server/lib/w1_server/w1_server
if [ -f "$BIN" ]; then
  sudo setcap 'cap_sys_nice,cap_ipc_lock=+ep' "$BIN" || true
  echo "setcap applied to $BIN:"; getcap -v "$BIN" || true
else
  echo "ERROR: $BIN 不存在，检查构建是否成功以及目标名是否为 w1_server"
fi

# 复制到 /opt 并配置运行时库路径
sudo rm -rf /opt/w1_server
sudo cp -r install /opt/w1_server

{
    echo "/opt/ros/humble/lib"
    echo "/opt/ros/humble/lib/aarch64-linux-gnu"
    echo "/opt/ros/humble/lib/x86_64-linux-gnu"
    echo "/opt/w1_server/chassis_interfaces/lib"
    echo "/opt/w1_server/joint_interfaces/lib"
    echo "/opt/w1_server/robot_server_interfaces/lib"
    echo "/opt/w1_server/robot_server_lib/lib"
} | sudo tee /etc/ld.so.conf.d/w1_server.conf
sudo ldconfig
##################################################################