# 切换到包含所有包的根目录
WORKSPACE_DIR=$(dirname "$0")
source /opt/ros/humble/setup.bash

# 构建类型：Release（默认）或 RelWithDebInfo（开发模式，编译更快）
BUILD_TYPE=${BUILD_TYPE:-Release}

# === 1. 检查 ROS 环境 ===
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ 请先 source 对应ROS环境，例如："
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi
echo "✅ ROS版本: $ROS_DISTRO"

# 输出环境中已有的ROS包名字, 检查是否有robot_server_interfaces包
if ros2 pkg list | grep -q "robot_server_interfaces"; then
    echo "✅ 找到 robot_server_interfaces 包, 跳过dexe_interface包编译。"
else
    echo "⚠️ 未找到 robot_server_interfaces 包, 需要重新编译 dexe_interface 包。"
    if [ -f "${WORKSPACE_DIR}/.gitmodules" ]; then
        echo "🔄 检测到 .gitmodules，正在同步子模块..."
        git submodule sync --recursive
        git submodule update --init --recursive
        echo "✅ 子模块已更新完成。"
    else
        echo "ℹ️ 未检测到 .gitmodules，跳过子模块更新。"
    fi
fi

cd "${WORKSPACE_DIR}/.."
PACKAGES=$(find dexe_robot_server -name "package.xml" -exec sh -c 'dirname {} | xargs basename' \; | sort -u | tr '\n' ' ')
colcon build --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DENABLE_TORQUE_CONTROL_SIMULATION=OFF --packages-select $PACKAGES

# 检查 w1_server 二进制文件并设置权限
BIN=install/w1_server/lib/w1_server/w1_server
if [ -f "$BIN" ]; then
  sudo setcap 'cap_sys_nice,cap_ipc_lock=+ep' "$BIN" || true
  echo "setcap applied to $BIN:"; getcap -v "$BIN" || true
else
  echo "ERROR: $BIN 不存在，检查构建是否成功"
fi

# 复制到 /opt 并配置运行时库路径
OPT_DIR=/opt/w1_server
sudo rm -rf $OPT_DIR
sudo cp -r install $OPT_DIR

{
    echo "/opt/ros/humble/lib"
    echo "/opt/ros/humble/lib/aarch64-linux-gnu"
    echo "/opt/ros/humble/lib/x86_64-linux-gnu"
    echo "$OPT_DIR/chassis_interfaces/lib"
    echo "$OPT_DIR/joint_interfaces/lib"
    echo "$OPT_DIR/robot_server_interfaces/lib"
    echo "$OPT_DIR/robot_server_lib/lib"
} | sudo tee /etc/ld.so.conf.d/w1_server.conf
sudo ldconfig