# 切换到包含所有包的根目录
WORKSPACE_DIR=$(dirname "$0")
source /opt/ros/humble/setup.bash

# 构建类型：Release（默认）或 RelWithDebInfo（开发模式，编译更快）
BUILD_TYPE=${BUILD_TYPE:-Release}
# colcon 包级并发和单包编译并发默认保守一些，避免大文件编译时被系统直接杀掉
COLCON_PARALLEL_WORKERS=${COLCON_PARALLEL_WORKERS:-2}
CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-4}
export CMAKE_BUILD_PARALLEL_LEVEL
export MAKEFLAGS="-j${CMAKE_BUILD_PARALLEL_LEVEL} -l${CMAKE_BUILD_PARALLEL_LEVEL}"

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
echo "🔧 colcon并发包数: ${COLCON_PARALLEL_WORKERS}, 单包编译并发: ${CMAKE_BUILD_PARALLEL_LEVEL}"
colcon build --parallel-workers "${COLCON_PARALLEL_WORKERS}" --cmake-args -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DENABLE_TORQUE_CONTROL_SIMULATION=OFF --packages-select $PACKAGES

# 复制到 /opt 并配置运行时库路径
OPT_DIR=/opt/w1_server
sudo rm -rf $OPT_DIR
sudo cp -r install $OPT_DIR

# 设置 w1_server 权限（复制后设置）
if [ -f "$OPT_DIR/w1_server/lib/w1_server/w1_server" ]; then
  sudo setcap 'cap_sys_nice,cap_ipc_lock=+ep' "$OPT_DIR/w1_server/lib/w1_server/w1_server" || true
  echo "setcap applied to $OPT_DIR/w1_server/lib/w1_server/w1_server:"; getcap -v "$OPT_DIR/w1_server/lib/w1_server/w1_server" || true
fi

# 设置 feedback 权限（复制后设置）
if [ -f "$OPT_DIR/w1_server/lib/w1_server/feedback" ]; then
  sudo setcap 'cap_sys_nice,cap_ipc_lock=+ep' "$OPT_DIR/w1_server/lib/w1_server/feedback" || true
  echo "setcap applied to $OPT_DIR/w1_server/lib/w1_server/feedback:"; getcap -v "$OPT_DIR/w1_server/lib/w1_server/feedback" || true
fi

# 设置 joint_control 权限（复制后设置）
if [ -f "$OPT_DIR/w1_server/lib/w1_server/joint_control" ]; then
  sudo setcap 'cap_sys_nice,cap_ipc_lock=+ep' "$OPT_DIR/w1_server/lib/w1_server/joint_control" || true
  echo "setcap applied to $OPT_DIR/w1_server/lib/w1_server/joint_control:"; getcap -v "$OPT_DIR/w1_server/lib/w1_server/joint_control" || true
fi

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
