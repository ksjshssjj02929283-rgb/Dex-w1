# 切换到包含所有包的根目录
WORKSPACE_DIR=$(dirname "$0")
source /opt/ros/humble/setup.bash

SUDO_PASS=123456

# 构建类型：Release（默认）或 RelWithDebInfo（开发模式，编译更快）
BUILD_TYPE=${BUILD_TYPE:-Release}

# colcon 包级并发和单包编译并发默认保守一些，避免大文件编译时被系统直接杀掉
COLCON_PARALLEL_WORKERS=${COLCON_PARALLEL_WORKERS:-2}
CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-4}
export CMAKE_BUILD_PARALLEL_LEVEL

# === 1. 检查 ROS 环境 ===
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ 请先 source 对应ROS环境，例如："
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi

if [ -f "${WORKSPACE_DIR}/.gitmodules" ]; then
    git submodule sync --recursive
    git submodule update --init --recursive
fi
cd "${WORKSPACE_DIR}/.."
colcon build --parallel-workers "${COLCON_PARALLEL_WORKERS}" --cmake-args -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DENABLE_TORQUE_CONTROL_SIMULATION=OFF -DCMAKE_CXX_COMPILER_LAUNCHER= -DCMAKE_C_COMPILER_LAUNCHER= --base-paths dexe_robot_server --packages-up-to robot_server_lib w1_server sim_server

# 复制到 /opt 并配置运行时库路径
OPT_DIR=/opt/w1_server
echo "$SUDO_PASS" | sudo -S rm -rf $OPT_DIR
echo "$SUDO_PASS" | sudo -S cp -r install $OPT_DIR

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

{
    echo "/opt/ros/humble/lib"
    echo "/opt/ros/humble/lib/aarch64-linux-gnu"
    echo "/opt/ros/humble/lib/x86_64-linux-gnu"
    echo "$OPT_DIR/chassis_interfaces/lib"
    echo "$OPT_DIR/joint_interfaces/lib"
    echo "$OPT_DIR/robot_server_interfaces/lib"
    echo "$OPT_DIR/robot_server_lib/lib"
} | sudo tee /etc/ld.so.conf.d/w1_server.conf