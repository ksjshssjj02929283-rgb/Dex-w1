#!/bin/bash
set -e
# 切换到包含所有包的根目录
WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"
INSTALL_DIR="$(cd "${WORKSPACE_DIR}/../install" && pwd 2>/dev/null || echo "${WORKSPACE_DIR}/../install")"
source /opt/ros/humble/setup.bash
apt update && apt install -y liburdfdom-dev

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
    git -C "${WORKSPACE_DIR}" submodule sync --recursive
    git -C "${WORKSPACE_DIR}" submodule update --init --recursive --remote
fi
cd "${WORKSPACE_DIR}/.."

# 记录的源码路径冲突;未装则正常编 submodule 版本(此时无旧缓存,不会冲突)。
SKIP_INTERFACES=""
for d in "$WORKSPACE_DIR/dexe_interfaces"/*/; do
  [ -d "$d" ] || continue
  pkg=$(basename "$d")
  if [ -d "install/$pkg" ]; then
    SKIP_INTERFACES="$SKIP_INTERFACES $pkg"
    echo "    - 跳过 $pkg (已在 install/ 中存在)"
  else
    echo "    - 编译 $pkg (install/ 中不存在,从 submodule 编译)"
  fi
done
SKIP_ARGS=""
if [ -n "$SKIP_INTERFACES" ]; then
  SKIP_ARGS="--packages-skip ${SKIP_INTERFACES# }"
fi

colcon build --parallel-workers "${COLCON_PARALLEL_WORKERS}" --cmake-args -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DENABLE_TORQUE_CONTROL_SIMULATION=OFF -DCMAKE_CXX_COMPILER_LAUNCHER= -DCMAKE_C_COMPILER_LAUNCHER= --base-paths dexe_robot_server --packages-up-to robot_server_lib sim_server w1_server $SKIP_ARGS

# === 2. 检查 Sudo 权限 ===
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

# 配置 ldconfig（setcap 后二进制会忽略 LD_LIBRARY_PATH）
cat > /tmp/w1_server.conf << LDCONF
/opt/ros/humble/lib
/opt/ros/humble/lib/aarch64-linux-gnu
/opt/ros/humble/lib/x86_64-linux-gnu
$INSTALL_DIR/chassis_interfaces/lib
$INSTALL_DIR/joint_interfaces/lib
$INSTALL_DIR/robot_server_interfaces/lib
$INSTALL_DIR/robot_server_lib/lib
LDCONF
eval "$SUDO_PREFIX cp /tmp/w1_server.conf /etc/ld.so.conf.d/w1_server.conf"
eval "$SUDO_PREFIX ldconfig"
