#!/bin/bash
set -e

# 切换到包含所有包的根目录（脚本所在目录的上一级为 colcon workspace 根）
WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"
INSTALL_DIR="$(cd "${WORKSPACE_DIR}/../install" && pwd 2>/dev/null || echo "${WORKSPACE_DIR}/../install")"
source /opt/ros/humble/setup.bash

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

# === 2. 初始化 submodule（dexe_interfaces 使用 zyn/l1 分支，含 JointCommand.msg 等 L1 接口定义）===
if [ -f "${WORKSPACE_DIR}/.gitmodules" ]; then
    git -C "${WORKSPACE_DIR}" submodule sync --recursive
    git -C "${WORKSPACE_DIR}" submodule update --init --recursive
    # 切换 dexe_interfaces 到 zyn/l1 分支（L1 双足人形接口）
    git -C "${WORKSPACE_DIR}/dexe_interfaces" fetch origin
    git -C "${WORKSPACE_DIR}/dexe_interfaces" checkout origin/zyn/l1
fi
cd "${WORKSPACE_DIR}/.."

# === 2.5 检测 submodule 是否变更，若变更则清除旧接口产物（避免 install/ 中残留旧分支编译结果）===
SUBMODULE_HASH=$(git -C "${WORKSPACE_DIR}/dexe_interfaces" rev-parse HEAD 2>/dev/null || echo "")
HASH_FILE="${WORKSPACE_DIR}/.dexe_interfaces_hash"
if [ -n "$SUBMODULE_HASH" ]; then
    if [ -f "$HASH_FILE" ] && [ "$(cat "$HASH_FILE")" = "$SUBMODULE_HASH" ]; then
        echo "    - dexe_interfaces 未变化 ($SUBMODULE_HASH)，复用 install/ 中的接口包"
    else
        echo "    - dexe_interfaces 已更新 ($SUBMODULE_HASH)，清除旧接口产物以强制重编译"
        for d in "$WORKSPACE_DIR/dexe_interfaces"/*/; do
            [ -d "$d" ] || continue
            pkg=$(basename "$d")
            rm -rf "install/$pkg" "build/$pkg"
        done
        echo "$SUBMODULE_HASH" > "$HASH_FILE"
    fi
fi

# === 3. 复用已安装的 interfaces 包，避免与 build/ 缓存里的源码路径冲突 ===
# 已在 install/ 就跳过——复用独立 ~/zyn/dexe_interfaces 的产物；未装则正常编 submodule 版本
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

# === 4. 编译（--packages-up-to 自动按依赖顺序编译 L1_server 及其依赖）===
colcon build --parallel-workers "${COLCON_PARALLEL_WORKERS}" \
  --cmake-args -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  --base-paths dexe_robot_server \
  --packages-up-to L1_server \
  $SKIP_ARGS

# === 5. 检查 Sudo 权限 ===
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

BIN_SERVER="$INSTALL_DIR/L1_server/lib/L1_server/L1_server"
BIN_FEEDBACK="$INSTALL_DIR/L1_server/lib/L1_server/feedback"

if [ -f "$BIN_SERVER" ]; then
  sudo_setcap 'cap_sys_nice,cap_ipc_lock=+ep' "$BIN_SERVER" || true
  echo "setcap applied to $BIN_SERVER"
else
  echo "ERROR: $BIN_SERVER 不存在，检查构建是否成功以及目标名是否为 L1_server"
fi

if [ -f "$BIN_FEEDBACK" ]; then
  sudo_setcap 'cap_sys_nice,cap_ipc_lock=+ep' "$BIN_FEEDBACK" || true
  echo "setcap applied to $BIN_FEEDBACK"
else
  echo "ERROR: $BIN_FEEDBACK 不存在，检查构建是否成功以及目标名是否为 feedback"
fi

# === 6. 复制到 /opt 并配置运行时库路径（setcap 后二进制会忽略 LD_LIBRARY_PATH）===
eval "$SUDO_PREFIX rm -rf /opt/L1_server"
eval "$SUDO_PREFIX cp -r install /opt/L1_server"

cat > /tmp/L1_server.conf << LDCONF
/opt/ros/humble/lib
/opt/ros/humble/lib/aarch64-linux-gnu
/opt/ros/humble/lib/x86_64-linux-gnu
/opt/L1_server/chassis_interfaces/lib
/opt/L1_server/joint_interfaces/lib
/opt/L1_server/robot_server_interfaces/lib
LDCONF
eval "$SUDO_PREFIX cp /tmp/L1_server.conf /etc/ld.so.conf.d/L1_server.conf"
eval "$SUDO_PREFIX ldconfig"
