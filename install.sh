#!/bin/bash
set -e

CURDIR=$(pwd)
PROJECT_DIR=$(basename "$PWD")
PACKAGE_NAME=${PROJECT_DIR//_/-}
echo "Running file installation for $PACKAGE_NAME..."

DEBIAN_ROOT="$CURDIR/debian/$PACKAGE_NAME"
# 清理旧目录
rm -rf "$DEBIAN_ROOT"

INSTALL_PATH="/home/dexforce/w1/install"
SCRIPTS_PATH="/home/dexforce/w1/$PROJECT_DIR"
mkdir -p "$DEBIAN_ROOT$INSTALL_PATH"
mkdir -p "$DEBIAN_ROOT$SCRIPTS_PATH"

if [ ! -d "$CURDIR/../install" ]; then
    echo "Error: install directory not found!"
    exit 1
fi

# 保持 install 目录结构完整复制到目标路径
cp -r "$CURDIR/../install/"* "$DEBIAN_ROOT$INSTALL_PATH/"

# --- 剔除 dexe_interfaces 接口包: 机上接口由 dexe-interfaces deb 铺设, 本包不重复发布 ---
# 包名以 package.xml 的 <name> 为准, 目录名与包名可能不一致(dexe_interfaces_meta -> dexe_interfaces)。
# colcon 的 install/setup.bash 动态枚举包目录, 删子目录不影响 source。
if [ -d "$CURDIR/dexe_interfaces" ]; then
    for xml in "$CURDIR"/dexe_interfaces/*/package.xml; do
        [ -f "$xml" ] || continue
        pkg=$(sed -n 's|.*<name>\([^<]*\)</name>.*|\1|p' "$xml" | head -n1)
        [ -n "$pkg" ] || continue
        echo "Excluding interface package from deb: $pkg"
        rm -rf "$DEBIAN_ROOT$INSTALL_PATH/$pkg"
    done
fi

if [ -d "script" ]; then
    cp -r script "$DEBIAN_ROOT$SCRIPTS_PATH/"
fi
cp *.sh "$DEBIAN_ROOT$SCRIPTS_PATH/"

echo "Pruning non-runtime files for smaller package..."
# 删除 Python 缓存
find "$DEBIAN_ROOT" -type d -name '__pycache__' -prune -exec rm -rf {} +
find "$DEBIAN_ROOT" -type f -name '*.pyc' -delete

# 删除静态库/调试残留
find "$DEBIAN_ROOT$INSTALL_PATH" -type f \( -name '*.a' -o -name '*.la' -o -name '*.pdb' \) -delete

# 删除开发期头文件、CMake 包配置和 pkgconfig
find "$DEBIAN_ROOT$INSTALL_PATH" -type d -path '*/include' -prune -exec rm -rf {} +
find "$DEBIAN_ROOT$INSTALL_PATH" -type d -path '*/share/*/cmake' -prune -exec rm -rf {} +
find "$DEBIAN_ROOT$INSTALL_PATH" -type d \( -path '*/lib/pkgconfig' -o -path '*/share/pkgconfig' \) -prune -exec rm -rf {} +

DEBIAN_DIR="$DEBIAN_ROOT/DEBIAN"
mkdir -p "$DEBIAN_DIR"
cat > "$DEBIAN_DIR/postinst" <<EOF
#!/bin/bash
set -e

if [ -d "$SCRIPTS_PATH" ]; then
    chown -R dexforce:dexforce "$SCRIPTS_PATH"
    chmod +x "$SCRIPTS_PATH"/*.sh
fi

# 设置实时任务权限 (Set capabilities for real-time tasks)
BINS=(
    "$INSTALL_PATH/w1_server/lib/w1_server/w1_server"
    "$INSTALL_PATH/w1_server/lib/w1_server/feedback"
)
for BIN in "\${BINS[@]}"; do
    if [ -f "\$BIN" ]; then
        if setcap 'cap_sys_nice,cap_ipc_lock=+ep' "\$BIN"; then
            echo "Successfully set capabilities for \$BIN"
        else
            echo "Failed to set capabilities for \$BIN"
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
LDCONF
cp /tmp/w1_server.conf /etc/ld.so.conf.d/w1_server.conf
ldconfig

exit 0
EOF
chmod 0755 "$DEBIAN_DIR/postinst"

# --- 自动从 CMakeLists.txt 读取版本号 ---
VERSION=$(grep "set(W1_SERVER_VERSION" w1_server/CMakeLists.txt | sed -e 's/set(W1_SERVER_VERSION "\(.*\)")/\1/')
if [ -z "$VERSION" ]; then
    VERSION="0.5.0"
fi

# --- 自动生成 control ---
cat > "$DEBIAN_DIR/control" <<EOF
Package: $PACKAGE_NAME
Version: $VERSION
Architecture: arm64
Maintainer: Dexforce <dexforce@dexforce.com>
Description: $PACKAGE_NAME
EOF