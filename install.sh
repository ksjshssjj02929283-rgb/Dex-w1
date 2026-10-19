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

# --- 生成 postinst ---
# 直接 dpkg-deb --build 时使用 debian/<package>/DEBIAN/postinst。
DEB_ROOT="$CURDIR/debian/$PACKAGE_NAME"
DEBIAN_DIR="$DEB_ROOT/DEBIAN"
mkdir -p "$DEBIAN_DIR"

if [ -f "$CURDIR/postinst.sh" ]; then
    cp "$CURDIR/postinst.sh" "$DEBIAN_DIR/postinst"
    chmod 0755 "$DEBIAN_DIR/postinst"
else
    echo "未找到 postinst.sh，跳过 postinst 安装"
fi

if [ -f "$CURDIR/preinst.sh" ]; then
    cp "$CURDIR/preinst.sh" "$DEBIAN_DIR/preinst"
    chmod 0755 "$DEBIAN_DIR/preinst"
else
    echo "未找到 preinst.sh，跳过 preinst 安装"
fi

# --- 生成本地 dpkg-deb 所需的 control ---
if [ ! -f "$CURDIR/debian/control" ]; then
    cat > "$DEBIAN_DIR/control" <<EOF
Package: $PACKAGE_NAME
Version: 0.5.1
Architecture: arm64
Maintainer: Dexforce <dexforce@dexforce.com>
Description: $PACKAGE_NAME
EOF
    echo "已生成本地 dpkg-deb control: $DEBIAN_DIR/control"
else
    echo "CI 由.gitlab-ci.yml进行配置"
fi
