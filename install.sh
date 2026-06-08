#!/bin/bash

CURDIR=$(pwd)
PROJECT_DIR=$(basename "$PWD")
PACKAGE_NAME=${PROJECT_DIR//_/-}
echo "Running file installation for $PACKAGE_NAME..."

if [ ! -d "$CURDIR/install" ]; then
    echo "Error: install directory not found!"
    exit 1
fi

# PC1 用 /home/dexforce/w1、PC2 用 /home/dexforce/workspace,且要求「只能有其中一个」,
# 所以不能把文件直接烤进某个 home 路径(那样 dpkg 会在另一台机器上凭空建出另一个目录)。
# 这里把整个 colcon install/ 暂存到包内中立路径 /opt/$PACKAGE_NAME/install,
# 由 postinst.sh 在安装时检测「实际已存在」的根目录再铺过去。
STAGE="$CURDIR/debian/$PACKAGE_NAME/opt/$PACKAGE_NAME/install"
mkdir -p "$STAGE"
cp -a "$CURDIR/install/." "$STAGE/"

# --- DEBIAN/postinst:复用仓库根已提交的 postinst.sh(单一来源)---
# 本地手动 dpkg-deb 打包时需要这一份;CI(gen_deb.sh)则会直接拿仓库根的 postinst.sh,
# 两条路径用的是同一段逻辑,行为一致。
DEBIAN_DIR="$CURDIR/debian/$PACKAGE_NAME/DEBIAN"
mkdir -p "$DEBIAN_DIR"
if [ -f "$CURDIR/postinst.sh" ]; then
    cp "$CURDIR/postinst.sh" "$DEBIAN_DIR/postinst"
    chmod 0755 "$DEBIAN_DIR/postinst"
else
    echo "Warning: postinst.sh not found, deb will have no postinst!" >&2
fi

# --- DEBIAN/control:固定版本号 ---
# 本地手动打包用;CI 会用 dh_make + dch 重新生成(并追加 +timestamp+githash)。
cat > "$DEBIAN_DIR/control" <<EOF
Package: $PACKAGE_NAME
Version: 0.4.0
Architecture: arm64
Maintainer: Dexforce <dexforce@dexforce.com>
Description: $PACKAGE_NAME
EOF
