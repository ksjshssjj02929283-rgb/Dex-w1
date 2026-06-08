#!/bin/sh
# dexe_interfaces 安装后脚本(单一来源)。
#   - CI(gen_deb.sh)会直接把本文件作为 deb 的 DEBIAN/postinst;
#   - install.sh 本地打包时也复用本文件,二者逻辑一致。
# 若删掉本文件,CI 会退回到只处理 /home/dexforce/w1 的默认 postinst,既不会从 /opt
# 铺设,workspace 也不会被处理。
#
# install.sh 已把整个 colcon install/ 暂存进包内 /opt/dexe-interfaces/install。
# PC1 用 /home/dexforce/w1、PC2 用 /home/dexforce/workspace —— 这里只往目标机上
# 「实际已存在」的根目录铺,绝不凭空创建另一个(PC1 不会冒出 workspace,反之亦然)。
set -e

STAGE=/opt/dexe-interfaces/install
GROUP=dexforce

if ! getent group "$GROUP" >/dev/null 2>&1; then
    addgroup --system "$GROUP" || true
fi

if [ ! -d "$STAGE" ]; then
    echo "Warning: $STAGE not found, nothing to install" >&2
    exit 0
fi

FOUND=0
for base in /home/dexforce/w1 /home/dexforce/workspace; do
    if [ -d "$base" ]; then
        mkdir -p "$base/install"
        cp -a "$STAGE/." "$base/install/"
        chown -R "$GROUP":"$GROUP" "$base/install"
        chmod -R 775 "$base/install"
        echo "dexe-interfaces installed into $base/install"
        FOUND=1
    fi
done

if [ "$FOUND" -eq 0 ]; then
    echo "Warning: neither /home/dexforce/w1 nor /home/dexforce/workspace exists; nothing installed" >&2
fi

# 清理包内中立暂存目录,保持目标机干净
rm -rf /opt/dexe-interfaces

exit 0
