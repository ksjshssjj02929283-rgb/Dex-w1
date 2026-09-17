#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
PROJECT_DIR=$(basename "$SCRIPT_DIR")
PACKAGE_NAME=${PROJECT_DIR//_/-}
WHEEL_DIR="$SCRIPT_DIR/dist"
DEBIAN_ROOT="$SCRIPT_DIR/debian/$PACKAGE_NAME"
STAGE_PROJECT="$DEBIAN_ROOT/home/dexforce/w1/$PROJECT_DIR"

echo "开始准备 $PACKAGE_NAME 的 Debian 文件..."

if [[ ! -d "$WHEEL_DIR" ]]; then
    echo "错误: 未找到 wheel 目录 $WHEEL_DIR" >&2
    exit 1
fi

shopt -s nullglob
wheel_files=("$WHEEL_DIR"/*.whl)
shopt -u nullglob
if (( ${#wheel_files[@]} == 0 )); then
    echo "错误: $WHEEL_DIR 中未找到 wheel 文件" >&2
    exit 1
fi

first_wheel=${wheel_files[0]}
wheel_name=$(basename "$first_wheel")
VERSION=$(awk -F- '{print $2}' <<< "$wheel_name")
if [[ -z "$VERSION" || "$VERSION" == "$wheel_name" ]]; then
    echo "错误: 无法从 wheel 文件名解析版本号: $wheel_name" >&2
    exit 1
fi
echo "使用 wheel: $wheel_name (版本 $VERSION)"

# dh_auto_install 可能在一次 debuild 中被重复调用；每次从干净目录开始，
# 避免旧 wheel 或旧维护脚本混入新包。
rm -rf "$DEBIAN_ROOT"
mkdir -p "$STAGE_PROJECT"
cp -- "${wheel_files[@]}" "$STAGE_PROJECT/"

# CI 的 gen_deb.sh 会先生成 debian/control。直接更新该文件，避免为了
# 声明 Debian 依赖再引入额外的 requirements.txt 约定；重复执行时不会重复追加。
SOURCE_CONTROL="$SCRIPT_DIR/debian/control"
if [[ -f "$SOURCE_CONTROL" ]]; then
    CONTROL_DEPS="python3, python3-pip, dexe-robot-planning"
    if grep -q '^Depends:' "$SOURCE_CONTROL"; then
        current_deps=$(sed -n 's/^Depends:[[:space:]]*//p' "$SOURCE_CONTROL" | head -n1)
        for dependency in python3 python3-pip dexe-robot-planning; do
            if ! grep -Eq "(^|,)[[:space:]]*$dependency([[:space:]]*,|$)" <<< "$current_deps"; then
                if [[ -n "$current_deps" ]]; then
                    current_deps="$current_deps, $dependency"
                else
                    current_deps="$dependency"
                fi
            fi
        done
        sed -i "s|^Depends:.*|Depends: $current_deps|" "$SOURCE_CONTROL"
    else
        sed -i "/^Architecture:/a Depends: $CONTROL_DEPS" "$SOURCE_CONTROL"
    fi
fi

# commonpub/gen_deb.sh 已经在 debian/ 下生成 control 和维护脚本。这里仅
# 在本地直接运行 install.sh 时补齐 DEBIAN 元数据；CI 路径不会把嵌套的
# DEBIAN 目录误打进 /home/dexforce/w1/$PROJECT_DIR。
if [[ ! -f "$SCRIPT_DIR/debian/control" ]]; then
    DEBIAN_DIR="$DEBIAN_ROOT/DEBIAN"
    mkdir -p "$DEBIAN_DIR"
    cat > "$DEBIAN_DIR/control" <<EOF
Package: $PACKAGE_NAME
Version: $VERSION
Architecture: ${DEB_ARCH:-arm64}
Depends: python3, python3-pip, dexe-robot-planning
Maintainer: Dexforce <dexforce@dexforce.com>
Description: $PROJECT_DIR Robot Client
EOF
fi

# 根目录钩子是 CI gen_deb.sh 和本地 dpkg-deb 共用的单一来源。
for hook in preinst postinst; do
    if [[ -f "$SCRIPT_DIR/$hook.sh" ]]; then
        if [[ -f "$SCRIPT_DIR/debian/control" ]]; then
            cp "$SCRIPT_DIR/$hook.sh" "$SCRIPT_DIR/debian/$hook"
            chmod 0755 "$SCRIPT_DIR/debian/$hook"
        else
            cp "$SCRIPT_DIR/$hook.sh" "$DEBIAN_ROOT/DEBIAN/$hook"
            chmod 0755 "$DEBIAN_ROOT/DEBIAN/$hook"
        fi
    fi
done

echo "Debian 文件准备完成: $DEBIAN_ROOT"
