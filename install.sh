#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
CURDIR="$SCRIPT_DIR"
PROJECT_DIR=$(basename "$CURDIR")
PACKAGE_NAME=${PROJECT_DIR//_/-}

echo "开始构建dexe_robot_client..."

# 检查whl文件
WHEEL_DIR="dist"
if [ ! -d "$WHEEL_DIR" ]; then
  echo "错误: 未找到whl目录 $WHEEL_DIR"
  exit 1
fi

shopt -s nullglob
wheel_files=($WHEEL_DIR/*.whl)
shopt -u nullglob
if [ ${#wheel_files[@]} -eq 0 ]; then
  echo "错误: 未找到生成的whl文件"
  exit 1
fi
echo "已生成whl包: $(basename "${wheel_files[0]}")"

VERSION=$(basename "${wheel_files[0]}" | awk -F- '{print $2}')
if [ -z "$VERSION" ]; then
  echo "错误: 无法从whl文件名解析版本号: $(basename "${wheel_files[0]}")"
  exit 1
fi
echo "已解析版本号: $VERSION"

# 准备deb目录结构
echo "准备deb包目录..."
STAGE_ROOT="$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1"
STAGE_PROJECT="$STAGE_ROOT/$PROJECT_DIR"
DEBIAN_DIR="$CURDIR/debian/$PACKAGE_NAME/DEBIAN"

mkdir -p "$STAGE_PROJECT"
mkdir -p "$DEBIAN_DIR"

# 复制whl文件
cp "${wheel_files[@]}" "$STAGE_PROJECT/"

# 生成postinst脚本
cat > "$DEBIAN_DIR/postinst" <<EOF
#!/bin/sh
set -e

case "\$1" in
  configure)
    TARGET_DIR="/home/dexforce/w1/$PROJECT_DIR"
    chown -R dexforce:dexforce /home/dexforce/w1

    if ! command -v python3 >/dev/null 2>&1; then
      echo "Warning: python3 not found; skipping pip install."
      exit 0
    fi

    if ! python3 -m pip --version >/dev/null 2>&1; then
      echo "Warning: pip not available; skipping pip install."
      exit 0
    fi

    # 以dexforce用户身份安装，避免root权限问题
    echo "正在以dexforce用户身份安装Python包..."
    su - dexforce -c "python3 -m pip install --user \$TARGET_DIR/*.whl"
    echo "安装成功！包已安装到dexforce用户的本地目录"
    
    # 清理whl文件
    rm -f \$TARGET_DIR/*.whl
    echo "安装完成"
    ;;
esac

exit 0
EOF
chmod 0755 "$DEBIAN_DIR/postinst"
# debhelper(dh_installdeb)会用 debian/postinst(源)覆盖 DEBIAN/postinst,
# 而 gen_deb.sh 已在 debian/postinst 放了默认版;这里把我们的版本写过去,确保生效。
mkdir -p "$CURDIR/debian"
cp "$DEBIAN_DIR/postinst" "$CURDIR/debian/postinst"

# 生成prerm脚本（卸载前执行）
cat > "$DEBIAN_DIR/prerm" <<EOF
#!/bin/sh
set -e

case "\$1" in
  remove|upgrade)
    # 获取whl包的实际包名
    TARGET_DIR="/home/dexforce/w1/$PROJECT_DIR"
    if [ -d "\$TARGET_DIR" ]; then
      # 查找可能的whl包文件（虽然通常已删除，但作为备份检查）
      # 修复语法：避免在for循环中使用2>/dev/null
      for whl_file in "\$TARGET_DIR"/*.whl; do
        if [ -f "\$whl_file" ]; then
          # 从whl文件名中提取包名
          PACKAGE_NAME=\$(basename "\$whl_file" | sed 's/-[0-9].*//')
          echo "正在卸载包: \$PACKAGE_NAME"
          # 将错误重定向移到su命令内部
          su - dexforce -c "python3 -m pip uninstall -y \$PACKAGE_NAME 2>/dev/null" || true
        fi
      done
      
      # 尝试从常见包名中卸载
      POSSIBLE_NAMES="dexe_robot_client dexe_rc dexeclient"
      for pkg in \$POSSIBLE_NAMES; do
        # 将错误重定向移到su命令内部
        su - dexforce -c "python3 -m pip uninstall -y \$pkg 2>/dev/null" || true
      done
    fi
    ;;
esac

exit 0
EOF
chmod 0755 "$DEBIAN_DIR/prerm"

# 生成postrm脚本（卸载后执行）
cat > "$DEBIAN_DIR/postrm" <<EOF
#!/bin/sh
set -e

case "\$1" in
  purge|remove)
    # 清理安装目录
    TARGET_DIR="/home/dexforce/w1/$PROJECT_DIR"
    if [ -d "\$TARGET_DIR" ]; then
      echo "清理安装目录..."
      rm -rf "\$TARGET_DIR"
    fi
    
    # 如果目录为空，删除父目录
    if [ -d "/home/dexforce/w1" ] && [ -z "\$(ls -A /home/dexforce/w1 2>/dev/null)" ]; then
      rmdir /home/dexforce/w1
    fi
    ;;
esac

exit 0
EOF
chmod 0755 "$DEBIAN_DIR/postrm"

# 生成control文件
cat > "$DEBIAN_DIR/control" <<EOF
Package: $PACKAGE_NAME
Version: $VERSION
Architecture: arm64
Depends: python3, python3-pip
Maintainer: Dexforce <dexforce@dexforce.com>
Description: $PROJECT_DIR Robot Client
EOF
