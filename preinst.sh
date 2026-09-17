#!/bin/sh
set -eu

# 清理上一次被中断安装留下的 wheel。pip 已安装的包不在这里删除，
# 这样升级失败时仍可回滚到原有运行环境。
TARGET_DIR="/home/dexforce/w1/dexe_robot_client"

case "${1:-}" in
    install|upgrade)
        if [ -d "$TARGET_DIR" ]; then
            find "$TARGET_DIR" -maxdepth 1 -type f -name '*.whl' -delete
        fi
        ;;
esac

exit 0
