#!/bin/sh
set -eu

TARGET_ROOT="/home/dexforce/w1"
TARGET_DIR="$TARGET_ROOT/dexe_robot_client"
GROUP="dexforce"

case "${1:-}" in
    configure)
        if ! command -v python3 >/dev/null 2>&1; then
            echo "错误: 未找到 python3，无法安装 dexe_robot_client" >&2
            exit 1
        fi
        if ! python3 -m pip --version >/dev/null 2>&1; then
            echo "错误: 未找到 python3-pip，无法安装 dexe_robot_client" >&2
            exit 1
        fi

        mkdir -p "$TARGET_DIR"
        if getent passwd "$GROUP" >/dev/null 2>&1; then
            chown -R "$GROUP":"$GROUP" "$TARGET_ROOT"
        fi

        found=0
        for wheel in "$TARGET_DIR"/*.whl; do
            [ -f "$wheel" ] || continue
            found=1
            echo "正在安装 $(basename "$wheel")..."
            if id -u "$GROUP" >/dev/null 2>&1; then
                if command -v runuser >/dev/null 2>&1; then
                    runuser -u "$GROUP" -- env HOME="/home/$GROUP" \
                        python3 -m pip install --user "$wheel"
                else
                    su -s /bin/sh - "$GROUP" -c \
                        "python3 -m pip install --user '$wheel'"
                fi
            else
                python3 -m pip install --user "$wheel"
            fi
            rm -f "$wheel"
        done

        if [ "$found" -eq 0 ]; then
            echo "错误: $TARGET_DIR 中未找到 wheel 文件" >&2
            exit 1
        fi

        if getent passwd "$GROUP" >/dev/null 2>&1; then
            chown -R "$GROUP":"$GROUP" "$TARGET_ROOT"
        fi
        ;;
esac

exit 0
