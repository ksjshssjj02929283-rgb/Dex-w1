#!/bin/sh
set -eu

PACKAGE="dexe_robot_client"
GROUP="dexforce"

case "${1:-}" in
    remove|purge)
        if ! command -v python3 >/dev/null 2>&1 || \
            ! python3 -m pip --version >/dev/null 2>&1; then
            echo "警告: 未找到 python3/pip，跳过卸载 $PACKAGE" >&2
            exit 0
        fi

        if id -u "$GROUP" >/dev/null 2>&1; then
            if command -v runuser >/dev/null 2>&1; then
                if ! runuser -u "$GROUP" -- env HOME="/home/$GROUP" \
                    python3 -m pip uninstall --yes "$PACKAGE"; then
                    echo "警告: pip 未能卸载 $PACKAGE；继续完成 dpkg 卸载" >&2
                fi
            else
                if ! su -s /bin/sh - "$GROUP" -c \
                    "python3 -m pip uninstall --yes '$PACKAGE'"; then
                    echo "警告: pip 未能卸载 $PACKAGE；继续完成 dpkg 卸载" >&2
                fi
            fi
        elif ! python3 -m pip uninstall --yes "$PACKAGE"; then
            echo "警告: pip 未能卸载 $PACKAGE；继续完成 dpkg 卸载" >&2
        fi
        ;;
esac

exit 0
