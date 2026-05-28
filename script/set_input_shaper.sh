#!/bin/bash
cd "$(dirname "$0")"
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "${SCRIPT_DIR}/../../install/setup.bash"

CONFIG_FILE="$HOME/dexe_robot_server_data/robot_config.yaml"

if [ ! -f "$CONFIG_FILE" ]; then
    CONFIG_FILE="$(dirname "$0")/../w1_server/robot_config.yaml"
fi

if [ $# -eq 0 ]; then
    echo "Usage: $0 <read>"
    echo "       $0 <true|false>"
    echo ""
    echo "Examples:"
    echo "  $0 read                    - Read input shaper status for all joints"
    echo "  $0 true                    - Enable input shaper"
    echo "  $0 false                   - Disable input shaper"
    exit 1
fi

# ==========================================
# 解析 robot_version_key (跟Python脚本逻辑对齐)
# ==========================================
if [ -f "$CONFIG_FILE" ]; then
    ROBOT_TYPE=$(grep "^robot_type:" "$CONFIG_FILE" | awk '{print $2}' | tr -d '"'\''' | tr -d '\r')
    VERSION=$(grep "^version:" "$CONFIG_FILE" | awk '{print $2}' | tr -d '"'\''' | tr -d '\r')
    
    if [ -n "$ROBOT_TYPE" ] && [ -n "$VERSION" ]; then
        VERSION_PREFIX="${VERSION:0:3}"
        ROBOT_VERSION_KEY="${ROBOT_TYPE}_${VERSION_PREFIX}"
    else
        echo "Error: Could not find robot_type or version in $CONFIG_FILE"
        exit 1
    fi
else
    echo "Error: Config file not found: $CONFIG_FILE"
    exit 1
fi

case "$1" in
    read)
        echo "Input Shaper Configuration:"
        echo "=============================="
        echo "ROBOT_VERSION_KEY: $ROBOT_VERSION_KEY"
        awk -v key="$ROBOT_VERSION_KEY:" '
            $1 == key {in_version_block=1; next}
            in_version_block && /^[a-zA-Z0-9_]+:/ {in_version_block=0}
            
            in_version_block && $1 == "joints:" {in_joints=1; next}
            in_version_block && in_joints && /^  [a-zA-Z0-9_]+:/ && $1 != "joints:" {in_joints=0}
            
            in_joints && $1 == "-" && $2 == "name:" {
                joint_name = $3
                gsub(/"/, "", joint_name)
                in_shaper=0
            }
            
            in_joints && $1 == "input_shaper:" {in_shaper=1; next}
            in_joints && in_shaper && $1 == "enabled:" { enabled=$2 }
            in_joints && in_shaper && $1 == "frequency:" { 
                frequency=$2
                printf "%s: enabled=%s, frequency=%s\n", joint_name, enabled, frequency
                in_shaper=0
            }
        ' "$CONFIG_FILE"
        ;;
    true|false)
        VALUE="$1"
        awk -v key="$ROBOT_VERSION_KEY:" -v val="$VALUE" '
            $1 == key {in_version_block=1; print; next}
            in_version_block && /^[a-zA-Z0-9_]+:/ {in_version_block=0}
            
            in_version_block && $1 == "joints:" {in_joints=1; print; next}
            in_version_block && in_joints && /^  [a-zA-Z0-9_]+:/ && $1 != "joints:" {in_joints=0}
            
            in_joints && $1 == "input_shaper:" {in_shaper=1; print; next}
            in_joints && in_shaper && $1 == "enabled:" { 
                sub($2, val)
                in_shaper=0
                print
                next
            }
            {print}
        ' "$CONFIG_FILE" > "${CONFIG_FILE}.tmp" && mv "${CONFIG_FILE}.tmp" "$CONFIG_FILE"

        if [ $? -eq 0 ]; then
            echo "Updated input_shaper enabled to $VALUE under $ROBOT_VERSION_KEY in $CONFIG_FILE"
        else
            echo "Warning: Failed to update config file"
        fi

        if [ "$VALUE" = "true" ]; then
            ROS_VAL="1"
        else
            ROS_VAL="0"
        fi
        
        # 通过ROS2服务设置参数
        ros2 service call /control/set_property robot_server_interfaces/srv/SetProperty "{property_type: 'EnableInputShaper', value: '$ROS_VAL'}"
        
        echo "Input shaper set to $VALUE"
        ;;
    *)
        echo "Error: Invalid argument '$1'"
        echo "Usage: $0 <read>"
        echo "       $0 <true|false>"
        exit 1
        ;;
esac
