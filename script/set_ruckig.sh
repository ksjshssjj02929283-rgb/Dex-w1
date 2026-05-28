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
    echo "       $0 <velocity> <acceleration> <jerk>"
    echo "       $0 '<velocity>, <acceleration>, <jerk>'"
    echo ""
    echo "Examples:"
    echo "  $0 read                    - Read current streaming ruckig status and bounds"
    echo "  $0 true                    - Enable streaming ruckig"
    echo "  $0 false                   - Disable streaming ruckig"
    echo "  $0 3.14 9.42 157.0         - Set joint streaming bounds"
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
        # 使用 awk 提取对应 robot_version_key 块的参数
        # 匹配到 ROBOT_VERSION_KEY: 后，在遇到下一个顶层键(非空格开头)前读取相关配置
        read -r RUCKIG_STATUS MAX_VEL MAX_ACC MAX_JERK < <(awk -v key="$ROBOT_VERSION_KEY:" '
            $1 == key {in_block=1; next}
            in_block && /^[a-zA-Z0-9_]+:/ {in_block=0}
            in_block && $1 == "enable_streaming_ruckig:" {status=$2}
            in_block && $1 == "max_velocity:" {vel=$2}
            in_block && $1 == "max_acceleration:" {acc=$2}
            in_block && $1 == "max_jerk:" {jerk=$2}
            END {print status, vel, acc, jerk}
        ' "$CONFIG_FILE")

        if [ -z "$RUCKIG_STATUS" ]; then
            echo "Error: Could not find enable_streaming_ruckig under block $ROBOT_VERSION_KEY in $CONFIG_FILE"
            exit 1
        else
            echo "ROBOT_VERSION_KEY: $ROBOT_VERSION_KEY"
            echo "Streaming ruckig status: $RUCKIG_STATUS"
        fi
        
        if [ -n "$MAX_VEL" ] && [ -n "$MAX_ACC" ] && [ -n "$MAX_JERK" ]; then
            echo "Joint streaming bounds: $MAX_VEL $MAX_ACC $MAX_JERK"
        else
            echo "Warning: Could not fully find joint streaming bounds under block $ROBOT_VERSION_KEY in $CONFIG_FILE"
        fi
        ;;
    true|false)
        VALUE="$1"
        
        # 只在对应的 robot_version_key 块下匹配并替换 enable_streaming_ruckig
        awk -v key="$ROBOT_VERSION_KEY:" -v val="$VALUE" '
            $1 == key {in_block=1; print; next}
            in_block && /^[a-zA-Z0-9_]+:/ {in_block=0}
            in_block && $1 == "enable_streaming_ruckig:" {
                sub($2, val)
                print
                changed=1
                next
            }
            {print}
        ' "$CONFIG_FILE" > "${CONFIG_FILE}.tmp" && mv "${CONFIG_FILE}.tmp" "$CONFIG_FILE"
        
        if [ $? -eq 0 ]; then
            echo "Updated streaming ruckig to $VALUE under $ROBOT_VERSION_KEY in $CONFIG_FILE"
        else
            echo "Error: Failed to update config file"
            exit 1
        fi
        
        if [ "$VALUE" = "true" ]; then
            ros2 service call /control/set_property robot_server_interfaces/srv/SetProperty "{property_type: 'EnableStreamingRuckig', value: '1'}"
        else
            ros2 service call /control/set_property robot_server_interfaces/srv/SetProperty "{property_type: 'EnableStreamingRuckig', value: '0'}"
        fi
        
        echo "Streaming ruckig set to $VALUE"
        ;;
    *)
        # 检测是否为设置 bounds 的格式
        if [ $# -eq 3 ] || [ $# -eq 1 ]; then
            if [ $# -eq 1 ]; then
                CLEANED=$(echo "$1" | sed 's/,/ /g' | sed 's/\s+/ /g')
                VELOCITY=$(echo "$CLEANED" | awk '{print $1}')
                ACCELERATION=$(echo "$CLEANED" | awk '{print $2}')
                JERK=$(echo "$CLEANED" | awk '{print $3}')
            else
                VELOCITY="$1"
                ACCELERATION="$2"
                JERK="$3"
            fi
            
            if ! echo "$VELOCITY" | grep -qE '^[0-9]+(\.[0-9]+)?$' || \
               ! echo "$ACCELERATION" | grep -qE '^[0-9]+(\.[0-9]+)?$' || \
               ! echo "$JERK" | grep -qE '^[0-9]+(\.[0-9]+)?$'; then
                echo "Error: All parameters must be numeric values"
                echo "Usage: $0 <velocity> <acceleration> <jerk>"
                exit 1
            fi
            
            # 使用 awk 在对应的块下面替换
            awk -v key="$ROBOT_VERSION_KEY:" -v vel="$VELOCITY" -v acc="$ACCELERATION" -v jerk="$JERK" '
                $1 == key {in_block=1; print; next}
                in_block && /^[a-zA-Z0-9_]+:/ {in_block=0}
                in_block && $1 == "max_velocity:" { sub($2, vel); print; next }
                in_block && $1 == "max_acceleration:" { sub($2, acc); print; next }
                in_block && $1 == "max_jerk:" { sub($2, jerk); print; next }
                {print}
            ' "$CONFIG_FILE" > "${CONFIG_FILE}.tmp" && mv "${CONFIG_FILE}.tmp" "$CONFIG_FILE"

            if [ $? -eq 0 ]; then
                echo "Updated joint streaming bounds under $ROBOT_VERSION_KEY in $CONFIG_FILE:"
                echo "  max_velocity: $VELOCITY"
                echo "  max_acceleration: $ACCELERATION"
                echo "  max_jerk: $JERK"
            else
                echo "Error: Failed to update config file"
                exit 1
            fi
            
            ros2 service call /control/set_property robot_server_interfaces/srv/SetProperty "{property_type: 'JointStreamingBounds', value: '$VELOCITY,$ACCELERATION,$JERK'}"
            
            echo "Joint streaming bounds set to: velocity=$VELOCITY, acceleration=$ACCELERATION, jerk=$JERK"
        else
            echo "Error: Invalid argument '$1'"
            exit 1
        fi
        ;;
esac
