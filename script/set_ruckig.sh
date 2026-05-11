cd "$(dirname "$0")"
source /opt/w1_server/setup.bash

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

case "$1" in
    read)
        # 从配置文件中读取所有值
        if [ -f "$CONFIG_FILE" ]; then
            # 读取 streaming ruckig 开关状态
            RUCKIG_STATUS=$(grep "^enable_streaming_ruckig:" "$CONFIG_FILE" | awk '{print $2}')
            if [ -n "$RUCKIG_STATUS" ]; then
                echo "Streaming ruckig status: $RUCKIG_STATUS"
            else
                echo "Error: Could not find enable_streaming_ruckig in $CONFIG_FILE"
                exit 1
            fi
            
            # 读取 joint streaming bounds 值
            MAX_VEL=$(grep "^  max_velocity:" "$CONFIG_FILE" | head -1 | awk '{print $2}')
            MAX_ACC=$(grep "^  max_acceleration:" "$CONFIG_FILE" | head -1 | awk '{print $2}')
            MAX_JERK=$(grep "^  max_jerk:" "$CONFIG_FILE" | head -1 | awk '{print $2}')
            
            if [ -n "$MAX_VEL" ] && [ -n "$MAX_ACC" ] && [ -n "$MAX_JERK" ]; then
                echo "Joint streaming bounds: $MAX_VEL $MAX_ACC $MAX_JERK"
            else
                echo "Error: Could not find joint streaming bounds in $CONFIG_FILE"
                exit 1
            fi
        else
            echo "Error: Config file not found: $CONFIG_FILE"
            exit 1
        fi
        ;;
    true|false)
        VALUE="$1"
        
        if [ -f "$CONFIG_FILE" ]; then
            sed -i "s/^enable_streaming_ruckig:.*/enable_streaming_ruckig: $VALUE/" "$CONFIG_FILE"
            if [ $? -eq 0 ]; then
                echo "Updated streaming ruckig to $VALUE in $CONFIG_FILE"
            else
                echo "Error: Failed to update config file"
                exit 1
            fi
        else
            echo "Error: Config file not found: $CONFIG_FILE"
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
        # 检测是否为设置 bounds 的格式 (3个数字参数或逗号分隔的字符串)
        if [ $# -eq 3 ] || [ $# -eq 1 ]; then
            # 如果是逗号分隔的字符串，分割成三个参数
            if [ $# -eq 1 ]; then
                # 移除空格和逗号，然后分割
                CLEANED=$(echo "$1" | sed 's/,/ /g' | sed 's/\s+/ /g')
                VELOCITY=$(echo "$CLEANED" | awk '{print $1}')
                ACCELERATION=$(echo "$CLEANED" | awk '{print $2}')
                JERK=$(echo "$CLEANED" | awk '{print $3}')
            else
                VELOCITY="$1"
                ACCELERATION="$2"
                JERK="$3"
            fi
            
            # 验证参数是否为数字
            if ! echo "$VELOCITY" | grep -qE '^[0-9]+(\.[0-9]+)?$' || \
               ! echo "$ACCELERATION" | grep -qE '^[0-9]+(\.[0-9]+)?$' || \
               ! echo "$JERK" | grep -qE '^[0-9]+(\.[0-9]+)?$'; then
                echo "Error: All parameters must be numeric values"
                echo "Usage: $0 <velocity> <acceleration> <jerk>"
                exit 1
            fi
            
            # 更新配置文件
            if [ -f "$CONFIG_FILE" ]; then
                sed -i "s/^  max_velocity:.*/  max_velocity: $VELOCITY/" "$CONFIG_FILE"
                sed -i "s/^  max_acceleration:.*/  max_acceleration: $ACCELERATION/" "$CONFIG_FILE"
                sed -i "s/^  max_jerk:.*/  max_jerk: $JERK/" "$CONFIG_FILE"
                
                if [ $? -eq 0 ]; then
                    echo "Updated joint streaming bounds in $CONFIG_FILE:"
                    echo "  max_velocity: $VELOCITY"
                    echo "  max_acceleration: $ACCELERATION"
                    echo "  max_jerk: $JERK"
                else
                    echo "Error: Failed to update config file"
                    exit 1
                fi
            else
                echo "Error: Config file not found: $CONFIG_FILE"
                exit 1
            fi
            
            # 通过ROS2服务设置参数
            ros2 service call /control/set_property robot_server_interfaces/srv/SetProperty "{property_type: 'JointStreamingBounds', value: '$VELOCITY,$ACCELERATION,$JERK'}"
            
            echo "Joint streaming bounds set to: velocity=$VELOCITY, acceleration=$ACCELERATION, jerk=$JERK"
        else
            echo "Error: Invalid argument '$1'"
            echo "Usage: $0 <read>"
            echo "       $0 <true|false>"
            echo "       $0 <velocity> <acceleration> <jerk>"
            echo "       $0 '<velocity>, <acceleration>, <jerk>'"
            exit 1
        fi
        ;;
esac