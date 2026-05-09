cd "$(dirname "$0")"
source /opt/w1_server/setup.bash

CONFIG_FILE="$HOME/dexe_robot_server_data/robot_config.yaml"

if [ ! -f "$CONFIG_FILE" ]; then
    CONFIG_FILE="$(dirname "$0")/../w1_server/robot_config.yaml"
fi

if [ $# -eq 0 ]; then
    echo "Usage: $0 <read>"
    echo "       $0 <true|false>"
    echo ""
    echo "Examples:"
    echo "  $0 read                    - Read record status"
    echo "  $0 true                    - Enable record"
    echo "  $0 false                   - Disable record"
    exit 1
fi

case "$1" in
    read)
        if [ -f "$CONFIG_FILE" ]; then
            # 读取 enable_record 配置
            ENABLED=$(grep "^enable_record:" "$CONFIG_FILE" | head -1 | awk '{print $2}')
            
            if [ -n "$ENABLED" ]; then
                echo "Record enabled: $ENABLED"
            else
                echo "Error: Could not find enable_record in $CONFIG_FILE"
                exit 1
            fi
        else
            echo "Error: Config file not found: $CONFIG_FILE"
            exit 1
        fi
        ;;
    true|false)
        # EnableRecord 需要布尔值字符串 true/false
        VALUE="$1"
        
        # 更新配置文件
        if [ -f "$CONFIG_FILE" ]; then
            sed -i "s/^enable_record:.*/enable_record: $VALUE/" "$CONFIG_FILE"
            if [ $? -eq 0 ]; then
                echo "Updated enable_record to $VALUE in $CONFIG_FILE"
            else
                echo "Warning: Failed to update config file"
            fi
        else
            echo "Warning: Config file not found: $CONFIG_FILE"
        fi
        
        # 通过ROS2服务设置参数
        ros2 service call /control/set_property robot_server_interfaces/srv/SetProperty "{property_type: 'EnableRecord', value: '$VALUE'}"
        
        echo "Record set to $1"
        ;;
    *)
        echo "Error: Invalid argument '$1'"
        echo "Usage: $0 <read>"
        echo "       $0 <true|false>"
        exit 1
        ;;
esac