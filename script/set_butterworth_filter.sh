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
    echo "  $0 read                    - Read butterworth filter status for all joints"
    echo "  $0 true                    - Enable butterworth filter"
    echo "  $0 false                   - Disable butterworth filter"
    exit 1
fi

case "$1" in
    read)
        if [ -f "$CONFIG_FILE" ]; then
            echo "Butterworth Filter Configuration:"
            echo "=============================="
            
            # 读取 joints 部分的所有关节配置
            JOINT_NAME=""
            WHILE_IN_JOINTS=0
            WHILE_IN_BUTTERWORTH=0
            
            while IFS= read -r line; do
                # 检测 joints 部分的开始
                if echo "$line" | grep -q "^joints:"; then
                    WHILE_IN_JOINTS=1
                    continue
                fi
                
                # 如果在 joints 部分中
                if [ $WHILE_IN_JOINTS -eq 1 ]; then
                    # 检测新的关节名称
                    if echo "$line" | grep -q '^\s*- name:'; then
                        JOINT_NAME=$(echo "$line" | sed 's/.*name: *"\?\([^"]*\)"\?.*/\1/')
                        WHILE_IN_BUTTERWORTH=0
                    fi
                    
                    # 检测 butterworth_filter 部分
                    if echo "$line" | grep -q '^\s*butterworth_filter:'; then
                        WHILE_IN_BUTTERWORTH=1
                        continue
                    fi
                    
                    # 如果在 butterworth_filter 部分中，读取 enabled 和 frequency
                    if [ $WHILE_IN_BUTTERWORTH -eq 1 ]; then
                        if echo "$line" | grep -q '^\s*enabled:'; then
                            ENABLED=$(echo "$line" | awk '{print $2}')
                        elif echo "$line" | grep -q '^\s*frequency:'; then
                            FREQUENCY=$(echo "$line" | awk '{print $2}')
                            # 输出这一行的信息
                            echo "$JOINT_NAME: enabled=$ENABLED, frequency=$FREQUENCY"
                            WHILE_IN_BUTTERWORTH=0
                        fi
                    fi
                    
                    # 检测是否离开了 joints 部分
                    if echo "$line" | grep -q '^[a-z_]*:$' && [ -n "$JOINT_NAME" ]; then
                        # 检查是否是新的顶级部分
                        if ! echo "$line" | grep -q '^\s'; then
                            WHILE_IN_JOINTS=0
                        fi
                    fi
                fi
            done < "$CONFIG_FILE"
        else
            echo "Error: Config file not found: $CONFIG_FILE"
            exit 1
        fi
        ;;
    true|false)
        # 更新配置文件
        if [ -f "$CONFIG_FILE" ]; then
            # 替换所有关节下的 butterworth_filter 使能状态
            # 使用 sed 匹配 butterworth_filter: 后面的 enabled: 行并修改
            sed -i "/butterworth_filter:/!b;n;s/enabled: .*/enabled: $1/" "$CONFIG_FILE"
            if [ $? -eq 0 ]; then
                echo "Updated butterworth_filter enabled to $1 in $CONFIG_FILE"
            else
                echo "Warning: Failed to update config file"
            fi
        else
            echo "Warning: Config file not found: $CONFIG_FILE"
        fi

        if [ "$1" = "true" ]; then
            VALUE="1"
        else
            VALUE="0"
        fi
        
        # 通过ROS2服务设置参数
        ros2 service call /control/set_property robot_server_interfaces/srv/SetProperty "{property_type: 'EnableButterworthFilter', value: '$VALUE'}"
        
        echo "Butterworth filter set to $1"
        ;;
    *)
        echo "Error: Invalid argument '$1'"
        echo "Usage: $0 <read>"
        echo "       $0 <true|false>"
        exit 1
        ;;
esac