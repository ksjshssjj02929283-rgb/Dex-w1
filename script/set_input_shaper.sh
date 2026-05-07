cd "$(dirname "$0")"
source ../../install/setup.bash

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

case "$1" in
    read)
        if [ -f "$CONFIG_FILE" ]; then
            echo "Input Shaper Configuration:"
            echo "=============================="
            
            # 读取 joints 部分的所有关节配置
            JOINT_NAME=""
            WHILE_IN_JOINTS=0
            WHILE_IN_INPUT_SHAPER=0
            
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
                        WHILE_IN_INPUT_SHAPER=0
                    fi
                    
                    # 检测 input_shaper 部分
                    if echo "$line" | grep -q '^\s*input_shaper:'; then
                        WHILE_IN_INPUT_SHAPER=1
                        continue
                    fi
                    
                    # 如果在 input_shaper 部分中，读取 enabled 和 frequency
                    if [ $WHILE_IN_INPUT_SHAPER -eq 1 ]; then
                        if echo "$line" | grep -q '^\s*enabled:'; then
                            ENABLED=$(echo "$line" | awk '{print $2}')
                        elif echo "$line" | grep -q '^\s*frequency:'; then
                            FREQUENCY=$(echo "$line" | awk '{print $2}')
                            # 输出这一行的信息
                            echo "$JOINT_NAME: enabled=$ENABLED, frequency=$FREQUENCY"
                            WHILE_IN_INPUT_SHAPER=0
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
        if [ "$1" = "true" ]; then
            VALUE="1"
        else
            VALUE="0"
        fi
        
        # 通过ROS2服务设置参数
        ros2 service call /control/set_property robot_server_interfaces/srv/SetProperty "{property_type: 'EnableInputShaper', value: '$VALUE'}"
        
        echo "Input shaper set to $1"
        ;;
    *)
        echo "Error: Invalid argument '$1'"
        echo "Usage: $0 <read>"
        echo "       $0 <true|false>"
        exit 1
        ;;
esac