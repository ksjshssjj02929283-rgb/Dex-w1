#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "${SCRIPT_DIR}/../install/setup.bash"

# === 检查 Sudo 权限 ===
# 尝试使用默认密码验证，如果失败则提示用户手动输入
if echo "123456" | sudo -S -v >/dev/null 2>&1; then
    SUDO_RUN() { echo "123456" | sudo -S "$@"; }
else
    # 仅在需要 sudo 时提示
    SUDO_RUN() { sudo "$@"; }
fi

# 打印使用说明
print_usage() {
    echo "Usage: ./set_property.sh <property_type> [value]"
    echo ""
    echo "Available property types:"
    echo "  echo            - Echo robot server state topic continuously"
    echo "  echo_once [field] - Echo robot server state topic once (default field: data)"
    echo "  echo_sim        - Echo robot server state sim topic continuously"
    echo "  echo_sim_once [field] - Echo robot server state sim topic once (default field: data)"
    echo "  clone ethercat  - Clone dexe_ethercat (dev) repository to parent directory"
    echo "  clone interface - Clone dexe_interfaces (dev) repository to parent directory"
    echo "  home            - Move robot to home joint position"
    echo "  standby         - Move robot to standby joint position"
    echo "  left            - Move left arm (7 joints, value: '[j1,j2,...]' or home/standby)"
    echo "  right           - Move right arm (7 joints, value: '[j1,j2,...]' or home/standby)"
    echo "  stop_move       - Stop current movement"
    echo "  log [lines]     - Follow robot server log (tail -n lines -f)"
    echo "  clear           - Clear errors (no value needed)"
    echo "  drag            - Enable/Disable drag mode (value: 0 or 1)"
    echo "  start           - Start dexe-system.service"
    echo "  stop            - Stop dexe-system.service"
    echo "  restart         - Restart dexe-system.service"
    echo "  sleep           - Sleep mode"
    echo "  quickstop       - QuickStop mode"
    echo "  watch           - Watch EtherCAT status (watch -n 0.1 ethercat sl)"
    echo "  state           - Monitor feedback state (python3 script/monitor_state.py)"
    echo "  shm             - Monitor shared memory (python3 script/monitor_shm.py)"
    echo "  ctrl            - Joint control GUI (python3 script/qt_joint_control.py)"
    echo "  analyzer        - Joint analyzer GUI (python3 script/joint_analyzer.py)"
    echo "  dyn [args]      - Dynamic parameter identification GUI (python3 script/qt_dyn_params_ident.py)"
    echo "  codegen         - Generate dynamics code (python3 script/dyn_params_ident/dyn_code_generator.py)"
    echo "  dh [left|right] - Compute DH parameters from URDF (python3 script/dyn_params_ident/urdf2dh.py)"
    echo "  gui             - Robot Launcher GUI (python3 script/gui.py)"
    echo "  tool            - Edit end-effector tool dynamics config (python3 script/set_tool_dynamics.py)"
    echo "  trajgen         - Trajectory generator (python3 script/traj_generator.py)"
    echo "  vis             - Visualization ROS2 node (python3 script/visual_ros2_node.py)"
    echo "  config          - Edit robot_config.yaml (gedit/vim ~/dexe_robot_server_data/robot_config.yaml)"
    echo "  pos_limit       - Enable/Disable position limit check (value: 0 or 1)"
    echo "  mode            - Switch run mode (value: mode number, e.g., 3)"
    echo "  safe_level      - Set safety level (value: float, e.g., 0.3)"
    echo "  ruckig          - Enable/Disable ruckig (value: true or false)"
    echo "  filter          - Enable/Disable butterworth filter (value: true or false)"
    echo "  zvd             - Enable/Disable input shaper (value: true or false)"
    echo "  ethercat run   - Run dexe_ethercat (./../dexe_ethercat/run.sh)"
    echo "  ethercat build - Build dexe_ethercat (./../dexe_ethercat/build.sh)"
    echo "  ethercat git   - Show dexe_ethercat git log"
    echo "  hardware        - Cat /etc/dexe_robot/hardware.yaml"

    echo ""
    echo "Examples:"
    echo "  ./set_w1.sh echo"
    echo "  ./set_w1.sh echo_once"
    echo "  ./set_w1.sh echo_once server_version"
    echo "  ./set_w1.sh echo_sim"
    echo "  ./set_w1.sh echo_sim_once"
    echo "  ./set_w1.sh clone ethercat"
    echo "  ./set_w1.sh clone interface"
    echo "  ./set_w1.sh home"
    echo "  ./set_w1.sh standby"
    echo "  ./set_w1.sh left home"
    echo "  ./set_w1.sh left '[0.0, -1.4, 0.0, -0.6, 0.0, -0.5, -0.5]'"
    echo "  ./set_w1.sh right standby"
    echo "  ./set_w1.sh right '[0.0, 1.4, 0.0, 0.6, 0.0, 0.5, 0.5]'"
    echo "  ./set_w1.sh stop_move"
    echo "  ./set_w1.sh log"
    echo "  ./set_w1.sh log 100"
    echo "  ./set_w1.sh clear"
    echo "  ./set_w1.sh drag 1"
    echo "  ./set_w1.sh start"
    echo "  ./set_w1.sh stop"
    echo "  ./set_w1.sh restart"
    echo "  ./set_w1.sh sleep"
    echo "  ./set_w1.sh quickstop"
    echo "  ./set_w1.sh watch"
    echo "  ./set_w1.sh state"
    echo "  ./set_w1.sh shm"
    echo "  ./set_w1.sh ctrl"
    echo "  ./set_w1.sh dyn"
    echo "  ./set_w1.sh codegen"
    echo "  ./set_w1.sh dh left"
    echo "  ./set_w1.sh dh"
    echo "  ./set_w1.sh gui"
    echo "  ./set_w1.sh tool"
    echo "  ./set_w1.sh tool --list"
    echo "  ./set_w1.sh tool --set-enabled primary false"
    echo "  ./set_w1.sh tool --set-tool 'BrainCo_Revo1_R' 0.5 0.0 0.0 0.0"
    echo "  ./set_w1.sh tool --set-tool 'BrainCo_Revo1_R'"
    echo "  ./set_w1.sh trajgen"
    echo "  ./set_w1.sh vis"
    echo "  ./set_w1.sh config"
    echo "  ./set_w1.sh pos_limit 0"
    echo "  ./set_w1.sh mode 1"
    echo "  ./set_w1.sh safe_level 0.3"
    echo "  ./set_w1.sh ruckig true"
    echo "  ./set_w1.sh filter true"
    echo "  ./set_w1.sh zvd true"
    echo "  ./set_w1.sh ethercat run"
    echo "  ./set_w1.sh ethercat build"
    echo "  ./set_w1.sh ethercat git"
    echo "  ./set_w1.sh hardware"
}

# 检查输入参数
if [ $# -lt 1 ]; then
    print_usage
    exit 1
fi

PROPERTY_KEY=$1
VALUE=$2

# 根据输入映射属性类型
case $PROPERTY_KEY in
    drag)
        PROPERTY_TYPE="EnableDrag"
        ;;
    pos_limit)
        PROPERTY_TYPE="EnablePosLimitCheck"
        ;;
    mode)
        PROPERTY_TYPE="SwitchRunMode"
        ;;
    safe_level)
        PROPERTY_TYPE="SafeLevel"
        ;;
    ruckig)
        echo "Running set_ruckig.sh ${@:2}..."
        "${SCRIPT_DIR}/script/set_ruckig.sh" "${@:2}"
        exit 0
        ;;
    filter)
        echo "Running set_butterworth_filter.sh ${@:2}..."
        "${SCRIPT_DIR}/script/set_butterworth_filter.sh" "${@:2}"
        exit 0
        ;;
    zvd)
        echo "Running set_input_shaper.sh ${@:2}..."
        "${SCRIPT_DIR}/script/set_input_shaper.sh" "${@:2}"
        exit 0
        ;;
    clear)
        PROPERTY_TYPE="ClearError"
        VALUE=""
        ;;
    echo)
        echo "Echoing robot server state..."
        ros2 topic echo /feedback/robot_server_state --field data
        exit 0
        ;;
    echo_once)
        if [ -n "$VALUE" ]; then
            echo "Searching robot server state for keys containing: '$VALUE'..."
            ros2 topic echo /feedback/robot_server_state --field data --once | python3 -c "import sys, json; raw=sys.stdin.read(); data=json.loads(raw.split('---')[0]); matches={k:v for k,v in data.items() if '$VALUE' in k}; print(json.dumps(matches, indent=2)) if matches else print('No keys containing \'$VALUE\' found')"
        else
            echo "Echoing robot server state..."
            ros2 topic echo /feedback/robot_server_state --field data --once
        fi
        exit 0
        ;;
    echo_sim)
        echo "Echoing robot server state sim..."
        ros2 topic echo /feedback/robot_server_state_sim --field data
        exit 0
        ;;
    echo_sim_once)
        if [ -n "$VALUE" ]; then
            echo "Searching robot server state sim for keys containing: '$VALUE'..."
            ros2 topic echo /feedback/robot_server_state_sim --field data --once | python3 -c "import sys, json; raw=sys.stdin.read(); data=json.loads(raw.split('---')[0]); matches={k:v for k,v in data.items() if '$VALUE' in k}; print(json.dumps(matches, indent=2)) if matches else print('No keys containing \'$VALUE\' found')"
        else
            echo "Echoing robot server state sim..."
            ros2 topic echo /feedback/robot_server_state_sim --field data --once
        fi
        exit 0
        ;;
    clone)
        PARENT_DIR="$(realpath "${SCRIPT_DIR}/..")"
        if [ "$VALUE" == "ethercat" ]; then
            echo "Cloning dexe_ethercat (dev) to ${PARENT_DIR}..."
            rm -rf "${PARENT_DIR}/dexe_ethercat"
            git clone -b dev http://192.168.3.16/dexembodied/dexe_ethercat.git "${PARENT_DIR}/dexe_ethercat"
        elif [ "$VALUE" == "interface" ]; then
            echo "Cloning dexe_interfaces (dev) to ${PARENT_DIR}..."
            rm -rf "${PARENT_DIR}/dexe_interfaces"
            git clone -b dev http://192.168.3.16/dexembodied/dexe_interfaces.git "${PARENT_DIR}/dexe_interfaces"
        else
            echo "Usage: ./set_w1.sh clone [ethercat|interface]"
        fi
        exit 0
        ;;
    home)
        echo "Moving robot to home position..."
        ros2 action send_goal /control/move_robot robot_server_interfaces/action/MoveRobot "{motion_group: WholeBody, type: MoveJoint, target: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], config: '{\"speed\":0.1,\"acceleration\":0.1}'}"
        exit 0
        ;;
    standby)
        echo "Moving robot to standby position..."
        ros2 action send_goal /control/move_robot robot_server_interfaces/action/MoveRobot "{motion_group: WholeBody, type: MoveJoint, target: [0.6, -1.2, 0.6, 0.0, 0.0, 0.0, 0.0, -1.4, 0.0, -0.6, 0.0, -0.5, -0.5, 0.0, 1.4, 0.0, 0.6, 0.0, 0.5, 0.5], config: '{\"speed\":0.1,\"acceleration\":0.1}'}"
        exit 0
        ;;    
    left)
        # Shift all arguments to gather the target array
        shift
        TARGET_VALUE="$*"
        
        # Check for special values
        if [ "$TARGET_VALUE" == "home" ]; then
            TARGET_VALUE="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
        elif [ "$TARGET_VALUE" == "standby" ]; then
            TARGET_VALUE="[0.0, -1.4, 0.0, -0.6, 0.0, -0.5, -0.5]"
        fi

        # Ensure it starts and ends with [ ]
        if [[ ! "$TARGET_VALUE" =~ ^\[.*\]$ ]]; then
            TARGET_VALUE="[$TARGET_VALUE]"
        fi
        echo "Moving left arm to $TARGET_VALUE..."
        ros2 action send_goal /control/move_robot robot_server_interfaces/action/MoveRobot "{motion_group: Primary, type: MoveJoint, target: $TARGET_VALUE, config: '{\"speed\":0.1,\"acceleration\":0.1}'}"
        exit 0
        ;;
    right)
        # Shift all arguments to gather the target array
        shift
        TARGET_VALUE="$*"

        # Check for special values
        if [ "$TARGET_VALUE" == "home" ]; then
            TARGET_VALUE="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
        elif [ "$TARGET_VALUE" == "standby" ]; then
            TARGET_VALUE="[0.0, 1.4, 0.0, 0.6, 0.0, 0.5, 0.5]"
        fi
        
        # Ensure it starts and ends with [ ]
        if [[ ! "$TARGET_VALUE" =~ ^\[.*\]$ ]]; then
            TARGET_VALUE="[$TARGET_VALUE]"
        fi
        echo "Moving right arm to $TARGET_VALUE..."
        ros2 action send_goal /control/move_robot robot_server_interfaces/action/MoveRobot "{motion_group: Secondary, type: MoveJoint, target: $TARGET_VALUE, config: '{\"speed\":0.1,\"acceleration\":0.1}'}"
        exit 0
        ;;
    stop_move)
        echo "Stopping movement..."
        ros2 action send_goal /control/move_robot robot_server_interfaces/action/MoveRobot "{motion_group: Arm, type: StopMove, target: [], config: ''}"
        exit 0
        ;;
    log)
        if [ -f ~/log/robot_server.txt ]; then
            if [ -n "$VALUE" ]; then
                tail -n "$VALUE" -f ~/log/robot_server.txt
            else
                tail -f ~/log/robot_server.txt
            fi
        else
            echo "Error: Log file ~/log/robot_server.txt not found."
        fi
        exit 0
        ;;
    start)
        echo "Starting dexe-system.service..."
        SUDO_RUN systemctl start dexe-system.service
        echo "Watching EtherCAT status..."
        watch -n 0.1 ethercat sl
        exit 0
        ;;
    stop)
        echo "Stopping dexe-system.service..."
        SUDO_RUN systemctl stop dexe-system.service
        exit 0
        ;;
    restart)
        echo "Restarting dexe-system.service..."
        SUDO_RUN systemctl restart dexe-system.service
        echo "Watching EtherCAT status..."
        watch -n 0.1 ethercat sl
        exit 0
        ;;
    sleep)
        PROPERTY_TYPE="EnableSleep"
        ;;
    quickstop)
        PROPERTY_TYPE="QuickStop"
        ;;
    watch)
        echo "Watching EtherCAT status..."
        watch -n 0.1 ethercat sl
        exit 0
        ;;
    state)
        echo "Monitoring feedback state..."
        python3 "${SCRIPT_DIR}/script/monitor_state.py"
        exit 0
        ;;
    shm)
        echo "Monitoring shared memory..."
        python3 "${SCRIPT_DIR}/script/monitor_shm.py"
        exit 0
        ;;
    ctrl)
        echo "Starting Joint Control GUI..."
        python3 "${SCRIPT_DIR}/script/qt_joint_control.py"
        exit 0
        ;;
    analyzer)
        echo "Starting Joint Analyzer GUI..."
        python3 "${SCRIPT_DIR}/script/joint_analyzer.py"
        exit 0
        ;;
    dyn)
        echo "Starting Dynamic Parameter Identification GUI..."
        python3 "${SCRIPT_DIR}/script/qt_dyn_params_ident.py" "${@:2}"
        exit 0
        ;;
    codegen)
        echo "Starting Dynamics Code Generator..."
        python3 "${SCRIPT_DIR}/script/dyn_params_ident/dyn_code_generator.py"
        exit 0
        ;;
    dh)
        echo "Computing DH parameters from URDF..."
        python3 "${SCRIPT_DIR}/script/dyn_params_ident/urdf2dh.py" "${@:2}"
        exit 0
        ;;
    gui)
        echo "Starting Robot Launcher GUI..."
        python3 "${SCRIPT_DIR}/script/gui.py"
        exit 0
        ;;
    tool)
        echo "Starting Tool Dynamics Config Editor..."
        python3 "${SCRIPT_DIR}/script/set_tool_dynamics.py" "${@:2}"
        exit 0
        ;;
    trajgen)
        echo "Starting Trajectory Generator..."
        python3 "${SCRIPT_DIR}/script/dyn_params_ident/traj_generator.py"
        exit 0
        ;;
    vis)
        echo "Starting Visualization ROS2 node..."
        python3 "${SCRIPT_DIR}/script/visual_ros2_node.py"
        exit 0
        ;;
    hardware)
        echo "Cat /etc/dexe_robot/hardware.yaml..."
        cat /etc/dexe_robot/hardware.yaml
        exit 0
        ;;
    config)
        echo "Editing robot_config.yaml..."
        if [ -n "$DISPLAY" ]; then
            gedit ~/dexe_robot_server_data/robot_config.yaml
        else
            vim ~/dexe_robot_server_data/robot_config.yaml
        fi
        exit 0
        ;;
    ethercat)
        ETHERCAT_DIR="${SCRIPT_DIR}/../dexe_ethercat"
        if [ ! -d "$ETHERCAT_DIR" ]; then
            echo "Error: dexe_ethercat directory not found at $ETHERCAT_DIR"
            exit 1
        fi
        case "$VALUE" in
            run)
                echo "Running dexe_ethercat..."
                "$ETHERCAT_DIR/run.sh"
                ;;
            build)
                echo "Building dexe_ethercat..."
                "$ETHERCAT_DIR/build.sh"
                ;;
            git)
                echo "dexe_ethercat git log:"
                git -C "$ETHERCAT_DIR" log
                ;;
            *)
                echo "Usage: ./set_w1.sh ethercat [run|build|git]"
                exit 1
                ;;
        esac
        exit 0
        ;;
    *)
        echo "Unknown property type: $PROPERTY_KEY"
        print_usage
        exit 1
        ;;
esac

# 如果不是 clear 且没有提供 value，则报错
if [ "$PROPERTY_TYPE" != "ClearError" ] && [ -z "$VALUE" ]; then
    echo "Error: Value is required for $PROPERTY_TYPE"
    print_usage
    exit 1
fi

# 执行 ROS2 服务调用
echo "Calling service: /control/set_property with property_type: '$PROPERTY_TYPE' and value: '$VALUE'"

ros2 service call /control/set_property robot_server_interfaces/srv/SetProperty "{property_type: '$PROPERTY_TYPE', value: '$VALUE'}"
