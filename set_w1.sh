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
    echo "  echo            - Echo robot server state topic once"
    echo "  echo_sim        - Echo robot server state sim topic once"
    echo "  clone ethercat  - Clone dexe_ethercat (dev) repository to parent directory"
    echo "  clone interface - Clone dexe_interfaces (dev) repository to parent directory"
    echo "  home            - Move robot to home joint position"
    echo "  standby         - Move robot to standby joint position"
    echo "  left            - Move left arm (7 joints, value: '[j1,j2,...]' or home/standby)"
    echo "  right           - Move right arm (7 joints, value: '[j1,j2,...]' or home/standby)"
    echo "  stop_move       - Stop current movement"
    echo "  force           - Monitor force_sensor and raw_force_sensor at 10Hz"
    echo "  log             - Print robot server log"
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
    echo "  dyn [args]      - Dynamic parameter identification GUI (python3 script/qt_dyn_params_ident.py)"
    echo "  codegen         - Generate dynamics code (python3 script/dyn_params_ident/dyn_code_generator.py)"
    echo "  dh [left|right] - Compute DH parameters from URDF (python3 script/dyn_params_ident/urdf2dh.py)"
    echo "  gui             - Robot Launcher GUI (python3 script/gui.py)"
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
    echo "  ./set_w1.sh echo_sim"
    echo "  ./set_property.sh force"
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
        ros2 topic echo /feedback/robot_server_state --field data --once
        exit 0
        ;;
    echo_sim)
        echo "Echoing robot server state sim..."
        ros2 topic echo /feedback/robot_server_state_sim --field data --once
        exit 0
        ;;
    force)
        echo "Monitoring force sensor data at 10Hz..."
        python3 - <<'PY'
import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ForceMonitor(Node):
    def __init__(self):
        super().__init__("force_monitor")
        self.latest = None
        self.lock = threading.Lock()
        self.create_subscription(String, "/feedback/robot_server_state", self.callback, 10)
        self.create_timer(0.1, self.print_latest)

    def callback(self, msg):
        try:
            data = json.loads(msg.data)
            latest = (
                data.get("force_sensor", []),
                data.get("raw_force_sensor", []),
                data.get("force_sensor_bias", []),
                data.get("tool_inertia_force_norm", None),
                data.get("tool_inertia_torque_norm", None),
                data.get("tool_inertia_filter_cutoff_hz", None),
            )
        except Exception:
            return
        with self.lock:
            self.latest = latest

    def print_latest(self):
        with self.lock:
            latest = self.latest
        if latest is None:
            return
        (force_sensor, raw_force_sensor, force_sensor_bias,
         tool_inertia_force_norm, tool_inertia_torque_norm, tool_inertia_filter_cutoff_hz) = latest
        print("------------------------------------------------", flush=True)
        print(f"force_sensor: {json.dumps(force_sensor, separators=(',', ':'))},", flush=True)
        print(f"raw_force_sensor: {json.dumps(raw_force_sensor, separators=(',', ':'))},", flush=True)
        print(f"force_sensor_bias: {json.dumps(force_sensor_bias, separators=(',', ':'))}", flush=True)
        # 工具惯量补偿诊断: -1 = 本周期未运行 (首拍/失效/未启用), >=0 = 本周期实际扣减的范数
        f_str = f"{tool_inertia_force_norm:.6f}" if tool_inertia_force_norm is not None else "n/a"
        t_str = f"{tool_inertia_torque_norm:.6f}" if tool_inertia_torque_norm is not None else "n/a"
        c_str = f"{tool_inertia_filter_cutoff_hz:.2f}Hz" if tool_inertia_filter_cutoff_hz is not None else "n/a"
        print(f"tool_inertia_force_norm: {f_str}, tool_inertia_torque_norm: {t_str}, "
              f"tool_inertia_filter_cutoff_hz: {c_str}", flush=True)


rclpy.init()
node = ForceMonitor()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
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
            cat ~/log/robot_server.txt
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
