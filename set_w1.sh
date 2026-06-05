#!/bin/bash
source /opt/w1_server/setup.bash
# 打印使用说明
print_usage() {
    echo "Usage: ./set_property.sh <property_type> [value]"
    echo ""
    echo "Available property types:"
    echo "  echo            - Echo robot server state topic once"
    echo "  force           - Monitor dual-arm torque_ext(14D), force_sensor, force_sensor_gravity, raw_force_sensor, force_sensor_bias, and force_sensor_inertia at 10Hz"
    echo "  log             - Print robot server log"
    echo "  clear_error     - Clear errors (no value needed)"
    echo "  drag            - Enable/Disable drag mode (value: 0 or 1)"
    echo "  pos_limit       - Enable/Disable position limit check (value: 0 or 1)"
    echo "  run_mode        - Switch run mode (value: mode number, e.g., 3)"
    echo "  safe_level      - Set safety level (value: float, e.g., 0.3)"
    echo "  start           - Start dexe-system.service"
    echo "  stop            - Stop dexe-system.service"
    echo "  restart         - Restart dexe-system.service"
    echo "  watch           - Watch EtherCAT status (watch -n 0.1 ethercat sl)"
    echo "  state           - Monitor feedback state (python3 script/monitor_feedback.py)"
    echo "  shm             - Monitor shared memory (python3 script/monitor_shm.py)"
    echo "  yaml            - Edit robot_config.yaml (vim ~/dexe_robot_server_data/robot_config.yaml)"
    echo ""
    echo "Examples:"
    echo "  ./set_property.sh echo"
    echo "  ./set_property.sh force"
    echo "  ./set_property.sh yaml"
    echo "  ./set_property.sh log"
    echo "  ./set_property.sh watch"
    echo "  ./set_property.sh state"
    echo "  ./set_property.sh shm"
    echo "  ./set_property.sh clear_error"
    echo "  ./set_property.sh drag 1"
    echo "  ./set_property.sh pos_limit 0"
    echo "  ./set_property.sh run_mode 3"
    echo "  ./set_property.sh safe_level 0.3"
    echo "  ./set_property.sh restart"
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
    run_mode)
        PROPERTY_TYPE="SwitchRunMode"
        ;;
    safe_level)
        PROPERTY_TYPE="SafeLevel"
        ;;
    clear_error)
        PROPERTY_TYPE="ClearError"
        VALUE=""
        ;;
    echo)
        echo "Echoing robot server state..."
        ros2 topic echo /feedback/robot_server_state --field data --once
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

ARM_JOINT_NAMES = (
    "LEFT_J1", "LEFT_J2", "LEFT_J3", "LEFT_J4", "LEFT_J5", "LEFT_J6", "LEFT_J7",
    "RIGHT_J1", "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7",
)


def select_dual_arm_torque_ext(data):
    torque_ext = data.get("torque_ext", [])
    if not isinstance(torque_ext, list):
        return []

    joint_names = data.get("joint_name", [])
    if isinstance(joint_names, list) and len(joint_names) == len(torque_ext):
        torque_by_name = dict(zip(joint_names, torque_ext))
        selected = [torque_by_name[name] for name in ARM_JOINT_NAMES if name in torque_by_name]
        if selected:
            return selected

    if len(torque_ext) == len(ARM_JOINT_NAMES):
        return torque_ext
    return torque_ext[:len(ARM_JOINT_NAMES)]


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
                select_dual_arm_torque_ext(data),
                data.get("force_sensor", []),
                data.get("force_sensor_gravity", []),
                data.get("raw_force_sensor", []),
                data.get("force_sensor_bias", []),
                data.get("force_sensor_inertia", []),
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
        (torque_ext, force_sensor, force_sensor_gravity, raw_force_sensor, force_sensor_bias,
         force_sensor_inertia) = latest
        print("------------------------------------------------", flush=True)
        print(f"torque_ext: {json.dumps(torque_ext, separators=(',', ':'))},", flush=True)
        print(f"force_sensor: {json.dumps(force_sensor, separators=(',', ':'))},", flush=True)
        print(f"force_sensor_gravity: {json.dumps(force_sensor_gravity, separators=(',', ':'))},", flush=True)
        print(f"raw_force_sensor: {json.dumps(raw_force_sensor, separators=(',', ':'))},", flush=True)
        print(f"force_sensor_bias: {json.dumps(force_sensor_bias, separators=(',', ':'))}", flush=True)
        print(f"force_sensor_inertia: {json.dumps(force_sensor_inertia, separators=(',', ':'))}", flush=True)


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
        sudo systemctl start dexe-system.service
        exit 0
        ;;
    stop)
        echo "Stopping dexe-system.service..."
        sudo systemctl stop dexe-system.service
        exit 0
        ;;
    restart)
        echo "Restarting dexe-system.service..."
        sudo systemctl restart dexe-system.service
        exit 0
        ;;
    watch)
        echo "Watching EtherCAT status..."
        watch -n 0.1 ethercat sl
        exit 0
        ;;
    state)
        echo "Monitoring feedback state..."
        python3 script/monitor_feedback.py
        exit 0
        ;;
    shm)
        echo "Monitoring shared memory..."
        python3 script/monitor_shm.py
        exit 0
        ;;
    yaml)
        echo "Editing robot_config.yaml..."
        vim ~/dexe_robot_server_data/robot_config.yaml
        exit 0
        ;;
    *)
        echo "Unknown property type: $PROPERTY_KEY"
        print_usage
        exit 1
        ;;
esac

# 如果不是 clear_error 且没有提供 value，则报错
if [ "$PROPERTY_TYPE" != "ClearError" ] && [ -z "$VALUE" ]; then
    echo "Error: Value is required for $PROPERTY_TYPE"
    print_usage
    exit 1
fi

# 执行 ROS2 服务调用
echo "Calling service: /control/set_property with property_type: '$PROPERTY_TYPE' and value: '$VALUE'"

ros2 service call /control/set_property robot_server_interfaces/srv/SetProperty "{property_type: '$PROPERTY_TYPE', value: '$VALUE'}"
