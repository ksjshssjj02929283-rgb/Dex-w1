#!/bin/bash
# L1 机器人统一操作入口脚本（借鉴 set_w1.sh 的设计，适配 L1 的接口体系）
#
# 注意：L1 与 W1 的控制接口不同——
#   - L1 没有 /control/move_robot action 和 /control/set_property service
#   - L1 通过 /hardware/joint_command (JointCommand) 控制关节
#   - L1 通过 /control/debug topic (JSON) 清错误等
#   - L1 通过 /feedback/robot_server_state 发布状态

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "${SCRIPT_DIR}/../install/setup.bash"

# === 检查 Sudo 权限 ===
# 尝试使用默认密码验证，如果失败则提示用户手动输入
if echo "123456" | sudo -S -v >/dev/null 2>&1; then
    SUDO_RUN() { echo "123456" | sudo -S "$@"; }
else
    SUDO_RUN() { sudo "$@"; }
fi

# 打印使用说明
print_usage() {
    echo "Usage: ./set_l1.sh <command> [value]"
    echo ""
    echo "状态与日志:"
    echo "  echo            - 打印一次 robot server 状态 (/feedback/robot_server_state)"
    echo "  log             - 打印 robot server 日志 (~/log/robot_server.txt)"
    echo "  state           - 关节状态查看 GUI (script/joint_viewer.py)"
    echo "  watch           - 监视 EtherCAT 从站状态 (watch -n 0.1 ethercat sl)"
    echo "  bat             - 监视电池电量 (script/watch_bat_status.sh)"
    echo "  dc              - 读取 DC 寄存器 (script/read_DC_regs.sh)"
    echo ""
    echo "控制:"
    echo "  ctrl            - 关节控制 GUI (script/qt_joint_control.py)"
    echo "  hand            - 灵巧手控制 GUI (script/qt_hand.py)"
    echo "  viewer          - ROS 可视化 GUI (script/qt_ros_viewer.py)"
    echo "  zero            - 发送零位指令 (script/send_zero_command.py)"
    echo "  clear           - 清除错误 (向 /control/debug 发送 {\"clear_error\":true})"
    echo "  lock            - 锁定双臂 (script/arms_lock.sh)"
    echo "  unlock          - 解锁双臂 (script/arms_unlock.sh)"
    echo ""
    echo "动力学辨识:"
    echo "  dyn [args]      - 动力学参数辨识 GUI (script/qt_dyn_params_ident.py)"
    echo "  codegen         - 动力学代码生成 (script/RobotDynamicsCodeGenerator.py)"
    echo ""
    echo "服务管理:"
    echo "  start           - 启动 dexe-system.service"
    echo "  stop            - 停止 dexe-system.service"
    echo "  restart         - 重启 dexe-system.service"
    echo ""
    echo "EtherCAT:"
    echo "  ethercat run    - 运行 dexe_ethercat (../dexe_ethercat/run.sh)"
    echo "  ethercat build  - 编译 dexe_ethercat (../dexe_ethercat/build.sh)"
    echo "  ethercat git    - 查看 dexe_ethercat git log"
    echo "  clone ethercat  - 克隆 dexe_ethercat (dev) 到上级目录"
    echo "  clone interface - 克隆 dexe_interfaces (zyn/l1) 到上级目录"
    echo ""
    echo "配置:"
    echo "  config          - 编辑 l1_config.json (~/dexe_robot_server_data/l1_config.json)"
    echo "  zero_offset     - 编写零位偏移 (script/write_zero_offset.py)"
    echo ""
    echo "Examples:"
    echo "  ./set_l1.sh echo"
    echo "  ./set_l1.sh ctrl"
    echo "  ./set_l1.sh clear"
    echo "  ./set_l1.sh zero"
    echo "  ./set_l1.sh watch"
    echo "  ./set_l1.sh ethercat run"
    echo "  ./set_l1.sh config"
}

# 检查输入参数
if [ $# -lt 1 ]; then
    print_usage
    exit 1
fi

COMMAND=$1
VALUE=$2

# 根据输入分发命令
case $COMMAND in
    # === 状态与日志 ===
    echo)
        echo "Echoing robot server state..."
        ros2 topic echo /feedback/robot_server_state --field data --once
        ;;
    log)
        if [ -f ~/log/robot_server.txt ]; then
            cat ~/log/robot_server.txt
        else
            echo "Error: 日志文件 ~/log/robot_server.txt 不存在"
        fi
        ;;
    state)
        echo "Starting joint viewer..."
        python3 "${SCRIPT_DIR}/script/joint_viewer.py"
        ;;
    watch)
        echo "Watching EtherCAT status..."
        watch -n 0.1 ethercat sl
        ;;
    bat)
        echo "Watching battery status..."
        "${SCRIPT_DIR}/script/watch_bat_status.sh"
        ;;
    dc)
        echo "Reading DC registers..."
        "${SCRIPT_DIR}/script/read_DC_regs.sh"
        ;;

    # === 控制 ===
    ctrl)
        echo "Starting Joint Control GUI..."
        python3 "${SCRIPT_DIR}/script/qt_joint_control.py"
        ;;
    hand)
        echo "Starting Hand Control GUI..."
        python3 "${SCRIPT_DIR}/script/qt_hand.py"
        ;;
    viewer)
        echo "Starting ROS Viewer GUI..."
        python3 "${SCRIPT_DIR}/script/qt_ros_viewer.py"
        ;;
    zero)
        echo "Sending zero command..."
        python3 "${SCRIPT_DIR}/script/send_zero_command.py"
        ;;
    clear)
        echo "Clearing errors..."
        ros2 topic pub --once /control/debug std_msgs/msg/String "{data: '{\"clear_error\":true}'}"
        ;;
    lock)
        echo "Locking arms..."
        "${SCRIPT_DIR}/script/arms_lock.sh"
        ;;
    unlock)
        echo "Unlocking arms..."
        "${SCRIPT_DIR}/script/arms_unlock.sh"
        ;;

    # === 动力学辨识 ===
    dyn)
        echo "Starting Dynamic Parameter Identification GUI..."
        python3 "${SCRIPT_DIR}/script/qt_dyn_params_ident.py" "${@:2}"
        ;;
    codegen)
        echo "Starting Dynamics Code Generator..."
        python3 "${SCRIPT_DIR}/script/RobotDynamicsCodeGenerator.py"
        ;;

    # === 服务管理 ===
    start)
        echo "Starting dexe-system.service..."
        SUDO_RUN systemctl start dexe-system.service
        echo "Watching EtherCAT status..."
        watch -n 0.1 ethercat sl
        ;;
    stop)
        echo "Stopping dexe-system.service..."
        SUDO_RUN systemctl stop dexe-system.service
        ;;
    restart)
        echo "Restarting dexe-system.service..."
        SUDO_RUN systemctl restart dexe-system.service
        echo "Watching EtherCAT status..."
        watch -n 0.1 ethercat sl
        ;;

    # === EtherCAT ===
    ethercat)
        ETHERCAT_DIR="${SCRIPT_DIR}/../dexe_ethercat"
        if [ ! -d "$ETHERCAT_DIR" ]; then
            echo "Error: dexe_ethercat 目录不存在: $ETHERCAT_DIR"
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
                echo "Usage: ./set_l1.sh ethercat [run|build|git]"
                exit 1
                ;;
        esac
        ;;
    clone)
        PARENT_DIR="$(realpath "${SCRIPT_DIR}/..")"
        if [ "$VALUE" == "ethercat" ]; then
            echo "Cloning dexe_ethercat (dev) to ${PARENT_DIR}..."
            rm -rf "${PARENT_DIR}/dexe_ethercat"
            git clone -b dev http://192.168.3.16/dexembodied/dexe_ethercat.git "${PARENT_DIR}/dexe_ethercat"
        elif [ "$VALUE" == "interface" ]; then
            echo "Cloning dexe_interfaces (zyn/l1) to ${PARENT_DIR}..."
            rm -rf "${PARENT_DIR}/dexe_interfaces"
            git clone -b zyn/l1 http://192.168.3.16/dexembodied/dexe_interfaces.git "${PARENT_DIR}/dexe_interfaces"
        else
            echo "Usage: ./set_l1.sh clone [ethercat|interface]"
            exit 1
        fi
        ;;

    # === 配置 ===
    config)
        echo "Editing l1_config.json..."
        CONFIG_FILE=~/dexe_robot_server_data/l1_config.json
        if [ ! -f "$CONFIG_FILE" ]; then
            echo "提示: $CONFIG_FILE 不存在，首次启动 L1_server 时会自动创建默认配置"
        fi
        if [ -n "$DISPLAY" ]; then
            gedit "$CONFIG_FILE"
        else
            vim "$CONFIG_FILE"
        fi
        ;;
    zero_offset)
        echo "Running zero offset writer..."
        python3 "${SCRIPT_DIR}/script/write_zero_offset.py" "${@:2}"
        ;;

    # === 帮助 ===
    help|--help|-h)
        print_usage
        ;;

    *)
        echo "未知命令: $COMMAND"
        print_usage
        exit 1
        ;;
esac
