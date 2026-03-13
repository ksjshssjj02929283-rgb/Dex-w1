#include <cerrno>
#include <cstring>
#include <iostream>
#include <pthread.h>
#include <sched.h>

#include "chassis_node.h"
#include "w1_server.h"

// 设置主线程实时优先级
static void SetupRealtimeThread(int priority) {
    pthread_t this_thread = pthread_self();
    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_setschedparam(this_thread, SCHED_FIFO, &param) != 0) {
        std::cerr << "pthread_setschedparam failed: " << std::strerror(errno) << std::endl;
    }
}

int main(int argc, char* argv[]) {
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");

    // 支持通过命令行参数选择配置文件
    std::string cfg_file = "robot.cfg";
    robot_server_lib::RobotMode robot_mode = robot_server_lib::RobotMode::W1;
    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "--a1" || arg == "a1") {
            robot_mode = robot_server_lib::RobotMode::A1;
        } else if (arg == "--arm" || arg == "arm") {
            robot_mode = robot_server_lib::RobotMode::Arm;
        }
    }

    const auto robot_cfg_path = std::filesystem::path(package_share_path) / cfg_file;
    std::cout << "Using config file: " << robot_cfg_path << std::endl;
    std::string mode_str;
    switch (robot_mode) {
        case robot_server_lib::RobotMode::Arm:
            mode_str = "Arm (7 DOF)";
            break;
        case robot_server_lib::RobotMode::A1:
            mode_str = "A1 (14 DOF)";
            break;
        case robot_server_lib::RobotMode::W1:
        default:
            mode_str = "W1 (20 DOF)";
            break;
    }
    std::cout << "Mode: " << mode_str << std::endl;

    rclcpp::init(argc, argv);

    // 启动节点
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node_server = std::make_shared<robot_server_lib::RobotServer>(robot_cfg_path, false, robot_mode);

    std::shared_ptr<chassis_node::ChassisNode> node_chassis;
    // W1 模式下启动底盘节点，A1/Arm 模式下不启动
    if (robot_mode == robot_server_lib::RobotMode::W1) {
        node_chassis = std::make_shared<chassis_node::ChassisNode>();
        executor.add_node(node_chassis);
    }

    executor.add_node(node_server);

    // 设置主线程实时优先级（用于 ROS 回调）
    SetupRealtimeThread(70);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
