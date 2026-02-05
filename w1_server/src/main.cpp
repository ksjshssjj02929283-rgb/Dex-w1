#include <cerrno>
#include <cstring>
#include <iostream>

#include "chassis_node.h"
#include "w1_server.h"

int main(int argc, char* argv[]) {
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");

    // 支持通过命令行参数选择配置文件
    std::string cfg_file = "robot.cfg";
    bool is_a1_mode = false;
    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "--a1" || arg == "a1") {
            is_a1_mode = true;
        }
    }

    const auto robot_cfg_path = std::filesystem::path(package_share_path) / cfg_file;
    std::cout << "Using config file: " << robot_cfg_path << std::endl;
    std::cout << "Mode: " << (is_a1_mode ? "A1 (14 DOF)" : "W1 (20 DOF)") << std::endl;

    rclcpp::init(argc, argv);

    // 启动节点
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node_server = std::make_shared<robot_server_lib::RobotServer>(robot_cfg_path, false, is_a1_mode);

    std::shared_ptr<chassis_node::ChassisNode> node_chassis;
    // A1 模式下不启动底盘节点
    if (!is_a1_mode) {
        node_chassis = std::make_shared<chassis_node::ChassisNode>();
        executor.add_node(node_chassis);
    }

    executor.add_node(node_server);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
