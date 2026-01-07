
#include <cerrno>
#include <cstring>
#include <iostream>

#include "chassis_node.h"
#include "w1_server.h"

int main(int argc, char* argv[])
{
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.cfg";

    rclcpp::init(argc, argv);

    // 启动节点
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node_server = std::make_shared<robot_server_lib::RobotServer>(robot_cfg_path, false);
    auto node_chassis = std::make_shared<chassis_node::ChassisNode>();
    executor.add_node(node_server);
    executor.add_node(node_chassis);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
