#include "l1_server.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // 启动节点
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node_server = std::make_shared<robot_server_lib::RobotServer>();
    executor.add_node(node_server);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
