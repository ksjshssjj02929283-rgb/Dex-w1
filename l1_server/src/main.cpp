#include "l1_server.h"

int main(int argc, char* argv[]) {
    // 固定路径为 ~/log
    std::filesystem::path home = std::filesystem::path(std::getenv("HOME"));
    std::filesystem::create_directories(home / "log");
    nanolog::initialize(nanolog::GuaranteedLogger(), (home / "log/").string(), "robot_server", 10, 10);
    nanolog::set_log_level(nanolog::LogLevel::INFO);
    std::cout << "Logs directory: " << ((home / "log").string() + "/robot_server.txt") << std::endl;
    // 启动节点
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node_server = std::make_shared<robot_server_lib::RobotServer>();
    executor.add_node(node_server);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
