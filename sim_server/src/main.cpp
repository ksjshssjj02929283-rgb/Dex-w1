#include "robot_server.hpp"

int main(int argc, char* argv[])
{
    std::filesystem::create_directories("log/");
    nanolog::initialize(nanolog::GuaranteedLogger(), "log/", "robot_server_sim", 10);
    nanolog::set_log_level(nanolog::LogLevel::INFO);  // options: DEBUG, INFO, WARN, CRIT 高频过程数据打印设置为DEBUG

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotServer>());
    rclcpp::shutdown();

    return 0;
}
