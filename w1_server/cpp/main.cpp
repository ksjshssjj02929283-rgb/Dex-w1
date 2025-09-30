#include "robot_server.h"

int main(int argc, char* argv[])
{
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / "robot.cfg";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_server_lib::RobotServer>(robot_cfg_path, false));
    rclcpp::shutdown();

    return 0;
}
