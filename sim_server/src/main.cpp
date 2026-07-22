#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "robot_server_lib/utils.h"
#include "robot_server_sim.h"

int main(int argc, char* argv[]) {
    // sim_server 直接读取默认配置, 不读写用户配置文件, 避免与 w1_server 并发写冲突
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    std::string cfg_file = "robot_config.yaml";
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / cfg_file;
    std::cout << "Using config file: " << robot_cfg_path << std::endl;

    // 初始化RCLCPP
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_server_lib::RobotServer>(robot_cfg_path, true));
    rclcpp::shutdown();

    return 0;
}
