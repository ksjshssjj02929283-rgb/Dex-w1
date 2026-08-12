#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "robot_server_lib/utils.h"
#include "robot_server_sim.h"

int main(int argc, char* argv[]) {
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    std::string cfg_file = "robot_config.yaml";
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / cfg_file;

    // 只读用户配置, 不写不创建; 不存在则回退默认配置
    auto user_cfg_path = std::filesystem::path(std::getenv("HOME")) / "dexe_robot_server_data" / cfg_file;
    std::filesystem::path cfg_path;
    if (std::filesystem::exists(user_cfg_path)) {
        cfg_path = user_cfg_path;
    } else {
        cfg_path = robot_cfg_path;
    }
    std::cout << "Using config file: " << cfg_path << std::endl;

    // 初始化RCLCPP
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_server_lib::RobotServer>(cfg_path, true));
    rclcpp::shutdown();

    return 0;
}
