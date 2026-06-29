#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "robot_server_lib/utils.h"
#include "robot_server_sim.h"

int main(int argc, char* argv[]) {
    // 初始化配置文件: 默认配置取自 w1_server 的 share 目录, 用户配置在 ~/dexe_robot_server_data/
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    std::string cfg_file = "robot_config.yaml";
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / cfg_file;
    std::cout << "Using config file: " << robot_cfg_path << std::endl;

    std::filesystem::path user_cfg_path =
        std::filesystem::path(std::getenv("HOME")) / "dexe_robot_server_data" / cfg_file;
    if (!std::filesystem::exists(user_cfg_path.parent_path())) {
        std::filesystem::create_directories(user_cfg_path.parent_path());
        std::cout << "[INFO] Created data directory: " << user_cfg_path.parent_path() << std::endl;
    }
    robot_server_lib::utils::CheckConfig(robot_cfg_path, user_cfg_path);

    robot_server_lib::utils::SyncHardwareConfig(robot_cfg_path);
    if (std::filesystem::exists(user_cfg_path)) {
        robot_server_lib::utils::SyncHardwareConfig(user_cfg_path);
    }

    // 初始化RCLCPP
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_server_lib::RobotServer>(user_cfg_path, true));
    rclcpp::shutdown();

    return 0;
}
