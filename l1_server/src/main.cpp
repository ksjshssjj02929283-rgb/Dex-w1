#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "l1_server.h"

// 检查并合并配置文件：用户配置优先，缺少字段从默认配置补充
static void CheckConfig(const std::filesystem::path& default_cfg_path, const std::filesystem::path& user_cfg_path) {
    // 确保用户配置目录存在
    if (!std::filesystem::exists(user_cfg_path.parent_path())) {
        std::filesystem::create_directories(user_cfg_path.parent_path());
    }

    if (!std::filesystem::exists(user_cfg_path)) {
        // 用户配置不存在，直接拷贝默认配置
        std::filesystem::copy_file(default_cfg_path, user_cfg_path, std::filesystem::copy_options::overwrite_existing);
        std::cout << "[INFO] Created default config: " << user_cfg_path << std::endl;
        return;
    }

    // 用户配置存在，检查配置版本；默认版本领先则整体覆盖用户配置
    YAML::Node default_config = YAML::LoadFile(default_cfg_path.string());
    YAML::Node user_config = YAML::LoadFile(user_cfg_path.string());

    std::string default_version =
        default_config["config_version"] ? default_config["config_version"].as<std::string>() : "";
    std::string user_version =
        user_config["config_version"] ? user_config["config_version"].as<std::string>() : "";

    // 版本格式 "vXX"，字符串比较即可判断先后；用户版本缺失视为过旧需覆盖
    if (default_version > user_version) {
        std::cout << "[INFO] Default config version (" << default_version
                  << ") is newer than user config version ("
                  << (user_version.empty() ? "none" : user_version) << "). Overwriting user config." << std::endl;
        std::filesystem::copy_file(default_cfg_path, user_cfg_path, std::filesystem::copy_options::overwrite_existing);
        user_config = YAML::LoadFile(user_cfg_path.string());
    }

    bool modified = false;

    for (const auto& default_kv : default_config) {
        const std::string key = default_kv.first.as<std::string>();
        if (!user_config[key]) {
            user_config[key] = default_kv.second;
            modified = true;
            std::cout << "[WARN] Added missing field: " << key << std::endl;
        }
    }

    if (modified) {
        std::ofstream fout(user_cfg_path);
        fout << user_config;
        std::cout << "[INFO] Updated config with missing fields: " << user_cfg_path << std::endl;
    }
}

int main(int argc, char* argv[]) {
    // 配置文件路径
    const auto package_share_path = ament_index_cpp::get_package_share_directory("l1_server");
    const auto default_cfg_path = std::filesystem::path(package_share_path) / "l1_config.yaml";
    const auto user_cfg_dir = std::filesystem::path(std::getenv("HOME")) / "dexe_robot_server_data";
    const auto user_cfg_path = user_cfg_dir / "l1_config.yaml";

    CheckConfig(default_cfg_path, user_cfg_path);

    // 日志
    std::filesystem::path home = std::filesystem::path(std::getenv("HOME"));
    std::filesystem::create_directories(home / "log");
    nanolog::initialize(nanolog::GuaranteedLogger(), (home / "log/").string(), "robot_server", 10, 10);
    nanolog::set_log_level(nanolog::LogLevel::INFO);
    std::cout << "Logs directory: " << ((home / "log").string() + "/robot_server.txt") << std::endl;

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node_server = std::make_shared<robot_server_lib::RobotServer>(user_cfg_path.string());
    executor.add_node(node_server);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
