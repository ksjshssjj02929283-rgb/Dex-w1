#include <pthread.h>
#include <sched.h>
#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "chassis_node.h"
#include "w1_server.h"

static YAML::Node MergeYamlNode(const YAML::Node& user_node,
                                const YAML::Node& default_node,
                                const std::string& path,
                                bool& modified) {
    YAML::Node result = user_node;
    if (!default_node.IsMap()) return result;

    for (const auto& default_field : default_node) {
        std::string field_key = default_field.first.as<std::string>();
        std::string full_path = path.empty() ? field_key : path + "." + field_key;

        if (!result[field_key]) {
            result[field_key] = default_field.second;
            modified = true;
            std::cout << "[WARN] Added missing field: " << full_path << std::endl;
        } else if (default_field.second.IsMap()) {
            result[field_key] = MergeYamlNode(result[field_key], default_field.second, full_path, modified);
        }
    }
    return result;
}

static void CheckConfig(const std::filesystem::path& default_cfg_path, const std::filesystem::path& user_cfg_path) {
    if (!std::filesystem::exists(user_cfg_path)) {
        if (!std::filesystem::copy_file(default_cfg_path, user_cfg_path,
                                        std::filesystem::copy_options::overwrite_existing)) {
            std::cerr << "[ERROR] Failed to copy default config file: " << user_cfg_path << std::endl;
            std::exit(1);
        }
        std::cout << "[INFO] Created default config file: " << user_cfg_path << std::endl;
    } else {
        YAML::Node default_config = YAML::LoadFile(default_cfg_path.string());
        YAML::Node user_config = YAML::LoadFile(user_cfg_path.string());
        bool modified = false;

        for (const auto& default_node : default_config) {
            std::string key = default_node.first.as<std::string>();

            if (!user_config[key]) {
                user_config[key] = default_node.second;
                modified = true;
                std::cout << "[WARN] Added missing field: " << key << std::endl;
            } else if (default_node.second.IsMap()) {
                user_config[key] = MergeYamlNode(user_config[key], default_node.second, key, modified);
            } else if (default_node.second.IsSequence() && key == "joints") {
                for (size_t i = 0; i < default_node.second.size(); ++i) {
                    if (i >= user_config[key].size()) {
                        user_config[key].push_back(default_node.second[i]);
                        modified = true;
                        std::cout << "[WARN] Added missing joint at index: " << i << std::endl;
                    } else {
                        user_config[key][i] = MergeYamlNode(user_config[key][i], default_node.second[i],
                                                            "joints[" + std::to_string(i) + "]", modified);
                    }
                }
            }
        }

        if (modified) {
            std::ofstream fout(user_cfg_path.string());
            fout << user_config;
            fout.close();
            std::cout << "[INFO] Updated config file with missing fields: " << user_cfg_path << std::endl;
        }
    }
}

static void SetupRealtimeThread(int priority) {
    pthread_t this_thread = pthread_self();
    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_setschedparam(this_thread, SCHED_FIFO, &param) != 0) {
        std::cerr << "pthread_setschedparam failed: " << std::strerror(errno) << std::endl;
    }
}

int main(int argc, char* argv[]) {
    // 初始化配置文件
    const auto package_share_path = ament_index_cpp::get_package_share_directory("w1_server");
    std::string cfg_file = "robot_config.yaml";
    const auto robot_cfg_path = std::filesystem::path(package_share_path) / cfg_file;
    std::cout << "Using config file: " << robot_cfg_path << std::endl;

    std::filesystem::path user_cfg_path;
    user_cfg_path = std::filesystem::path(std::getenv("HOME")) / "dexe_robot_server_data" / cfg_file;

    if (!std::filesystem::exists(user_cfg_path.parent_path())) {
        std::filesystem::create_directories(user_cfg_path.parent_path());
        std::cout << "[INFO] Created data directory: " << user_cfg_path.parent_path() << std::endl;
    }

    CheckConfig(robot_cfg_path, user_cfg_path);
    std::cout << "[INFO] Mode: 自动检测模式（根据实际机器人关节配置）" << std::endl;
    // 初始化RCLCPP
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node_server = std::make_shared<robot_server_lib::RobotServer>(user_cfg_path, false);
    std::shared_ptr<chassis_node::ChassisNode> node_chassis;
    node_chassis = std::make_shared<chassis_node::ChassisNode>();
    executor.add_node(node_chassis);
    executor.add_node(node_server);
    SetupRealtimeThread(70);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
