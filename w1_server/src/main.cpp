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
    YAML::Node result = user_node;             // 复制用户节点作为基础
    if (!default_node.IsMap()) return result;  // 只有字典(Map)类型才需要向下遍历合并

    for (const auto& default_field : default_node) {
        std::string field_key = default_field.first.as<std::string>();
        std::string full_path = path.empty() ? field_key : path + "." + field_key;

        if (!result[field_key]) {
            // 用户配置中不存在该字段，将默认配置中的整个节点补充进去
            result[field_key] = default_field.second;
            modified = true;
            std::cout << "[WARN] Added missing field: " << full_path << std::endl;
        } else if (default_field.second.IsMap()) {
            // 字段存在且为字典类型，递归合并其内部的子字段
            result[field_key] = MergeYamlNode(result[field_key], default_field.second, full_path, modified);
        }
    }
    return result;
}

static void CheckConfig(const std::filesystem::path& default_cfg_path, const std::filesystem::path& user_cfg_path) {
    if (!std::filesystem::exists(user_cfg_path)) {
        // 用户配置文件不存在，执行完全拷贝
        if (!std::filesystem::copy_file(default_cfg_path, user_cfg_path,
                                        std::filesystem::copy_options::overwrite_existing)) {
            std::cerr << "[ERROR] Failed to copy default config file: " << user_cfg_path << std::endl;
            std::exit(1);
        }
        std::cout << "[INFO] Created default config file: " << user_cfg_path << std::endl;
    } else {
        // 加载默认配置和用户修改过的配置进行对比合并
        YAML::Node default_config = YAML::LoadFile(default_cfg_path.string());
        YAML::Node user_config = YAML::LoadFile(user_cfg_path.string());

        // 检查配置版本，如果默认配置版本领先，则直接覆盖历史用户配置
        std::string default_version =
            default_config["config_version"] ? default_config["config_version"].as<std::string>() : "";
        std::string user_version = user_config["config_version"] ? user_config["config_version"].as<std::string>() : "";

        // 字符串比较版本号，假设版本格式为 "vXX"，直接比较字符串即可判断版本先后, 如果用户版本不存在则视为过旧需要覆盖
        if (default_version > user_version) {
            std::cout << "[INFO] Default config version (" << default_version << ") is newer than user config version ("
                      << (user_version.empty() ? "none" : user_version) << "). Overwriting user config." << std::endl;
            if (!std::filesystem::copy_file(default_cfg_path, user_cfg_path,
                                            std::filesystem::copy_options::overwrite_existing)) {
                std::cerr << "[ERROR] Failed to overwrite config file: " << user_cfg_path << std::endl;
                std::exit(1);
            }
            // 覆盖后重新加载最新的配置内容，以便于继续执行后续正常的一致性动作
            user_config = YAML::LoadFile(user_cfg_path.string());
        }

        bool modified = false;

        for (const auto& default_node : default_config) {
            std::string key = default_node.first.as<std::string>();

            if (!user_config[key]) {
                // 顶层字段缺失，补充整个节点
                user_config[key] = default_node.second;
                modified = true;
                std::cout << "[WARN] Added missing field: " << key << std::endl;
            } else if (default_node.second.IsMap()) {
                // 顶层字典类型字段存在，但内部二级字段可能缺失，遍历子字典处理
                for (const auto& sub_node : default_node.second) {
                    std::string sub_key = sub_node.first.as<std::string>();
                    if (!user_config[key][sub_key]) {
                        user_config[key][sub_key] = sub_node.second;
                        modified = true;
                        std::cout << "[WARN] Added missing field: " << key << "." << sub_key << std::endl;
                    }
                }
            } else if (default_node.second.IsSequence() && key == "joints") {
                // 特殊处理 joints 参数，joints 是一个数组，每个元素包含各个关节的具体配置
                for (size_t i = 0; i < default_node.second.size(); ++i) {
                    if (i >= user_config[key].size()) {
                        // 如果用户配置的关节数少于默认配置的关节数，将新增的关节补充到末尾
                        user_config[key].push_back(default_node.second[i]);
                        modified = true;
                        std::cout << "[WARN] Added missing joint at index: " << i << std::endl;
                    } else {
                        // 关节参数虽然存在但也可能缺失某些下级属性，调用合并函数进行属性补齐
                        user_config[key][i] = MergeYamlNode(user_config[key][i], default_node.second[i],
                                                            "joints[" + std::to_string(i) + "]", modified);
                    }
                }
            }
        }

        if (modified) {
            // 如果上述检查补齐了任何字段，将完整的用户配置覆盖写回原文件
            std::ofstream fout(user_cfg_path.string());
            fout << user_config;
            fout.close();
            std::cout << "[INFO] Updated config file with missing fields: " << user_cfg_path << std::endl;
        }
    }
}

static void SyncHardwareConfig(const std::filesystem::path& robot_cfg_path) {
    const std::string external_cfg_path = std::string(std::getenv("HOME")) + "/.cache/dexe_data/config.yaml";
    if (!std::filesystem::exists(external_cfg_path)) {
        std::cout << "[INFO] External hardware config not found, skipping sync." << std::endl;
        return;
    }

    try {
        YAML::Node external_config = YAML::LoadFile(external_cfg_path);
        YAML::Node robot_config = YAML::LoadFile(robot_cfg_path.string());

        bool modified = false;
        for (auto const& it : external_config) {
            std::string key = it.first.as<std::string>();
            robot_config[key] = it.second;
            modified = true;
        }

        if (modified) {
            std::ofstream fout(robot_cfg_path.string());
            fout << robot_config;
            fout.close();
            std::cout << "[INFO] Synchronized all hardware fields from " << external_cfg_path << " to "
                      << robot_cfg_path << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to sync hardware config: " << e.what() << std::endl;
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

    SyncHardwareConfig(robot_cfg_path);
    if (std::filesystem::exists(user_cfg_path)) {
        SyncHardwareConfig(user_cfg_path);
    }
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
