#include "utils.h"

#include <yaml-cpp/yaml.h>

#include <iostream>

namespace robot_server_lib {
namespace utils {

std::ostringstream RobotStatuswordToJsonString(const int& status_word) {
    std::ostringstream ss;
    switch (status_word) {
        case 0:
            ss << "\"NOT_READY\"";
            break;
        case 1:
            ss << "\"READY\"";
            break;
        case 2:
            ss << "\"ENABLED\"";
            break;
        case 4:
            ss << "\"MOTOR_FAULT\"";
            break;
        case 8:
            ss << "\"MOTOR_QUICK_STOP\"";
            break;
        default:
            ss << status_word;
            break;
    }
    return ss;
}

std::ostringstream RobotControlwordToJsonString(const int& control_word) {
    std::ostringstream ss;
    std::vector<std::string> flags;

    if (control_word & CTRLWORD_MOTOR_SWITCH_ON) flags.push_back("\"MOTOR_SWITCH_ON\"");
    if (control_word & CTRLWORD_MOTOR_ENABLE) flags.push_back("\"MOTOR_ENABLE\"");
    if (control_word & CTRLWORD_MOTOR_QUICK_STOP) flags.push_back("\"MOTOR_QUICK_STOP\"");
    if (control_word & CTRLWORD_MOTOR_CLEAR_FAULT) flags.push_back("\"MOTOR_CLEAR_FAULT\"");
    if (control_word & CTRLWORD_MOTOR_CST_MODE) flags.push_back("\"MOTOR_CST_MODE\"");

    if (!flags.empty()) {
        ss << "[";
        for (size_t i = 0; i < flags.size(); ++i) {
            if (i > 0) ss << ",";
            ss << flags[i];
        }
        ss << "]";
    } else {
        ss << control_word;
    }

    return ss;
}

}  // namespace utils

namespace {

// 解析长度为 3 的 YAML 序列为 Eigen::Vector3d，缺失或格式错误时返回默认值
Eigen::Vector3d ParseVec3(const YAML::Node& node, Eigen::Vector3d def = Eigen::Vector3d::Zero()) {
    if (!node || !node.IsSequence() || node.size() < 3) {
        return def;
    }
    return Eigen::Vector3d(node[0].as<double>(def[0]), node[1].as<double>(def[1]), node[2].as<double>(def[2]));
}

// 解析单侧（左/右）踝关节参数
AnkleParams ParseAnkleParams(const YAML::Node& node) {
    AnkleParams params;
    if (!node || !node.IsMap()) {
        return params;
    }
    params.r_roll_center = ParseVec3(node["r_roll_center"]);
    params.r_pitch_center = ParseVec3(node["r_pitch_center"]);

    params.r_a1 = ParseVec3(node["r_a1"]);
    params.r_c1 = ParseVec3(node["r_c1"]);
    params.l_bar1 = node["l_bar1"].as<double>(0.0);
    params.l_rod1 = node["l_rod1"].as<double>(0.0);
    params.alpha_1_rad = node["alpha_1_rad"].as<double>(0.0);

    params.r_a2 = ParseVec3(node["r_a2"]);
    params.r_c2 = ParseVec3(node["r_c2"]);
    params.l_bar2 = node["l_bar2"].as<double>(0.0);
    params.l_rod2 = node["l_rod2"].as<double>(0.0);
    params.alpha_2_rad = node["alpha_2_rad"].as<double>(0.0);
    return params;
}

}  // namespace

L1Config LoadConfig(const std::string& config_path) {
    L1Config config;
    try {
        YAML::Node node = YAML::LoadFile(config_path);
        if (node["run_mode"]) {
            config.run_mode = node["run_mode"].as<int>();
        }
        config.robot_type = node["robot_type"].as<std::string>("");
        config.version = node["version"].as<std::string>("");

        // 踝关节参数位于 "<robot_type>_<version>" 子节点下，如 "l1_v01"
        const std::string profile_key = config.robot_type + "_" + config.version;
        const YAML::Node& ankle_node = node[profile_key]["ankle_params"];
        if (ankle_node) {
            config.left_ankle_params = ParseAnkleParams(ankle_node["left"]);
            config.right_ankle_params = ParseAnkleParams(ankle_node["right"]);
        } else {
            std::cerr << "Warning: ankle params not found under '" << profile_key << ".ankle_params' in config "
                      << config_path << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config from " << config_path << ": " << e.what() << std::endl;
    }
    return config;
}

}  // namespace robot_server_lib
