#include "utils.h"

#include <iostream>

#include <yaml-cpp/yaml.h>

namespace robot_server_lib {
namespace utils {

std::ostringstream RobotStatuswordToJsonString(const int &status_word) {
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

std::ostringstream RobotControlwordToJsonString(const int &control_word) {
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

L1Config LoadConfig(const std::string& config_path) {
    L1Config config;
    try {
        YAML::Node node = YAML::LoadFile(config_path);
        if (node["run_mode"]) {
            config.run_mode = node["run_mode"].as<int>();
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config from " << config_path << ": " << e.what() << std::endl;
    }
    return config;
}

}  // namespace robot_server_lib
