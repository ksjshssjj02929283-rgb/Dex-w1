#pragma once

#include <bitset>
#include <nlohmann/json.hpp>
#include <sstream>
#include <vector>

using json = nlohmann::json;

#define CTRLWORD_MOTOR_SWITCH_ON (1U << 0)
#define CTRLWORD_MOTOR_ENABLE (1U << 1)
#define CTRLWORD_MOTOR_QUICK_STOP (1U << 2)
#define CTRLWORD_MOTOR_CLEAR_FAULT (1U << 3)
#define CTRLWORD_MOTOR_CST_MODE (1U << 4)

#define STATUSWORD_MOTOR_NOT_READY (0)
#define STATUSWORD_MOTOR_READY (1U << 0)
#define STATUSWORD_MOTOR_ENABLED (1U << 1)
#define STATUSWORD_MOTOR_FAULT (1U << 2)
#define STATUSWORD_MOTOR_QUICK_STOP (1U << 3)

namespace robot_server_lib {
namespace utils {

template <typename VectorType>
std::ostringstream MotorStatusToJsonArrayString(const VectorType &vec, int start_index, int count) {
    int total_size = static_cast<int>(GetContainerSize(vec));
    if (start_index < 0) start_index = 0;
    if (count < 0 || start_index + count > total_size) count = total_size - start_index;
    if (count < 0) count = 0;

    std::ostringstream ss;
    ss << "[";
    for (int i = 0; i < count; ++i) {
        int idx = start_index + i;
        using ElemT = std::decay_t<decltype(vec[idx])>;
        if constexpr (std::is_integral_v<ElemT>) {
            auto data = static_cast<long>(vec[idx]);
            switch (data) {
                case 1:
                    ss << "\"INIT\"";
                    break;
                case 2:
                    ss << "\"PREOP\"";
                    break;
                case 4:
                    ss << "\"SAFEOP\"";
                    break;
                case 8:
                    ss << "\"OP\"";
                    break;
                default:
                    ss << data;
                    break;
            }
        } else {
            ss << "invalid data";
        }
        if (i < count - 1) ss << ",";
    }
    ss << "]";
    return ss;
}

inline std::ostringstream RobotStatuswordToJsonString(const int &status_word) {
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

inline std::ostringstream RobotControlwordToJsonString(const int &control_word) {
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

template <typename VectorType>
std::ostringstream MotorErrorcodeToJsonArrayString(const VectorType &vec, int start_index, int count) {
    int total_size = static_cast<int>(GetContainerSize(vec));
    if (start_index < 0) start_index = 0;
    if (count < 0 || start_index + count > total_size) count = total_size - start_index;
    if (count < 0) count = 0;

    std::ostringstream ss;
    ss << "[";
    for (int i = 0; i < count; ++i) {
        int idx = start_index + i;
        using ElemT = std::decay_t<decltype(vec[idx])>;
        if constexpr (std::is_integral_v<ElemT>) {
            auto data = static_cast<long>(vec[idx]);
            switch (data) {
                default:
                    ss << data;
                    break;
            }
        } else {
            ss << "invalid data";
        }
        if (i < count - 1) ss << ",";
    }
    ss << "]";
    return ss;
}

}  // namespace utils
}  // namespace robot_server_lib