#pragma once

#include <bitset>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <type_traits>
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

// 基础类型别名
using Int64 = std::int64_t;

// 控制周期常量
constexpr int torque_ctrl_dt_micro = 1000;    // 1kHz
constexpr int publish_dt_micro = 10000;       // 100Hz

namespace utils {

// 获取容器大小（支持 STL 容器和原生数组）
template <typename T>
inline std::size_t GetContainerSize(const T &c) {
    return static_cast<std::size_t>(c.size());
}

template <typename T, std::size_t N>
inline std::size_t GetContainerSize(const T (&)[N]) {
    return N;
}

template <typename VectorType>
std::ostringstream MotorStatusToJsonArrayString(const VectorType &vec, int start_index, int end_index) {
    int total_size = static_cast<int>(GetContainerSize(vec));
    if (start_index < 0) start_index = 0;
    if (end_index < 0 || end_index > total_size) end_index = total_size;
    if (start_index > end_index) start_index = end_index;

    std::ostringstream ss;
    ss << "[";
    for (int i = start_index; i < end_index; ++i) {
        using ElemT = std::decay_t<decltype(vec[i])>;
        if constexpr (std::is_integral_v<ElemT>) {
            auto data = static_cast<long>(vec[i]);
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
        if (i < end_index - 1) ss << ",";
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
std::ostringstream MotorErrorcodeToJsonArrayString(const VectorType &vec, int start_index, int end_index) {
    int total_size = static_cast<int>(GetContainerSize(vec));
    if (start_index < 0) start_index = 0;
    if (end_index < 0 || end_index > total_size) end_index = total_size;
    if (start_index > end_index) start_index = end_index;

    std::ostringstream ss;
    ss << "[";
    for (int i = start_index; i < end_index; ++i) {
        using ElemT = std::decay_t<decltype(vec[i])>;
        if constexpr (std::is_integral_v<ElemT>) {
            auto data = static_cast<long>(vec[i]);
            switch (data) {
                default:
                    ss << data;
                    break;
            }
        } else {
            ss << "invalid data";
        }
        if (i < end_index - 1) ss << ",";
    }
    ss << "]";
    return ss;
}

// 将数值数组序列化为 JSON 数组字符串（带起止索引）
template <typename VectorType>
std::ostringstream VectorToJsonArrayString(const VectorType &vec, int start_index, int end_index) {
    int total_size = static_cast<int>(GetContainerSize(vec));
    if (start_index < 0) start_index = 0;
    if (end_index < 0 || end_index > total_size) end_index = total_size;
    if (start_index > end_index) start_index = end_index;

    std::ostringstream ss;
    ss << "[";
    for (int i = start_index; i < end_index; ++i) {
        using ElemT = std::decay_t<decltype(vec[i])>;
        if constexpr (std::is_integral_v<ElemT>) {
            ss << static_cast<long long>(vec[i]);
        } else {
            double v = static_cast<double>(vec[i]);
            if (std::isfinite(v)) {
                ss << std::fixed << std::setprecision(4) << v;
            } else {
                ss << "null";
            }
        }
        if (i < end_index - 1) ss << ",";
    }
    ss << "]";
    return ss;
}

}  // namespace utils
}  // namespace robot_server_lib