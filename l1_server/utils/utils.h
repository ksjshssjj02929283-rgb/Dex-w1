#pragma once

#include <Eigen/Dense>
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
constexpr int torque_ctrl_dt_micro = 1000;  // 1kHz
constexpr int publish_dt_micro = 10000;     // 100Hz

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

std::ostringstream RobotStatuswordToJsonString(const int &status_word);

std::ostringstream RobotControlwordToJsonString(const int &control_word);

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

// L1 配置结构（从 l1_config.yaml 加载）

// 单侧（左/右）踝关节并联机构参数
struct AnkleParams {
    Eigen::Vector3d r_roll_center{0.0, 0.0, 0.0};   // roll 旋转中心（基坐标系）
    Eigen::Vector3d r_pitch_center{0.0, 0.0, 0.0};  // pitch 旋转中心（基坐标系）

    bool is_left_motor1 = true;           // 执行器1 电机物理位置是否为左电机
    Eigen::Vector3d r_a1{0.0, 0.0, 0.0};  // 执行器1 A点坐标
    Eigen::Vector3d r_c1{0.0, 0.0, 0.0};  // 执行器1 C点初始坐标
    double l_bar1 = 0.0;                  // 执行器1 法兰杆长度
    double l_rod1 = 0.0;                  // 执行器1 连杆长度
    double alpha_1_rad = 0.0;             // 执行器1 零位偏移

    bool is_left_motor2 = false;          // 执行器2 电机物理位置是否为左电机
    Eigen::Vector3d r_a2{0.0, 0.0, 0.0};  // 执行器2 A点坐标
    Eigen::Vector3d r_c2{0.0, 0.0, 0.0};  // 执行器2 C点初始坐标
    double l_bar2 = 0.0;                  // 执行器2 法兰杆长度
    double l_rod2 = 0.0;                  // 执行器2 连杆长度
    double alpha_2_rad = 0.0;             // 执行器2 零位偏移
};

struct L1Config {
    int run_mode = 0;                // 上肢控制模式: 0=位置模式, 1=力矩模式(CST)
    std::string robot_type;          // 机器人型号，如 "l1"
    std::string version;             // 版本号，如 "v01"
    AnkleParams left_ankle_params;   // 左侧并联踝关节几何参数
    AnkleParams right_ankle_params;  // 右侧并联踝关节几何参数
};

// 从 YAML 文件加载配置
L1Config LoadConfig(const std::string &config_path);

}  // namespace robot_server_lib