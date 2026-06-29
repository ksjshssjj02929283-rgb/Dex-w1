/**
 * @file utils.h
 * @brief 通用工具函数
 * @copyright Copyright (C) 2025 DexForce Technology Co., Ltd. All Rights Reserved.
 */
#pragma once
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <iomanip>
#include <limits>
#include <magic_enum.hpp>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_server_interfaces/action/move_robot.hpp>
#include <robot_server_interfaces/msg/streaming_command.hpp>
#include <robot_server_interfaces/srv/set_property.hpp>
#include <string>
#include <type_traits>
#include <vector>

#include "NanoLog.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "data_structure.h"

namespace robot_server_lib {
namespace utils {
RobotModel GetRobotModelFromConfig(std::filesystem::path robot_cfg_path);

// 配置文件管理: 以默认配置为基准同步/补齐用户配置, 合并外部硬件配置
void CheckConfig(const std::filesystem::path& default_cfg_path, const std::filesystem::path& user_cfg_path);
void SyncHardwareConfig(const std::filesystem::path& robot_cfg_path);

void SetMax(std::atomic_int64_t& maximum_value, std::int64_t value);

PoseQuaternion VectorXdToPoseQuaternion(const Eigen::VectorXd& vec);

PoseQuaternion StringToPoseQuaternion(const std::string& str);

Vector3d StringToVector3d(const std::string& str);

template <typename T>
inline std::size_t GetContainerSize(const T& c) {
    return static_cast<std::size_t>(c.size());
}

template <typename T, std::size_t N>
inline std::size_t GetContainerSize(const T (&)[N]) {
    return N;
}

template <typename VectorType>
std::ostringstream VectorToJsonArrayString(const VectorType& vec, int start_index, int count) {
    int total_size = static_cast<int>(GetContainerSize(vec));
    if (start_index < 0) start_index = 0;
    if (count < 0 || start_index + count > total_size) count = total_size - start_index;
    if (count < 0) count = 0;

    std::ostringstream ss;
    ss << "[";
    for (int i = 0; i < count; ++i) {
        int idx = start_index + i;
        using ElemT = std::decay_t<decltype(vec[idx])>;
        if constexpr (std::is_enum_v<ElemT>) {
            auto sv = magic_enum::enum_name(vec[idx]);
            ss << "\"" << (sv.empty() ? "Unknown" : sv.data()) << "\"";
        } else if constexpr (std::is_integral_v<ElemT>) {
            ss << static_cast<long long>(vec[idx]);
        } else if constexpr (std::is_convertible_v<ElemT, double>) {
            double v = static_cast<double>(vec[idx]);
            if (std::isfinite(v)) {
                ss << std::fixed << std::setprecision(5) << v;
            } else {
                ss << "null";
            }
        } else {
            ss << "\"" << vec[idx] << "\"";
        }
        if (i < count - 1) ss << ",";
    }
    ss << "]";
    return ss;
}

template <typename VectorType>
std::ostringstream VectorToJsonArrayString(const VectorType& vec) {
    std::ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        using ElemT = std::decay_t<decltype(vec[i])>;
        if constexpr (std::is_enum_v<ElemT>) {
            auto sv = magic_enum::enum_name(vec[i]);
            ss << "\"" << (sv.empty() ? "Unknown" : sv.data()) << "\"";
        } else if constexpr (std::is_integral_v<ElemT>) {
            ss << static_cast<long long>(vec[i]);
        } else if constexpr (std::is_convertible_v<ElemT, double>) {
            double v = static_cast<double>(vec[i]);
            if (std::isfinite(v)) {
                ss << std::fixed << std::setprecision(5) << v;
            } else {
                ss << "null";
            }
        } else {
            ss << "\"" << vec[i] << "\"";
        }
        if (i != vec.size() - 1) ss << ",";
    }
    ss << "]";
    return ss;
}

PlannerRequest ParseSetPropertyRequest(
    const std::shared_ptr<robot_server_interfaces::srv::SetProperty::Request> request);
StreamingCommand ParseStreamingCommand(const std::shared_ptr<robot_server_interfaces::msg::StreamingCommand> msg);
PlannerRequest ParseMoveRequest(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>> action_handle);

bool MapJointToWholeBody(const RobotModel& robot_model,
                         const Eigen::VectorXd& q,
                         JointPositionDof& q_ndof,
                         const MotionGroup& motion_group);

bool GetJointFromWholeBody(const RobotModel& robot_model,
                           const JointPositionDof& q_ndof,
                           Eigen::VectorXd& q,
                           const MotionGroup& motion_group);

// 根据 motion_group 获取对应的关节数量
int GetJointCount(const RobotModel& robot_model, const MotionGroup& motion_group);

Vector12d GetSensorWrench(const Vector6d& wrench_primary, const Vector6d& wrench_secondary);

// 浮点数近似相等判断 (绝对+相对容差)
inline bool FloatEqual(double a, double b, double tol = 1e-9) noexcept {
    if (a == b) return true;
    double diff = std::abs(a - b);
    double scale = std::max(std::abs(a), std::abs(b));
    return diff <= tol || diff <= tol * scale;
}

// 三向比较: -1 (<), 0 (≈), 1 (>)
inline int FloatCompare(double a, double b, double tol = 1e-9) noexcept {
    double diff = a - b;
    if (std::abs(diff) <= tol) return 0;
    return diff < 0 ? -1 : 1;
}

// 线性插值器：用于在高低频率控制线程间平滑过渡
// 例如：250Hz规划层到1kHz力控层的插值
class LinearInterpolator {
public:
    LinearInterpolator(int interpolation_steps);

    // 执行一次插值计算，返回插值后的位置
    // Call every control cycle (1kHz)
    VectorDof Interpolate(const VectorDof& target_position);

    bool TargetChanged(const VectorDof& target_position) const {
        for (int i = 0; i < target_position.size(); ++i) {
            if (!FloatEqual(target_position[i], target_position_[i])) {
                return true;
            }
        }
        return false;
    }

    // 重置插值器状态
    void Reset();

private:
    int interpolation_steps_;        // 插值步数 (例如 4 表示 250Hz->1kHz 的 4 个周期)
    VectorDof pre_target_position_;  // 插值起始位置，也记录上一次的目标位置
    VectorDof target_position_;      // 插值目标位置

    int counter_ = 0;      // 当前插值计数器
    bool inited_ = false;  // 是否已初始化
};
}  // namespace utils
}  // namespace robot_server_lib