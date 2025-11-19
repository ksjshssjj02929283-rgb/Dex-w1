/**
 * @file utils.h
 * @brief 通用工具函数
 * @copyright Copyright (C) 2025 DexForce Technology Co., Ltd. All Rights Reserved.
 */
#pragma once
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <iomanip>
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

using json = nlohmann::json;

namespace robot_server_lib
{
namespace utils
{
RobotModel GetRobotModelFromConfig(std::filesystem::path robot_cfg_path);

void SetMax(std::atomic_int64_t &maximum_value, std::int64_t value);

PoseQuaternion VectorXdToPoseQuaternion(const Eigen::VectorXd &vec);

PoseQuaternion StringToPoseQuaternion(const std::string &str);

template <typename T>
inline std::size_t GetContainerSize(const T &c)
{
    return static_cast<std::size_t>(c.size());
}

template <typename T, std::size_t N>
inline std::size_t GetContainerSize(const T (&)[N])
{
    return N;
}

template <typename VectorType>
std::ostringstream VectorToJsonArrayString(const VectorType &vec, int start_index, int end_index)
{
    int total_size = static_cast<int>(GetContainerSize(vec));
    if (start_index < 0)
        start_index = 0;
    if (end_index < 0 || end_index > total_size)
        end_index = total_size;
    if (start_index > end_index)
        start_index = end_index;

    std::ostringstream ss;
    ss << "[";
    for (int i = start_index; i < end_index; ++i)
    {
        using ElemT = std::decay_t<decltype(vec[i])>;
        if constexpr (std::is_integral_v<ElemT>)
        {
            ss << static_cast<long long>(vec[i]);
        }
        else
        {
            double v = static_cast<double>(vec[i]);
            if (std::isfinite(v))
            {
                ss << std::fixed << std::setprecision(4) << v;
            }
            else
            {
                ss << "null";
            }
        }
        if (i < end_index - 1)
            ss << ",";
    }
    ss << "]";
    return ss;
}

template <typename VectorType>
std::ostringstream VectorToJsonArrayString(const VectorType &vec)
{
    std::ostringstream ss;
    ss << "[";
    for (int i = 0; i < vec.size(); ++i)
    {
        using ElemT = std::decay_t<decltype(vec[i])>;
        if constexpr (std::is_integral_v<ElemT>)
        {
            ss << static_cast<long long>(vec[i]);
        }
        else
        {
            double v = static_cast<double>(vec[i]);
            if (std::isfinite(v))
            {
                ss << std::fixed << std::setprecision(4) << v;
            }
            else
            {
                ss << "null";
            }
        }
        if (i != vec.size() - 1)
            ss << ",";
    }
    ss << "]";
    return ss;
}

PlannerRequest ParseSetPropertyRequest(const std::shared_ptr<robot_server_interfaces::srv::SetProperty::Request> request);
StreamingCommand ParseStreamingCommand(const std::shared_ptr<robot_server_interfaces::msg::StreamingCommand> msg);
PlannerRequest ParseMoveRequest(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>> action_handle);

bool MapJointToWholeBody(const RobotModel &robot_model, const Eigen::VectorXd &q, JointPositionDof &q_ndof, const MotionGroup &motion_group);

bool GetJointFromWholeBody(const RobotModel &robot_model, const JointPositionDof &q_ndof, Eigen::VectorXd &q, const MotionGroup &motion_group);

Vector12d GetSensorWrench(const Vector6d &wrench_primary, const Vector6d &wrench_secondary);
}  // namespace utils
}  // namespace robot_server_lib