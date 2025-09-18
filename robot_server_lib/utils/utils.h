#include <memory>

/**
 * @file utils.h
 * @brief 通用工具函数
 * @copyright Copyright (C) 2025 DexForce Technology Co., Ltd. All Rights Reserved.
 */
#pragma once

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_server_interfaces/action/move_robot.hpp>
#include <robot_server_interfaces/msg/streaming_command.hpp>
#include <robot_server_interfaces/srv/set_property.hpp>
#include <string>
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

template <typename VectorType>
std::ostringstream VectorToJsonArrayString(const VectorType &vec)
{
    std::ostringstream ss;
    ss << "[";
    for (int i = 0; i < vec.size(); ++i)
    {
        ss << vec[i];
        if (i != vec.size() - 1)
            ss << ",";
    }
    ss << "]";
    return ss;
}

PlannerRequest ParseSetPropertyRequest(const std::shared_ptr<robot_server_interfaces::srv::SetProperty::Request> request);
StreamingCommand ParseStreamingCommand(const std::shared_ptr<robot_server_interfaces::msg::StreamingCommand> msg);
PlannerRequest ParseMoveRequest(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_server_interfaces::action::MoveRobot>> action_handle);

bool MapArmToWholeBody(const RobotModel &robot_model, const Eigen::VectorXd &q, JointPositionDof &q_ndof, const MotionGroup &motion_group);

bool GetArmFromWholeBody(const RobotModel &robot_model, const JointPositionDof &q_ndof, Eigen::VectorXd &q, const MotionGroup &motion_group);

Eigen::VectorXd GetSensorWrench(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> &wrench_primary_ptr,
                                const std::shared_ptr<geometry_msgs::msg::WrenchStamped> &wrench_secondary_ptr);
}  // namespace utils
}  // namespace robot_server_lib