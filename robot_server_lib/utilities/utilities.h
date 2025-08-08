#pragma once

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "NanoLog.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "data_structure.h"
using json = nlohmann::json;

namespace robot_server_lib
{
namespace utilities
{

inline bool GetRobotParamsFromConfig(json config_json, RobotParams &params)
{
    const auto package_share_path = ament_index_cpp::get_package_share_directory("robot_server_lib");
    if (!config_json.contains("robot_name"))
    {
        LOG_CRIT << "robot.cfg missing 'robot_name' field";
        return false;
    }
    params.robot_name = config_json["robot_name"].get<std::string>();
    // 设置 urdf_path
    params.urdf_path = std::filesystem::path(package_share_path) / "robot_models" / params.robot_name / (params.robot_name + ".urdf");

    // 解析左臂和右臂模型
    if (config_json.contains("left_arm_name"))
    {
        params.left_arm_name = config_json["left_arm_name"].get<std::string>();
        params.left_arm_urdf_path =
            std::filesystem::path(package_share_path) / "robot_models" / params.left_arm_name / (params.left_arm_name + ".urdf");
    }
    if (config_json.contains("right_arm_name"))
    {
        params.right_arm_name = config_json["right_arm_name"].get<std::string>();
        params.right_arm_urdf_path =
            std::filesystem::path(package_share_path) / "robot_models" / params.right_arm_name / (params.right_arm_name + ".urdf");
    }

    // 解析关节映射
    auto parse_joints_maps = [&](const std::string &key, Eigen::VectorXd &dst)
    {
        if (config_json.contains(key))
        {
            auto arr = config_json[key];
            dst = Eigen::VectorXd::Zero(arr.size());
            for (size_t i = 0; i < arr.size(); ++i)
            {
                dst[i] = arr[i].get<int>();
            }
        }
    };

    parse_joints_maps("joints_primary", params.joints_primary);
    parse_joints_maps("joints_secondary", params.joints_secondary);

    if (config_json.contains("joint_command_sequence_by_name"))
    {
        auto arr = config_json["joint_command_sequence_by_name"];
        params.joint_command_sequence_by_name.clear();
        for (const auto& item : arr)
        {
            params.joint_command_sequence_by_name.push_back(item.get<std::string>());
        }
    }
    
    return true;
}

inline void SetMax(std::atomic_int64_t &maximum_value, std::int64_t value)
{
    std::int64_t prev_value = maximum_value;
    while (prev_value < value && !maximum_value.compare_exchange_weak(prev_value, value))
    {
    }
}

inline PoseQuaternion VectorXdToPoseQuaternion(const Eigen::VectorXd &vec)
{
    PoseQuaternion pose;
    pose.position = Eigen::Vector3d(vec[0], vec[1], vec[2]);
    pose.orientation = Eigen::Quaterniond(vec[3], vec[4], vec[5], vec[6]);
    pose.orientation.normalize();
    return pose;
}

inline PoseQuaternion StringToPoseQuaternion(const std::string &str)
{
    PoseQuaternion pose;
    std::stringstream ss(str);
    std::string item;
    double values[7] = {0};
    int idx = 0;
    while (std::getline(ss, item, ',') && idx < 7)
    {
        try
        {
            values[idx++] = std::stod(item);
        }
        catch (...)
        {
        }
    }
    pose.position = Eigen::Vector3d(values[0], values[1], values[2]);
    pose.orientation = Eigen::Quaterniond(values[3], values[4], values[5], values[6]);
    pose.orientation.normalize();
    return pose;
}
}  // namespace utilities
}  // namespace robot_server_lib