/**
 * @file utilities.h
 * @brief 通用工具函数
 * @copyright Copyright (C) 2025 DexForce Technology Co., Ltd. All Rights Reserved.
 */
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

namespace robot_server_lib {
namespace utilities {

bool GetRobotParamsFromConfig(json config_json, RobotParams &params);
void SetMax(std::atomic_int64_t &maximum_value, std::int64_t value);
PoseQuaternion VectorXdToPoseQuaternion(const Eigen::VectorXd &vec);
PoseQuaternion StringToPoseQuaternion(const std::string &str);

}  // namespace utilities
}  // namespace robot_server_lib