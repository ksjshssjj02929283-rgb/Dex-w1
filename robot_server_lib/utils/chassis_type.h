#pragma once
/**
 * @file chassis_type.h
 * @brief 底盘形态枚举。单独成一个零依赖的头文件，供上层的纯计算模块共用。
 *
 * 为什么从 utils.h 拆出来：utils.h 拖着 yaml-cpp / rclcpp_action / nlohmann-json /
 * ROS 接口，而底盘的运动学与命令生成模块要保持"无 ROS、可脱离节点单测"。
 * 把枚举放进这个零依赖头，两边就能共用**同一个**定义，不必各造一份同义枚举。
 *
 * @copyright Copyright (C) 2025 DexForce Technology Co., Ltd. All Rights Reserved.
 */

namespace robot_server_lib {
namespace utils {

/// 底盘形态。
/// 枚举值**刻意与共享内存 `ChassisControl::chassis_mode` 的线上取值对齐**（1 差速 / 2 全向），
/// 所以过 ABI 边界时 `static_cast<int>` 即可，不需要查表；共享内存那侧与 EtherCAT 主站共享，
/// 保持 `int` 不动，本进程内的真值源就是这个枚举。
enum class ChassisType : int { DiffDrive = 1, Omni = 2 };

inline const char* ChassisTypeName(ChassisType t) {
    switch (t) {
        case ChassisType::DiffDrive: return "diff_drive";
        case ChassisType::Omni:      return "omni";
    }
    return "unknown";
}

}  // namespace utils
}  // namespace robot_server_lib
