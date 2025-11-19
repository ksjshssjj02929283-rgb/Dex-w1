#pragma once

#include <nlohmann/json.hpp>
#include <sstream>
#include <vector>

using json = nlohmann::json;

#define CTRLWORD_MOTOR_SWITCH_ON (1U << 0)
#define CTRLWORD_MOTOR_ENABLE (1U << 1)
#define CTRLWORD_MOTOR_QUICK_STOP (1u << 2)
#define CTRLWORD_MOTOR_CLEAR_FAULT (1U << 3)
#define CTRLWORD_MOTOR_CST_MODE (1U << 4)

#define STATUSWORD_MOTOR_READY (1U << 0)
#define STATUSWORD_MOTOR_ENABLED (1U << 1)
#define STATUSWORD_MOTOR_FAULT (1U << 2)
#define STATUSWORD_MOTOR_QUICK_STOP (1U << 3)

namespace robot_server_lib
{
namespace utils
{

template <typename VectorType>
std::ostringstream MotorStatusToJsonArrayString(const VectorType &vec, int start_index, int end_index)
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
            auto data = static_cast<long>(vec[i]);
            switch (data)
            {
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
        }
        else
        {
            ss << "invalid data";
        }
        if (i < end_index - 1)
            ss << ",";
    }
    ss << "]";
    return ss;
}

inline std::ostringstream RobotStatuswordToJsonString(const int &status_word)
{
    std::ostringstream ss;
    switch (status_word)
    {
    case 0:
        ss << "\"NOT_READY\"";
        break;
    case 1:
        ss << "\"READY_NOT_ENABLE\"";
        break;
    case 2:
        ss << "\"ENABLED\"";
        break;
    case 3:
        ss << "\"READY_AND_ENABLED\"";
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

template <typename VectorType>
std::ostringstream MotorErrorcodeToJsonArrayString(const VectorType &vec, int start_index, int end_index)
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
            auto data = static_cast<long>(vec[i]);
            switch (data)
            {
            case 33792:
                ss << 10;
                break;
            case 34048:
                ss << 20;
                break;
            case 40960:
                ss << 30;
                break;
            default:
                ss << data;
                break;
            }
        }
        else
        {
            ss << "invalid data";
        }
        if (i < end_index - 1)
            ss << ",";
    }
    ss << "]";
    return ss;
}

}  // namespace utils
}  // namespace robot_server_lib