#pragma once
#include <sstream>
#include <vector>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

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

} // namespace utils
} // namespace robot_server_lib