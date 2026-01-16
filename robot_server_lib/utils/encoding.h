//
// Created by Wei Gao on 2/14/2023.
//

#pragma once

#include <cinttypes>

namespace robot_server_lib {
namespace utils {

// General utility to pack to integer type
template <typename Int32_Type, typename Int64_Type>
Int64_Type encodeTwo32BitIntAs64Bit(Int32_Type a, Int32_Type b) {
    return (static_cast<Int64_Type>(a) << 32) + b;
}

template <typename Int32_Type, typename Int64_Type>
void decodeTwo32BitIntFrom64Bit(Int64_Type &packed, Int32_Type &a, Int32_Type &b) {
    constexpr Int32_Type lower_32bits = 0xFFFFFFFF;
    a = static_cast<Int32_Type>(packed >> 32);
    b = static_cast<Int32_Type>(packed & lower_32bits);
}

// Special case
std::uint64_t encodeTwoUint32AsUint64(std::uint32_t a, std::uint32_t b);
void decodeTwoUint32FromUint64(std::uint64_t packed, std::uint32_t &a, std::uint32_t &b);

}  // namespace utils
}  // namespace robot_server_lib

#include "encoding.hpp"