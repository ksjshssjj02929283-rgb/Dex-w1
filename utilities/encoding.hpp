//
// Created by Wei Gao on 2/14/2023.
//

namespace robot_server_lib {

inline std::uint64_t encodeTwoUint32AsUint64(std::uint32_t a, std::uint32_t b) {
  return encodeTwo32BitIntAs64Bit<std::uint32_t, std::uint64_t>(a, b);
}

inline void decodeTwoUint32FromUint64(std::uint64_t packed, std::uint32_t& a,
                               std::uint32_t& b) {
  decodeTwo32BitIntFrom64Bit<std::uint32_t, std::uint64_t>(packed, a, b);
}

}  // namespace robot_server_lib