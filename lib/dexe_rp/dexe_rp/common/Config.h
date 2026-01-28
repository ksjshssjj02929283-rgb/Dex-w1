/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <spdlog/fmt/fmt.h>

#include <filesystem>
#include <string>

namespace DexeRP {

inline std::string GetLogFilePath() {
    static auto path = std::filesystem::path(getenv("HOME")) / "log";
    if (!std::filesystem::exists(path)) {
        if (!std::filesystem::create_directories(path)) {
            fmt::print("Failed to create path: {}\n", path.string());
            return "";
        }
    }
    return path.string();
}

inline std::string GetConfigFilePath() {
    static auto path = std::filesystem::path(getenv("HOME")) / "dexe_rp_data";
    if (!std::filesystem::exists(path)) {
        if (!std::filesystem::create_directories(path)) {
            fmt::print("Failed to create path: {}\n", path.string());
            return "";
        }
    }
    return path.string();
}

}  // namespace DexeRP