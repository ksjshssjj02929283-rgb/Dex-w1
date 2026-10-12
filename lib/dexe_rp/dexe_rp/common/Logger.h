/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <spdlog/spdlog.h>

#include "dexe_rp/common/TypesFmt.h"

#define DEXERP_TRACE(...) SPDLOG_LOGGER_CALL(DexeRP::GetLogger(), spdlog::level::trace, __VA_ARGS__)

#define DEXERP_DEBUG(...) SPDLOG_LOGGER_CALL(DexeRP::GetLogger(), spdlog::level::debug, __VA_ARGS__)

#define DEXERP_INFO(...) SPDLOG_LOGGER_CALL(DexeRP::GetLogger(), spdlog::level::info, __VA_ARGS__)

#define DEXERP_WARN(...) SPDLOG_LOGGER_CALL(DexeRP::GetLogger(), spdlog::level::warn, __VA_ARGS__)

#define DEXERP_ERROR(...) SPDLOG_LOGGER_CALL(DexeRP::GetLogger(), spdlog::level::err, __VA_ARGS__)

#define DEXERP_CRITICAL(...) SPDLOG_LOGGER_CALL(DexeRP::GetLogger(), spdlog::level::critical, __VA_ARGS__)

namespace DexeRP {

// The maximum size of a logging file. (default: 100MB)
inline constexpr size_t kMaxLogFileSize = 100;

std::shared_ptr<spdlog::logger> GetLogger();
void SetConsoleSinkLevel(spdlog::level::level_enum level);
void SetFileSinkLevel(spdlog::level::level_enum level);

}  // namespace DexeRP
