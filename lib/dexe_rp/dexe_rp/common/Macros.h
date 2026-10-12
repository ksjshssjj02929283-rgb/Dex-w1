/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <fmt/format.h>

#include <memory>
#include <stdexcept>

#include "dexe_rp/common/Logger.h"

/**
 * @brief Throw a runtime exception with a formatted error message
 *        and log the error at the same time.
 */
#define DEXERP_THROW(...)                                   \
    do {                                                    \
        DEXERP_ERROR(__VA_ARGS__);                          \
        throw std::runtime_error(fmt::format(__VA_ARGS__)); \
    } while (0)

/**
 * @brief Ensure that a condition is true, otherwise throw an exception.
 */
#define DEXERP_ENSURE(cond, ...)                \
    do {                                        \
        if (!(cond)) DEXERP_THROW(__VA_ARGS__); \
    } while (0)
