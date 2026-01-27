/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <string>

#define DEXERP_VERSION "0.1.3"

namespace DexeRP {

std::string GetVersion() { return DEXERP_VERSION; }

std::string GetDataPath() { return ""; }  // TODO:

}  // namespace DexeRP
