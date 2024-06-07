///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#pragma once

#include <string>

namespace Heex
{
#ifdef HEEX_BUILD_VERSION
const std::string HEEX_SYSTEM_SDK_VERSION = HEEX_BUILD_VERSION;
#else
const std::string HEEX_SYSTEM_SDK_VERSION = "0.0.0-unknown";
#endif
} // namespace Heex
