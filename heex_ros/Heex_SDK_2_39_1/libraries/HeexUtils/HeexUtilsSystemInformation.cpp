///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#include <HeexUtilsSystemInformation.h>

namespace HeexUtils
{
SystemInformation::SystemInformation() : _os(HEEX_BUILD_OS_NAME), _arch(HEEX_BUILD_ARCHITECTURE_NAME) {}
} // namespace HeexUtils
