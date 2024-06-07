///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#pragma once
#include <string>

namespace HeexUtils
{
const std::string HEEX_BUILD_UNKNOWN = "Unknown";

#ifndef HEEX_BUILD_OS_NAME
  #define HEEX_BUILD_OS_NAME HEEX_BUILD_UNKNOWN
#endif

#ifndef HEEX_BUILD_ARCHITECTURE_NAME
  #define HEEX_BUILD_ARCHITECTURE_NAME HEEX_BUILD_UNKNOWN
#endif

/// @brief A class for retrieving system information like architecture and os.
class SystemInformation
{
private:
  const std::string _os;
  const std::string _arch;

public:
  SystemInformation();

  std::string getOs() const { return _os; }

  std::string getArchitecture() const { return _arch; }
};
} // namespace HeexUtils
