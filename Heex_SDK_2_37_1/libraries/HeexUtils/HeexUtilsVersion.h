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
class VersionChecker
{
public:
  /// Constructor for the VersionChecker class
  VersionChecker(const std::string& version);

  /// @brief Function to check the version using a regular expression to match any version like M.N.*
  ///
  /// @param version Version of SDK to confront
  /// @return true Version is match
  /// @return false
  bool checkMatchMajorMinorVersion(const std::string& version);

  const std::string& getCurrentVersion() { return _version; }

private:
  const std::string _version;
  std::string _expectedMajor;
  std::string _expectedMinor;
};
} // namespace HeexUtils
