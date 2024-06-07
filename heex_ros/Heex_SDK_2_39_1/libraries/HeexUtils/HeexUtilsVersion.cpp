///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#include "HeexUtilsVersion.h"

#include <regex>

#include "HeexUtils.h"
#include "HeexUtilsLog.h"

namespace HeexUtils
{
VersionChecker::VersionChecker(const std::string& version) : _version(version)
{
  // Check if version is release "Major.Minor.Patch" or development "Major.Minor.Patch.Tweak"
  std::vector<std::string> versionDelimiterSplit = HeexUtils::split(version, '.');
  if (versionDelimiterSplit.size() != 3 && versionDelimiterSplit.size() != 4)
  {
    HEEX_LOG(error) << "VersionChecker | Can't initialized as version " << _version << " does not contains expected number of digits for the Major.Minor.Patch form.";
    return;
  }

  _expectedMajor = versionDelimiterSplit[0];
  _expectedMinor = versionDelimiterSplit[1];

  if (!checkMatchMajorMinorVersion(_version))
  {
    HEEX_LOG(error) << "VersionChecker | Can't initialized as version " << _version << " does not match the Major.Minor.Patch form.";
    return;
  }
}

bool VersionChecker::checkMatchMajorMinorVersion(const std::string& version)
{
  // Define the regular expression to match versions of the form X.Y.Z with optional [.tweak] and [-<dev>]
  // where X, Y, and Z are non-negative integers
  const std::regex versionRegex("^(\\d+)\\.(\\d+)\\.(\\d+)(\\.\\d+)?(-\\w+)?$");

  // Guard if the version doesn't match the regular expression
  if (!std::regex_match(version, versionRegex))
  {
    HEEX_LOG(error) << "VersionChecker | Provided version " << version << " does not match the Major.Minor.Patch form.";
    return false;
  }

  // Split the version into version components and guard on number
  std::vector<std::string> versionDelimiterSplit = HeexUtils::split(version, '.');
  if (versionDelimiterSplit.size() != 3 && versionDelimiterSplit.size() != 4)
  {
    HEEX_LOG(error) << "VersionChecker | Size:" << versionDelimiterSplit.size();
    HEEX_LOG(error) << "VersionChecker | Failed to extract version " << version << " with the Major.Minor.Patch form.";
    return false;
  }

  // Guard if the major and minor versions are not the same as the expected versions
  if (versionDelimiterSplit[0] != _expectedMajor || versionDelimiterSplit[1] != _expectedMinor)
  {
    return false;
  }

  return true;
}
} // namespace HeexUtils
