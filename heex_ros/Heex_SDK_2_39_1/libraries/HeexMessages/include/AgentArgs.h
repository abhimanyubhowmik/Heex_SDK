///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#pragma once

#include <ostream>
#include <string>

/// Args structure for Identification used in Agent interprocess communication.
struct IdentificationArgs
{
  bool valid{false};
  std::string uuid;
  std::string name;
  std::string timestamp;
  std::string implementationVersion;
  std::string sdkVersion;
  std::string agentType;
  std::string unparsedArgs;

  std::string returnMessages;

  /// Make the IdentificationArgs structure printable with output stream
  friend std::ostream& operator<<(std::ostream& os, const IdentificationArgs& item)
  {
    return os << "{ uuid = '" << item.uuid << "', name = '" << item.name << "', timestamp = '" << item.timestamp << "', implementationVersion = '" << item.implementationVersion
              << "', sdkVersion = '" << item.sdkVersion << "', agentType = '" << item.agentType << "', unparsedArgs = '" << item.unparsedArgs << "', returnMessages = '"
              << item.returnMessages << "' } ";
  }
};
namespace Heex
{
const char HEEX_MSG_ARG_SEPARATOR                            = ' ';
const char HEEX_COUPLE_SEPARATOR                             = ';';
const char HEEX_KEY_VALUE_SEPARATOR                          = ':';
const std::string HEEX_DEFAULT_AGENT_VERSION                 = "1.0.0";
const std::string HEEX_AGENT_IDENTIFICATION_INIT             = "Identification";
const std::string HEEX_AGENT_IDENTIFICATION_REQUEST          = "IdentificationRequest";
const std::string HEEX_AGENT_IDENTIFICATION_REJECTED         = "IdentificationRejected";
const std::string HEEX_AGENT_IDENTIFICATION_ACCEPTED         = "IdentificationAccepted";
const std::string HEEX_AGENT_CONFIGURATION                   = "ConfigurationChanged";
const std::string HEEX_AGENT_IDENTIFICATION_REJECTION_REASON = "HEEX_AGENT_IDENTIFICATION_REJECTION_REASON";
const std::string HEEX_AGENT_SDK_VERSION_DIFFER              = "HEEX_AGENT_SDK_VERSION_DIFFER";
const std::string HEEX_SDE_WORKING_DIRECTORY                 = "HEEX_SDE_WORKING_DIRECTORY";
const std::string HEEX_RECORDING_RANGE_KEY                   = "HEEX_RECORDING_RANGE_KEY";
const std::string HEEX_VALUE_CONFIGURATION_KEY               = "HEEX_VALUE_CONFIGURATION_KEY";
const std::string HEEX_CONSTANTS_VALUES_KEY                  = "HEEX_CONSTANTS_VALUES_KEY";
const std::string HEEX_UNIT_KEY                              = "HEEX_UNIT_KEY";
const std::string HEEX_SIGNAL_TYPE                           = "signalType";
const std::string HEEX_CONSTANT_MD5_KEY                      = "md5";
const std::string HEEX_AGENT_TYPE_DEFAULT                    = "DefaultType";
const std::string HEEX_AGENT_TYPE_RECORDERV2                 = "RecorderV2";
} // namespace Heex
