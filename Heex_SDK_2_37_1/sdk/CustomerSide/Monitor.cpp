///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file Monitor.cpp
///
/// @brief Library source file that contains the basic structure for any Monitors to appropriately register and communicate with the Smart Data Engine.
///
#include "Monitor.h"

#include "AgentArgs.h"
#include "Tools.h"

Monitor::Monitor(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion, bool autoStartCom)
    : Agent(uuid, serverIp, serverPort, implementationVersion, autoStartCom)
{
  // Log a warning message if the Detector UUID doesn't match the expected type and excluding the configuration value prefix.
  std::string warningMsg;
  if (!Heex::Tools::checkAgentNonConsistentUuidType(this->getUuid(), Heex::Tools::UuidPrefixType::Monitor, {"CV"}, warningMsg))
  {
    HEEX_LOG(warning) << warningMsg;
  }
}

Monitor::Monitor(const std::string& uuid, const std::string& configurationFile, const std::string& implementationVersion) : Agent(uuid, configurationFile, implementationVersion) {}

Monitor::~Monitor() = default;
void Monitor::onConfigurationChanged(const std::string& msg)
{
  Agent::onConfigurationChanged(msg);
  this->setSignalType();
}

void Monitor::setSignalType()
{
  if (_constantsValues.find(Heex::HEEX_SIGNAL_TYPE) != _constantsValues.end() && _constantsValues[Heex::HEEX_SIGNAL_TYPE][0] == "OnOff")
  {
    _signalType = MonitorSignalType::OnOff;
  }
  else
  {
    _signalType = MonitorSignalType::On;
  }
}

MonitorSignalType& Monitor::getSignalType()
{
  return _signalType;
}
