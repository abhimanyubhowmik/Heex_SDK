///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include <string>

#include "MonitorV2Interface.h"
#include "ValueConfiguration.h"

class BooleanMonitor : public MonitorV2Interface<BooleanMonitor>
{
public:
  BooleanMonitor(
      const std::string& uuid,
      const std::string& serverIp,
      const unsigned int& serverPort,
      const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION,
      bool autoStartCom                        = true);

  // Return name of the component as used int the "type" field in the systemConf.json
  static const std::string getFactoryName() { return "boolean"; }

  static std::shared_ptr<MonitorV2>
      createFunc(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion, bool autoStartCom)
  {
    return std::make_shared<BooleanMonitor>(uuid, serverIp, serverPort, implementationVersion, autoStartCom);
  }

  ValueConfiguration* handleValueConfiguration(const std::string& cmd, ValueConfHeader& head);
  void updateValue(bool inputValue);
  void updateValue(bool inputValue, const std::string& timestamp);
  void onServerDisconnection();
};
