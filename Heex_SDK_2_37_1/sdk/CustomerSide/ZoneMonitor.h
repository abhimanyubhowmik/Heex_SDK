///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#ifndef HEEX_SDK_NO_BOOST_JSON

  #pragma once

  #include "MonitorV2.h"
  #include "MonitorV2Interface.h"
  #include "ValueConfiguration.h"

class ZoneMonitor : public MonitorV2Interface<ZoneMonitor>
{
public:
  ZoneMonitor(
      const std::string& uuid,
      const std::string& serverIp,
      const unsigned int& serverPort,
      const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION,
      bool autoStartCom                        = true);

  // Return name of the component as used int the "type" field in the systemConf.json
  static const std::string getFactoryName() { return "zone"; }

  static std::shared_ptr<MonitorV2>
      createFunc(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion, bool autoStartCom)
  {
    return std::make_shared<ZoneMonitor>(uuid, serverIp, serverPort, implementationVersion, autoStartCom);
  }

  ValueConfiguration* handleValueConfiguration(const std::string& cmd, ValueConfHeader& head);
  void onConfigurationChanged(const std::string& msg) override;
  void updateValue(double latitude, double longitude);
  void updateValue(double latitude, double longitude, const std::string& timestamp);
  void onServerDisconnection();

private:
  std::unordered_map<std::string, bool> _wasInside; ///< Map of Configuration value CV as key and previous frame zone status as value
};

#endif
