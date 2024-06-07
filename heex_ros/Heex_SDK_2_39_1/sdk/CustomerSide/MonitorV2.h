///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include "BaseMonitor.h"

/// @brief

class MonitorV2 : public BaseMonitor
{
public:
  using CreateFunc = std::function<std::shared_ptr<
      MonitorV2>(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion, bool autoStartCom)>;

  /// @brief Constructor for MonitorV2 with its full configuration. It requires the uuid and the networking settings.
  ///
  /// @param uuid Shared identifier for the MonitorV2. Shall be a string starting with 'D-' and finishing with a version-4 UUID.
  /// @param serverIp Ip of the Heex Core to which the MonitorV2 will connect, identify and notify
  /// @param serverPort Port of the Heex Core dedicated to the Trigger Condition Evaluator (TCE) module
  /// @param implementationVersion Implementation version of the monitor. Transmitted to the SDE during agent identification process.
  MonitorV2(
      const std::string& uuid,
      const std::string& serverIp,
      const unsigned int& serverPort,
      const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION,
      bool autoStartCom                        = true);

  virtual ~MonitorV2() = default;

  static std::map<std::string, CreateFunc>& getRegistry()
  {
    static std::map<std::string, CreateFunc> registry;
    return registry;
  }

  static std::shared_ptr<MonitorV2> createMonitorV2(
      const std::string& type,
      const std::string& uuid,
      const std::string& serverIp,
      const unsigned int& serverPort,
      const std::string& implementationVersion,
      bool autoStartCom)
  {
    for (std::map<std::string, CreateFunc>::iterator it = MonitorV2::getRegistry().begin(); it != MonitorV2::getRegistry().end(); ++it)
    {
      if (it->first == type)
      {
        return it->second(uuid, serverIp, serverPort, implementationVersion, autoStartCom);
      }
    }
    return nullptr;
  }

  virtual void updateValue(bool value, const std::string& timestamp = "");
  virtual void updateValue(std::string value, const std::string& timestamp = "");
  virtual void updateValue(double value, const std::string& timestamp = "");
  virtual void updateValue(double lat, double lon, const std::string& timestamp = "");

private:
  // Prevent anyone from directly inhereting from MonitorV2
  virtual void doNotInheritFromMonitorV2DirectlyButFromMonitorV2InterfaceInstead() = 0;
};
