///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include "MonitorV2.h"

/// @brief GenericMonitor class that create one or multiple instances of Monitor types adapted to the type of value configuration it receive from the Core.
/// It needs only one instance of each type. All instances comunicates to the core via the GenericMonitor com object.
/// GenericMonitor do not store value configuration itself, it forwards them to it's diferent instances.
class GenericMonitor : public Monitor
{
public:
  GenericMonitor(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION);

  virtual ~GenericMonitor();

  /// @brief Create an instance for the value configuration type
  /// Forward agent configuration message with constant block and the value configuration
  virtual ValueConfiguration* handleValueConfiguration(const std::string& cmd, ValueConfHeader& head) override;

  virtual void updateValue(bool value, const std::string& timestamp = "");
  virtual void updateValue(std::string value, const std::string& timestamp = "");
  virtual void updateValue(double value, const std::string& timestamp = "");
  virtual void updateValue(double lat, double lon, const std::string& timestamp = "");

  virtual std::vector<ValueConfiguration*> getValueConfigurationFromInstance(const std::string& instanceType);
  virtual std::unordered_map<std::string, std::vector<std::string>> getConstantValuesFromInstance(const std::string& instanceType);

protected:
  virtual void onConfigurationChanged(const std::string&) override;

  virtual void forwardToCom(const std::string& msg);

  virtual void onServerDisconnection();

  // list of monitor types created after reception of a new type of value configuration.
  std::map<std::string, std::shared_ptr<MonitorV2>> _instanceTypes;
};
