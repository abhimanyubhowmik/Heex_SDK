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
#include "units.hpp"

class IntervalMonitor : public MonitorV2Interface<IntervalMonitor>
{
public:
  IntervalMonitor(
      const std::string& uuid,
      const std::string& serverIp,
      const unsigned int& serverPort,
      const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION,
      bool autoStartCom                        = true);

  // Return name of the component as used int the "type" field in the systemConf.json
  static const std::string getFactoryName() { return "interval"; }

  static std::shared_ptr<MonitorV2>
      createFunc(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion, bool autoStartCom)
  {
    return std::make_shared<IntervalMonitor>(uuid, serverIp, serverPort, implementationVersion, autoStartCom);
  }

  ValueConfiguration* handleValueConfiguration(const std::string& cmd, ValueConfHeader& head);
  void onConfigurationChanged(const std::string& msg) override;
  void updateValue(double inputValue);
  void updateValue(double inputValue, const std::string& timestamp);
  void onServerDisconnection();

  /// @brief Set the signal Unit
  void setSignalUnit(std::string unit);

  /// @brief Get the signal Unit as a string
  std::string getSignalUnit();

private:
  bool areUnitsCompatible();
  void updateIntervalLogic(ValueConfigurationDoubleInterval* vcdi, double inputValue, const std::string& timestamp);
  void updateThresholdLogic(ValueConfigurationThreshold* vcthr, double inputValue, const std::string& timestamp);

  /// @brief Wrap the logic to send signal to Core based on MonitorSignalType
  void sendSignalOn(const std::string cvUuid, std::string timestamp);

  units::unit _signalUnit;
  std::string _signalUnitStr;
  units::unit _targetUnit;
  std::string _targetUnitStr;
  bool _acceptUnitConversion;
};
