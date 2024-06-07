///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file BaseMonitor.h
///
/// @brief Library header file that defines and implements a Base Monitor structure that can send `signalOn` or `signalOff` signals to the Heex Smart Data Engine.
///
#pragma once

#include "Monitor.h"

/// @brief This BaseMonitor class is a specialization of the structure of the Monitor class to implement methods that push binary detection signals to the Heex Smart Data Engine as `signalOn` or `signalOff`.
///
/// Both the methods signalOn() and signalOff() can be overridden to fit.
/// Method signalOn() and signalOff() can be called without arguments to use the time returned by the getTimestampStr() method.
///
/// The getTimestampStr() method belongs to the Monitor class. It can also be overridden to fit a different time generation method.
class BaseMonitor : public Monitor
{
public:
  /// @brief Constructor for BaseMonitor with its full configuration. It requires the uuid and the networking settings. Reuses the constructor of the Monitor class as-is.
  ///
  /// @param uuid Unique identifier for the BaseMonitor. Shall be a string starting with 'D-' and finishing with a version-4 UUID.
  /// @param serverIp Ip of the Heex Core to which the BaseMonitor will connect, identify and notify
  /// @param serverPort Port of the Heex Core dedicated to the Trigger Condition Evaluator (TCE) module
  /// @param implementationVersion Implementation version of the Monitor. Transmitted to the SDE during agent identification process.
  BaseMonitor(
      const std::string& uuid,
      const std::string& serverIp,
      const unsigned int& serverPort,
      const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION,
      bool autoStartCom                        = true);

  /// @brief Constructor for BaseMonitor with its uuid and the path to the configuration file for the networking settings. Reuses the constructor of the Monitor class as-is.
  ///
  /// @param uuid Unique identifier for the BaseMonitor. Shall be a string starting with 'D-' and finishing with a version-4 UUID.
  /// @param configurationFile Path to the configuration file of the Data Sender.
  /// @param implementationVersion Implementation version of the Monitor. Transmitted to the SDE during agent identification process.
  BaseMonitor(const std::string& uuid, const std::string& configurationFile, const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION);

  /// @brief Push the binary detection signal `signalOn` to the Heex Smart Data Engine. An empty string or no argument for timestamp will result in calling the getTimestampStr() method that get system time.
  ///
  /// @param timestamp Timestamp that designates when the detection has happened. Default parameter is an empty string and will result to getTimestampStr() method call.
  virtual void signalOn();
  virtual void signalOn(const std::string& timestamp);
  virtual void signalOn(const std::string& timestamp, const std::string& uuid);

  /// @brief Push the binary detection signal `signalOff` to the Heex Smart Data Engine. An empty string or no argument for timestamp will result in calling the getTimestampStr() method that get system time.
  ///
  /// @param timestamp Timestamp that designates when the detection has happened. Default parameter is an empty string and will result to getTimestampStr() method call.
  virtual void signalOff();
  virtual void signalOff(const std::string& timestamp);
  virtual void signalOff(const std::string& timestamp, const std::string& uuid);

  /// @brief Push the event detection signal `signalOnOff` to the Heex Smart Data Engine. An empty string or no argument for timestamp will result in calling the getTimestampStr() method that get system time.
  ///
  /// @param timestamp Timestamp that designates when the detection has happened. Default parameter is an empty string and will result to getTimestampStr() method call.
  virtual void signalOnOff();
  virtual void signalOnOff(const std::string& timestamp);
  virtual void signalOnOff(const std::string& timestamp, const std::string& uuid);

  /// Map of Configuration value CV as key and activation status as value
  std::unordered_map<std::string, bool> _isOn;
};
