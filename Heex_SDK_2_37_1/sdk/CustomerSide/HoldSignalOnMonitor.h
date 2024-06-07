///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file HoldSignalOnMonitor.h
///
/// @brief Library header file that defines and implements a HoldSignalOnMonitor structure that send a `signalOn` and `singalOnOff` with a state-machine logic to hold the `On` state during a given time.
///
#pragma once
#include <boost/thread.hpp>

#include "Monitor.h"

/// @brief This HoldSignalOnMonitor class is a specialization of the structure of the Monitor class that implement a state machine to maintain a `On` state activation period. The initial transition to the 'On' state will send an `signalOn` detection signals to the Heex Smart Data Engine and a `signalOff` after the delay has passed.
///
/// Both the methods signalOn() and signalOff() have been overridden to fit the state transition and hold `On` logic.
/// NOTE: Requires to be linked with the boost_thread lib.
///
/// The getTimestampStr() method belongs to the Monitor class. It can also be overridden to fit a different time generation method.
class HoldSignalOnMonitor : public Monitor
{
public:
  /// @brief Constructor for HoldSignalOnMonitor with its full configuration. It requires the uuid and the networking settings. Reuses the constructor of the Monitor class as-is.
  ///
  /// @param uuid Unique identifier for the HoldSignalOnMonitor. Shall be a string starting with 'D-' and finishing with a version-4 UUID.
  /// @param serverIp Ip of the Heex Core to which the HoldSignalOnMonitor will connect, identify and notify
  /// @param serverPort Port of the Heex Core dedicated to the Trigger Condition Evaluator (TCE) module
  /// @param delay Delay in microseconds to maintain the On state in the Monitor discarding any signalOn and signalOff signals to be send to Heex Smart Data Engine while in the On state.
  /// @param implementationVersion Implementation version of the Monitor. Transmitted to the SDE during agent identification process.
  HoldSignalOnMonitor(
      const std::string& uuid,
      const std::string& serverIp,
      const unsigned int& serverPort,
      const u_int delay,
      const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION);

  /// @brief Constructor for HoldSignalOnMonitor with its uuid and the path to the configuration file for the networking settings. Reuses the constructor of the Monitor class as-is.
  ///
  /// @param uuid Unique identifier for the HoldSignalOnMonitor. Shall be a string starting with 'D-' and finishing with a version-4 UUID.
  /// @param configurationFile Path to the configuration file of the Data Sender.
  /// @param delay Delay to maintain the On state in the Heex Smart Data Engine. Any signalOn and signalOff will be discard while in the HoldSignalOnMonitor is in the On state.
  /// @param implementationVersion Implementation version of the Monitor. Transmitted to the SDE during agent identification process.
  HoldSignalOnMonitor(
      const std::string& uuid,
      const std::string& configurationFile,
      const u_int delay,
      const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION);

  /// @brief Push the binary detection signal `signalOn` to the Heex Smart Data Engine. Switch the Monitor state to On for the given delay.
  /// An empty string or no argument for timestamp will result in calling the getTimestampStr() method that get system time.
  ///
  /// @param timestamp Timestamp that designates when the detection has happened. Default parameter is an empty string and will result to getTimestampStr() method call.
  virtual void signalOn();
  virtual void signalOn(const std::string& timestamp);
  virtual void signalOn(const std::string& timestamp, const std::string& uuid);

protected:
  /// @brief Push the binary detection signal `signalOff` to the Heex Smart Data Engine. An empty string or no argument for timestamp will result in calling the getTimestampStr() method that get system time.
  ///
  /// @param timestamp Timestamp that designates when the detection has happened.
  virtual void signalOff(const std::string& timestamp);

  /// @brief Waiting function to be run in another thread to wait enough time before sending the `signalOff` message.
  virtual void sendSignalOffAfterDelay(const std::string& timestamp = "");

private:
  boost::posix_time::time_duration _delay;     ///< Delay to maintain the `On` state (in microseconds)
  boost::posix_time::ptime _lastestActivation; ///< Cache the last activation time
  bool _isOn;                                  ///< Catch the the Monitor state (On / Off)
  boost::thread* _thread{nullptr};             ///< Pointer to the thread in which the Monitor send a `signalOff` after the delay
  std::string _cvUuid;
};
