///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file Monitor.h
///
/// @brief Library header file that contains the basic structure for any Monitors to appropriately register and communicate with the Smart Data Engine.
///
#pragma once

#include "Agent.h"

enum class MonitorSignalType
{
  On,
  OnOff,
};

/// @brief This Monitor class defines the basic structure of any Monitor **agents** to be the interface from the customer edge system (data streams from vehicle, replay, simulation, etc.) to the Heex Smart Data Engine. This class shall not be used directly but aims to be extended with methods to push signals to the Heex SDE offering a modularity in the set of possible strategy and detection logic.
///
/// A Monitor is deployed as a network node within the Customer's embedded system, communicating with another node on which the Smart Data Engine is.
/// Monitors are customer-coded agents in the distributed architecture of the Heex embedded system. This Monitor class is part of the open-source sample codes to support the deployment of Monitors by the Customers. This approach allows all customers to fully ensure the compliance of Monitor implementation with their systems.
///
/// The time-related methods in the Monitor class aim to provide a default timestamping system that match Heex Messages specifications. They can be overridden for systems with a different clock or a dedicated time service.
class Monitor : public Agent
{
public:
  /// @brief Constructor for Monitor with its full configuration. It requires the uuid and the networking settings.
  ///
  /// @param uuid Unique identifier for the Monitor. Shall be a string starting with 'D-' and finishing with a version-4 UUID.
  /// @param serverIp Ip of the Heex Core to which the Monitor will connect, identify and notify
  /// @param serverPort Port of the Heex Core dedicated to the Trigger Condition Evaluator (TCE) module
  /// @param implementationVersion Implementation version of the Monitor. Transmitted to the SDE during agent identification process.
  Monitor(
      const std::string& uuid,
      const std::string& serverIp,
      const unsigned int& serverPort,
      const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION,
      bool autoStartCom                        = true);

  /// @brief Constructor for Monitor with its uuid and the path to the configuration file for the networking settings.
  ///
  /// @param uuid Unique identifier for the Monitor. Shall be a string starting with 'D-' and finishing with a version-4 UUID.
  /// @param configurationFile Path to the configuration file of the Data Sender.
  /// @param implementationVersion Implementation version of the Monitor. Transmitted to the SDE during agent identification process.
  Monitor(const std::string& uuid, const std::string& configurationFile, const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION);

  /// @brief Deconstructor for Monitor to free memory appropriately. In this case, it destroys the TcpClient pointer.
  virtual ~Monitor();

  /// @brief SDE specify that all agent configurations values needed
  virtual void onConfigurationChanged(const std::string&) override;

  /// set Monitor signalType
  virtual void setSignalType();

  /// Get Monitor signalType
  virtual MonitorSignalType& getSignalType();

protected:
  MonitorSignalType _signalType{MonitorSignalType::On};
};
