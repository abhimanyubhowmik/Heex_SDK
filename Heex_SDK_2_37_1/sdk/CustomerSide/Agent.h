///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file Agent.h
/// @brief Library header file that contains the basic structure for any Agents to appropriately register and communicate with the Smart Data Engine.
#pragma once

#include <boost/thread.hpp>
#include <string>
#include <unordered_map>

#include "AgentArgs.h"
#include "HeexUtilsLog.h"
#include "HeexUtilsVersion.h"
#include "Incident.h"
#include "SmartDataKitValues.h"
#include "TcpClient.h"
#include "ValueConfiguration.h"

/// @brief This Agent class defines the basic structure of any Agent **agents** to be the interface from the customer edge system (data streams from vehicle, replay, simulation, etc.) to the Heex Smart Data Engine. This class shall not be used directly but aims to be extended with methods to push signals to the Heex SDE offering a modularity in the set of possible strategy and detection logic.
///
/// A Agent is deployed as a network node within the Customer's embedded system, communicating with another node on which the Smart Data Engine is.
/// Agents are customer-coded agents in the distributed architecture of the Heex embedded system. This detector class is part of the open-source sample codes to support the deployment of Agents by the Customers. This approach allows all customers to fully ensure the compliance of Agent implementation with their systems.
///
/// The time-related methods in the Agent class aim to provide a default timestamping system that match Heex Messages specifications. They can be overridden for systems with a different clock or a dedicated time service.
class Agent
{
public:
  /// @brief Constructor for Agent with its full configuration. It requires the uuid and the networking settings.
  ///
  /// @param uuid Unique identifier for the Agent. Shall be a string starting with 'D-' and finishing with a version-4 UUID.
  /// @param serverIp Ip of the Heex Core to which the Agent will connect, identify and notify
  /// @param serverPort Port of the Heex Core dedicated to the Trigger Condition Evaluator (TCE) module
  /// @param implementationVersion Implementation version of the Agent. Transmitted to the SDE during agent identification process.
  Agent(
      const std::string& uuid,
      const std::string& serverIp,
      const unsigned int& serverPort,
      const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION,
      bool autoStartCom                        = true);

  /// @brief Constructor for Agent with its uuid and the path to the configuration file for the networking settings.
  ///
  /// @param uuid Unique identifier for the Agent. Shall be a string starting with 'D-' and finishing with a version-4 UUID.
  /// @param configurationFile Path to the configuration file of the Data Sender.
  /// @param implementationVersion Implementation version of the Agent. Transmitted to the SDE during agent identification process.
  Agent(const std::string& uuid, const std::string& configurationFile, const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION);

  /// @brief Deconstructor for Agent to free memory appropriately. In this case, it destroys the TcpClient pointer.
  virtual ~Agent();

  /// Inform on the connection status with the ConditionEvaluator.
  inline bool isConnected() { return _tcpC->isConnected(); }

  /// Return the uuid of the Agent
  virtual const std::string getUuid() { return _uuid; }

  /// Return the implementation version of the Agent
  virtual const std::string getImplementationVersion() { return _implementationVersion; }

  /// Return the Sdk version of the Agent have been compiled with
  virtual const std::string getSdkVersion() { return _versionChecker.getCurrentVersion(); }

  /// @brief Get current time of the system in the iso_extended std::string format. Aims to be overridden for a different time generation method.
  ///
  /// Default implementation returns the current time of the system using boost::posix_time::microsec_clock::universal_time().
  virtual std::string getTimestampStr();

  /// @brief return true when agent is ready to begin it's work.
  virtual bool isReady();

  /// @brief block until agent is ready to begin it's work.
  virtual void awaitReady();

  /// @brief report an incident to be sent with the metadata.
  virtual void reportIncident(const std::string msg);

  /// @brief Set com interface if defaul interface is not enouth. New com pointer will be deleted by the agent.
  virtual void setComInterface(std::shared_ptr<TcpClient> com);

  /// @brief Permanently disable logging ouput to files.
  void disableLogToFile();

  /// @brief returns Value Configurations.
  virtual std::vector<ValueConfiguration*> getValueConfigurations() { return _valueConfigurations; }

  /// Return constant values Map
  virtual std::unordered_map<std::string, std::vector<std::string>> getConstantValues() { return _constantsValues; };

  /// @brief Additionnal commands to be run before onConfigurationChanged has finished
  virtual void onConfigurationChangedCallback(){};

protected:
  /// @brief Initiate communication.
  virtual void initCom(const std::string& serverIp, const unsigned int& serverPort);

  /// @brief Subscribe all commands on com.
  virtual void subscribeCom();

  /// @brief Be advised when server connect.
  virtual void onServerConnection();

  /// @brief Be advised when server disconnect to clean configuration.
  virtual void onServerDisconnection();

  /// @brief Identification answer to be performed after receiving a identification request from a TCE.
  virtual void onIdentificationRequest(const std::string&);

  /// @brief Identification has been rejected by the SDE. Do not try to reconnect.
  virtual void onIdentificationRejected(const std::string&);

  /// @brief Identification has been accepted by the SDE. Try to log SDE comments.
  virtual void onIdentificationAccepted(const std::string&);

  /// @brief SDE specify a value configuration to the monitor. It provides a new CV-UUID that the monitor shall now use instead of _uuid when activated by the provided value.
  virtual void createValueConfiguration(const std::string&);

  /// @brief SDE specify a constant value to the monitor.
  virtual void setConstantValues();

  /// @brief SDE specify a constant value to the monitor.
  /// @param constantValues Map of key-value for constants values
  virtual void setConstantValues(const std::unordered_map<std::string, std::string>& constantValues);
  virtual void setConstantValues(const std::unordered_map<std::string, std::vector<std::string>>& constantValues);

  /// @brief SDE specify that all agent configurations values needed
  virtual void onConfigurationChanged(const std::string&);

  /// @brief Get current time of the system in the boost::posix_time::ptime format. Aims to be overridden for a different time generation method.
  virtual boost::posix_time::ptime getTimestamp();

  /// @brief Convert a given time in boost::posix_time::ptime format to a std::string in the iso_extended format. Aims to be overridden for a different time generation method.
  virtual std::string convertTimestampToStr(boost::posix_time::ptime time);

  /// @brief Needs to be implemented by any agent in need of value configuration.
  virtual ValueConfiguration* handleValueConfiguration(const std::string& cmd, ValueConfHeader& header);

  /// @brief Clean value configurations.
  virtual void deleteAllValueConfiguration();

  /// @brief Encode incidents to be sent to heexCore.
  virtual std::string encodeIncidents();

  /// @brief store the agent static information. Log any warning or error sent in the message.
  void getStaticInformationFromMsg(const std::string& msg);

  /// @brief Retrive static configuration values for the specific key
  std::vector<std::string> retrieveStaticConfigValues(const std::string& key);

  /// @brief Retrive dynamic (changed by OTA) configuration values for the specific key
  std::vector<std::string> retrieveDynamicConfigValues(const std::string& key);

  /// @brief Extract and set the uuid and implementation version of the Agent
  void extractAgentInfo(const std::string& agentInfo);

  /// @brief Decode Agent configuration message and load it into memory
  virtual void parseAndLoadNewConfiguration(const std::string& msg);

  std::string _agentType;
  std::string _uuid; ///<UUID of the Agent. Shall start by 'D-' followed by a version-4 UUID.
  std::string _serverIp;
  unsigned short _serverPort{};
  bool _agentReady;
  std::shared_ptr<TcpClient> _tcpC; ///<Pointer to the TcpClient instance
  std::vector<ValueConfiguration*> _valueConfigurations;
  std::vector<Heex::Incident> _incidents;
  std::string _implementationVersion;                            ///<Version of the Agent.
  const std::string _SdkVersion = Heex::HEEX_SYSTEM_SDK_VERSION; ///<Version of the Agent.
  /// Composed utility to manage the Sdk version of the Agent have been compiled with
  HeexUtils::VersionChecker _versionChecker;

  /// map that stores the key-values constants values of the agent
  std::unordered_map<std::string, std::vector<std::string>> _constantsValues;

  /// map that stores the key-values static configuration (not refreshed during OTA).A key can be associated to multiples values. min is 1
  std::unordered_map<std::string, std::vector<std::string>> _staticConfigRegistry;

  /// map that stores the key-values dynamic configuration (refreshed during OTA).A key can be associated to multiples values. min is 1
  std::unordered_map<std::string, std::vector<std::string>> _dynamicConfigRegistry;

  friend class GenericMonitor;
};
