///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "GenericMonitor.h"

#include "BooleanMonitor.h"
#include "CallbackClient.h"
#include "IntervalMonitor.h"
#include "MonitorV2.h"
#include "StringMonitor.h"
#include "ThresholdMonitor.h"
#include "Tools.h"
#include "ZoneMonitor.h"

GenericMonitor::GenericMonitor(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion)
    : Monitor(uuid, serverIp, serverPort, implementationVersion)
{
}

GenericMonitor::~GenericMonitor() = default;

ValueConfiguration* GenericMonitor::handleValueConfiguration(const std::string& cmd, ValueConfHeader& head)
{
  // Redirect Threshold ValueConf to be managed by the Interval monitor type
  if (HeexUtils::caseInsensitiveStringCompare(head.type, "threshold"))
  {
    head.type = "interval";
  }

  // Try to find an instance in our list that matches the type of the value conf.
  std::map<std::string, std::shared_ptr<MonitorV2>>::iterator it = _instanceTypes.find(head.type);
  if (it == _instanceTypes.end()) // If the right type is not in the list we have to create one.
  {
    HEEX_LOG(info) << "GenericMonitor::handleValueConfiguration() Creating " << head.type << " instance." << std::endl;
    /// Ask the factory to build a new Monitor without com.
    const std::shared_ptr<MonitorV2> newMonitorInstance = MonitorV2::createMonitorV2(head.type, _uuid, _serverIp, _serverPort, _implementationVersion, false);
    if (newMonitorInstance == nullptr)
    {
      HEEX_LOG(error) << "Failed creating monitor type '" << head.type << "' not registered in the factory.";
      return nullptr;
    }

    // Add a com object that link all com to generic monitor com.
    const std::shared_ptr<CallbackClient> cc = std::make_shared<CallbackClient>(_serverIp, _serverPort, false);
    cc->subscribeToSend(std::bind(&GenericMonitor::forwardToCom, this, std::placeholders::_1));
    newMonitorInstance->setComInterface(cc);

    _instanceTypes[head.type] = newMonitorInstance; // Add the new instance in our list.
    it                        = _instanceTypes.find(head.type);
  }

  // Forward valueConf to instance.
  it->second->createValueConfiguration(cmd);

  // No need to create or store value configurations fot the generic monitor as it is stored in the different instances.
  return nullptr;
}

void GenericMonitor::updateValue(bool value, const std::string& timestamp)
{
  HEEX_LOG(trace) << "GenericMonitor::updateValue(bool)" << std::endl;

  std::string t(timestamp);
  if (t.empty())
  {
    t = this->getTimestampStr();
  }

  // Check t validity. Warn if invalid but passthrough
  if (!Heex::Tools::checkTimestamp(t))
  {
    HEEX_LOG(warning) << "Received invalid timestamp: " << t << std::endl;
  }

  for (std::map<std::string, std::shared_ptr<MonitorV2>>::iterator it = _instanceTypes.begin(); it != _instanceTypes.end(); ++it)
  {
    if (HeexUtils::caseInsensitiveStringCompare(it->first, "boolean"))
    {
      it->second->updateValue(value, t);
    }
  }
}

void GenericMonitor::updateValue(std::string value, const std::string& timestamp)
{
  HEEX_LOG(trace) << "GenericMonitor::updateValue(string)" << std::endl;

  std::string t(timestamp);
  if (t.empty())
  {
    t = this->getTimestampStr();
  }

  // Check t validity. Warn if invalid but passthrough
  if (!Heex::Tools::checkTimestamp(t))
  {
    HEEX_LOG(warning) << "Received invalid timestamp: " << t << std::endl;
  }

  for (std::map<std::string, std::shared_ptr<MonitorV2>>::iterator it = _instanceTypes.begin(); it != _instanceTypes.end(); ++it)
  {
    if (HeexUtils::caseInsensitiveStringCompare(it->first, "string"))
    {
      it->second->updateValue(value, t);
    }
  }
}

void GenericMonitor::updateValue(double value, const std::string& timestamp)
{
  HEEX_LOG(trace) << "GenericMonitor::updateValue(interval)" << std::endl;

  std::string t(timestamp);
  if (t.empty())
  {
    t = this->getTimestampStr();
  }

  // Check t validity. Warn if invalid but passthrough
  if (!Heex::Tools::checkTimestamp(t))
  {
    HEEX_LOG(warning) << "Received invalid timestamp: " << t << std::endl;
  }

  for (std::map<std::string, std::shared_ptr<MonitorV2>>::iterator it = _instanceTypes.begin(); it != _instanceTypes.end(); ++it)
  {
    if (HeexUtils::caseInsensitiveStringCompare(it->first, "interval") || HeexUtils::caseInsensitiveStringCompare(it->first, "threshold"))
    {
      it->second->updateValue(value, t);
    }
  }
}

void GenericMonitor::updateValue(double lat, double lon, const std::string& timestamp)
{
  HEEX_LOG(trace) << "GenericMonitor::updateValue(zone)" << std::endl;

  std::string t(timestamp);
  if (t.empty())
  {
    t = this->getTimestampStr();
  }

  // Check t validity. Warn if invalid but passthrough
  if (!Heex::Tools::checkTimestamp(t))
  {
    HEEX_LOG(warning) << "Received invalid timestamp: " << t << std::endl;
  }

  for (std::map<std::string, std::shared_ptr<MonitorV2>>::iterator it = _instanceTypes.begin(); it != _instanceTypes.end(); ++it)
  {
    if (HeexUtils::caseInsensitiveStringCompare(it->first, "zone"))
    {
      it->second->updateValue(lat, lon, t);
    }
  }
}

std::vector<ValueConfiguration*> GenericMonitor::getValueConfigurationFromInstance(const std::string& instanceType)
{
  std::vector<ValueConfiguration*> ret;
  const std::map<std::string, std::shared_ptr<MonitorV2>>::iterator it = _instanceTypes.find(instanceType);
  if (it == _instanceTypes.end())
  {
    HEEX_LOG(warning) << "GenericMonitor::getValueConfigurationFromInstance instance type not found : " << instanceType;
  }
  else
  {
    ret = it->second->getValueConfigurations();
  }

  return ret;
}

std::unordered_map<std::string, std::vector<std::string>> GenericMonitor::getConstantValuesFromInstance(const std::string& instanceType)
{
  std::unordered_map<std::string, std::vector<std::string>> constantValues;
  const std::map<std::string, std::shared_ptr<MonitorV2>>::iterator it = _instanceTypes.find(instanceType);
  if (it == _instanceTypes.end())
  {
    HEEX_LOG(warning) << "GenericMonitor::getValueConfigurationFromInstance instance type not found : " << instanceType;
  }
  else
  {
    constantValues = it->second->getConstantValues();
  }

  return constantValues;
}

void GenericMonitor::forwardToCom(const std::string& msg)
{
  HEEX_LOG(debug) << "GenericMonitor::forwardToCom() : " << msg << std::endl;

  _tcpC->send(msg);
}

void GenericMonitor::onServerDisconnection()
{
  // Reset value configurations and _isOn variable on all instances.
  for (std::map<std::string, std::shared_ptr<MonitorV2>>::iterator it = _instanceTypes.begin(); it != _instanceTypes.end(); ++it)
  {
    it->second->onServerDisconnection();
  }
  Agent::onServerDisconnection();
}

void GenericMonitor::onConfigurationChanged(const std::string& msg)
{
  // reset instances config.
  for (std::map<std::string, std::shared_ptr<MonitorV2>>::iterator it = _instanceTypes.begin(); it != _instanceTypes.end(); ++it)
  {
    it->second->onConfigurationChanged("");
  }
  Agent::onConfigurationChanged(msg);

  const std::unordered_map<std::string, std::vector<std::string>> monitorConstantValues = this->getConstantValues();
  // Forward constant values to instances
  for (std::map<std::string, std::shared_ptr<MonitorV2>>::iterator it = _instanceTypes.begin(); it != _instanceTypes.end(); ++it)
  {
    it->second->setConstantValues(monitorConstantValues);
    it->second->setSignalType();
  }
}
