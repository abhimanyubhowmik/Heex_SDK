///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "StringMonitor.h"

#include "Tools.h"

StringMonitor::StringMonitor(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion, bool autoStartCom)
    : MonitorV2Interface(uuid, serverIp, serverPort, implementationVersion, autoStartCom)
{
}

ValueConfiguration* StringMonitor::handleValueConfiguration(const std::string& cmd, ValueConfHeader& header)
{
  HEEX_LOG(trace) << "StringMonitor::handleValueConfiguration()" << std::endl;
  if (HeexUtils::caseInsensitiveStringCompare(header.type, "string"))
  {
    ValueConfigurationString* vccb = new ValueConfigurationString(header);
    const bool isVcValid           = vccb->isValid();
    if (!isVcValid)
    {
      HEEX_LOG(error) << "StringMonitor::handleValueConfiguration() Invalid input command : '" << cmd << "'" << std::endl;
      delete vccb;
      return nullptr;
    }
    HEEX_LOG(debug) << "[" << vccb->getCvUuid() << "] StringMonitor::handleValueConfiguration() Values  : ";
    for (std::string s : vccb->getValues())
    {
      HEEX_LOG(debug) << s << " ";
    }
    HEEX_LOG(debug) << std::endl;

    /// This map will be used to keep track of the detection status of each condition values.
    _isOn[vccb->getCvUuid()] = false;
    return vccb;
  }
  else
  {
    HEEX_LOG(error) << "StringMonitor::handleValueConfiguration() Unreconized type : '" << header.type << "'" << std::endl;
  }
  return nullptr;
}

void StringMonitor::updateValue(std::string inputValue)
{
  this->updateValue(inputValue, this->getTimestampStr());
}

void StringMonitor::updateValue(std::string inputValue, const std::string& timestamp)
{
  HEEX_LOG(trace) << "StringMonitor::updateValue()" << std::endl;

  // Check timestamp validity. Warn if invalid but passthrough
  if (!Heex::Tools::checkTimestamp(timestamp))
  {
    HEEX_LOG(warning) << "Received invalid timestamp: " << timestamp << std::endl;
  }

  // Handle all ValueConfigurations registered
  for (std::vector<ValueConfiguration*>::iterator it = _valueConfigurations.begin(); it != _valueConfigurations.end(); ++it)
  {
    if (((*it) != nullptr) && HeexUtils::caseInsensitiveStringCompare((*it)->getType(), "string"))
    {
      ValueConfigurationString* vccb = dynamic_cast<ValueConfigurationString*>((*it));
      if (vccb != nullptr)
      {
        std::vector<std::string> v = vccb->getValues();

        const bool found = (std::find(v.begin(), v.end(), inputValue) != v.end());
        if (found == true && _isOn[vccb->getCvUuid()] == false)
        {
          HEEX_LOG(debug) << "[" << vccb->getCvUuid() << "] StringMonitor::updateValue() : Monitor is ON!" << std::endl;
          if (this->getSignalType() == MonitorSignalType::OnOff)
          {
            this->signalOnOff((timestamp.size() == 0 ? this->getTimestampStr() : timestamp), vccb->getCvUuid());
          }
          else
          {
            this->signalOn((timestamp.size() == 0 ? this->getTimestampStr() : timestamp), vccb->getCvUuid());
            _isOn[vccb->getCvUuid()] = true;
          }
        }
        else if (found == false && (_isOn[vccb->getCvUuid()] == true))
        {
          HEEX_LOG(debug) << "[" << vccb->getCvUuid() << "] StringMonitor::updateValue() : Monitor is Off" << std::endl;
          this->signalOff((timestamp.size() == 0 ? this->getTimestampStr() : timestamp), vccb->getCvUuid());
          _isOn[vccb->getCvUuid()] = false;
        }
      }
      else
      {
        HEEX_LOG(error) << "StringMonitor::updateValue() : Non-valid ValueConfigurationString!" << std::endl;
      }
    }
  }
}

void StringMonitor::onServerDisconnection()
{
  Agent::onServerDisconnection();
  /// We clear our track of the detection status of each condition values as we will get new ones whenever the server is back up.
  _isOn.clear();
}
