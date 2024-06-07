///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#include "BooleanMonitor.h"

#include <boost/algorithm/string.hpp>

#include "Tools.h"

BooleanMonitor::BooleanMonitor(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion, bool autoStartCom)
    : MonitorV2Interface(uuid, serverIp, serverPort, implementationVersion, autoStartCom)
{
}

ValueConfiguration* BooleanMonitor::handleValueConfiguration(const std::string& cmd, ValueConfHeader& header)
{
  HEEX_LOG(trace) << "BooleanMonitor::handleValueConfiguration()" << std::endl;
  if (HeexUtils::caseInsensitiveStringCompare(header.type, "boolean"))
  {
    ValueConfigurationBoolean* vcb = new ValueConfigurationBoolean(header); // NOLINT: not using smart pointer for valueConfigurations due to backwards compatibility
    const bool isVcValid           = vcb->isValid();
    if (!isVcValid)
    {
      HEEX_LOG(error) << "BooleanMonitor::handleValueConfiguration() Invalid input command : '" << cmd << "'" << std::endl;
      delete vcb; // NOLINT: not using smart pointer for valueConfigurations due to backwards compatibility
      return nullptr;
    }

    HEEX_LOG(debug) << "[" << vcb->getCvUuid() << "] BooleanMonitor::handleValueConfiguration() Value boolean : " << vcb->getValue() << std::endl;
    /// This map will be used to keep track of the detection status of each condition values.
    _isOn[vcb->getCvUuid()] = false;
    return vcb;
  }
  else
  {
    HEEX_LOG(error) << "BooleanMonitor::handleValueConfiguration() Unrecognized type : '" << header.type << "'" << std::endl;
  }
  return nullptr;
}

void BooleanMonitor::updateValue(bool inputValue)
{
  this->updateValue(inputValue, this->getTimestampStr());
}

void BooleanMonitor::updateValue(bool inputValue, const std::string& timestamp)
{
  HEEX_LOG(trace) << "BooleanMonitor::updateValue()" << std::endl;

  // Check timestamp validity. Warn if invalid but passthrough
  if (!Heex::Tools::checkTimestamp(timestamp))
  {
    HEEX_LOG(warning) << "Received invalid timestamp: " << timestamp << std::endl;
  }

  // Handle all ValueConfigurations registered
  for (std::vector<ValueConfiguration*>::iterator it = _valueConfigurations.begin(); it != _valueConfigurations.end(); ++it)
  {
    if (((*it) != nullptr) && HeexUtils::caseInsensitiveStringCompare((*it)->getType(), "boolean"))
    {
      ValueConfigurationBoolean* vcb = dynamic_cast<ValueConfigurationBoolean*>((*it));
      if (vcb != nullptr)
      {
        HEEX_LOG(debug) << "[" << vcb->getCvUuid() << "] BooleanMonitor::updateValue() requestedValue : " << vcb->getValue() << " inputValue : " << inputValue << std::endl;
        HEEX_LOG(debug) << "[" << vcb->getCvUuid() << "] BooleanMonitor::updateValue() IsValueGood : " << (vcb->getValue() == inputValue) << "[" << vcb->getCvUuid()
                        << "]  _isOn : " << _isOn[vcb->getCvUuid()] << std::endl;

        if (vcb->getValue() == inputValue && _isOn[vcb->getCvUuid()] == false)
        {
          HEEX_LOG(debug) << "[" << vcb->getCvUuid() << "] BooleanMonitor::updateValue() : Monitor is ON!" << std::endl;
          if (this->getSignalType() == MonitorSignalType::OnOff)
          {
            this->signalOnOff((timestamp.size() == 0 ? this->getTimestampStr() : timestamp), vcb->getCvUuid());
          }
          else
          {
            this->signalOn((timestamp.size() == 0 ? this->getTimestampStr() : timestamp), vcb->getCvUuid());
            _isOn[vcb->getCvUuid()] = true;
          }
        }
        else if ((vcb->getValue() != inputValue) && (_isOn[vcb->getCvUuid()] == true))
        {
          HEEX_LOG(debug) << "[" << vcb->getCvUuid() << "] BooleanMonitor::updateValue() : Monitor is OFF!" << std::endl;
          this->signalOff((timestamp.size() == 0 ? this->getTimestampStr() : timestamp), vcb->getCvUuid());
          _isOn[vcb->getCvUuid()] = false;
        }
      }
      else
      {
        HEEX_LOG(error) << "BooleanMonitor::updateValue() : Non-valid ValueConfigurationBoolean!" << std::endl;
      }
    }
  }
}

void BooleanMonitor::onServerDisconnection()
{
  Agent::onServerDisconnection();
  /// We clear our track of the detection status of each condition values as we will get new ones whenever the server is back up.
  _isOn.clear();
}
