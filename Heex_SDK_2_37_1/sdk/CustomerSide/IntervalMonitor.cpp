///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "IntervalMonitor.h"

#include "Tools.h"

IntervalMonitor::IntervalMonitor(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion, bool autoStartCom)
    : MonitorV2Interface(uuid, serverIp, serverPort, implementationVersion, autoStartCom)
{
  _acceptUnitConversion = false;
}

ValueConfiguration* IntervalMonitor::handleValueConfiguration(const std::string& cmd, ValueConfHeader& header)
{
  HEEX_LOG(trace) << "IntervalMonitor::handleValueConfiguration()" << std::endl;
  if (HeexUtils::caseInsensitiveStringCompare(header.type, "interval"))
  {
    ValueConfigurationDoubleInterval* vcdi = new ValueConfigurationDoubleInterval(header);
    const bool isVcValid                   = vcdi->isValid();
    if (!isVcValid)
    {
      HEEX_LOG(error) << "IntervalMonitor::handleValueConfiguration() Invalid input command : '" << cmd << "'" << std::endl;
      delete vcdi;
      return nullptr;
    }
    HEEX_LOG(debug) << "[" << vcdi->getCvUuid() << "] IntervalMonitor::handleValueConfiguration() Value interval : " << vcdi->getLowValue() << "-" << vcdi->getHighValue()
                    << std::endl;
    /// This map will be used to keep track of the detection status of each condition values.
    _isOn[vcdi->getCvUuid()] = false;
    return vcdi;
  }
  else if (HeexUtils::caseInsensitiveStringCompare(header.type, "threshold"))
  {
    ValueConfigurationThreshold* vcthr = new ValueConfigurationThreshold(header);
    const bool isVcValid               = vcthr->isValid();
    if (!isVcValid)
    {
      HEEX_LOG(error) << "IntervalMonitor::handleValueConfiguration() Invalid input command : '" << cmd << "'" << std::endl;
      delete vcthr;
      return nullptr;
    }
    HEEX_LOG(debug) << "[" << vcthr->getCvUuid() << "] IntervalMonitor::handleValueConfiguration() Value : " << vcthr->getValue() << ", Comparison" << vcthr->getComparison()
                    << std::endl;
    /// This map will be used to keep track of the detection status of each condition values.
    _isOn[vcthr->getCvUuid()] = false;
    return vcthr;
  }
  else
  {
    HEEX_LOG(error) << "IntervalMonitor::handleValueConfiguration() Unreconized type : '" << header.type << "'" << std::endl;
  }
  return nullptr;
}

void IntervalMonitor::updateValue(double inputValue)
{
  this->updateValue(inputValue, this->getTimestampStr());
}

void IntervalMonitor::updateValue(double inputValue, const std::string& timestamp)
{
  HEEX_LOG(trace) << "IntervalMonitor::updateValue()" << std::endl;

  // Guard unit compatibility only if targetUnit and signalUnit has been set (from systemConf) and compatible
  if (_signalUnitStr.length() != 0 && _targetUnitStr.length() != 0 && !this->areUnitsCompatible())
  {
    return;
  }

  if (_acceptUnitConversion)
  {
    // convert input value to the unit in the constants block
    inputValue = units::measurement(inputValue * _signalUnit).convert_to(_targetUnit).value();
  }

  // Check timestamp validity. Warn if invalid but passthrough
  if (!Heex::Tools::checkTimestamp(timestamp))
  {
    HEEX_LOG(warning) << "Received invalid timestamp: " << timestamp << std::endl;
  }

  // Handle all ValueConfigurations registered
  for (std::vector<ValueConfiguration*>::iterator it = _valueConfigurations.begin(); it != _valueConfigurations.end(); ++it)
  {
    if (((*it) != nullptr) && HeexUtils::caseInsensitiveStringCompare((*it)->getType(), "interval"))
    {
      ValueConfigurationDoubleInterval* vcdi = dynamic_cast<ValueConfigurationDoubleInterval*>((*it));
      if (vcdi == nullptr)
      {
        HEEX_LOG(error) << "IntervalMonitor::updateValue() : Non-valid ValueConfigurationInterval!" << std::endl;
        return;
      }

      this->updateIntervalLogic(vcdi, inputValue, timestamp);
    }
    else if (((*it) != nullptr) && HeexUtils::caseInsensitiveStringCompare((*it)->getType(), "threshold"))
    {
      ValueConfigurationThreshold* vcthr = dynamic_cast<ValueConfigurationThreshold*>((*it));
      if (vcthr == nullptr)
      {
        HEEX_LOG(error) << "IntervalMonitor::updateValue() : Non-valid ValueConfigurationThreshold!" << std::endl;
        return;
      }

      this->updateThresholdLogic(vcthr, inputValue, timestamp);
    }
  }
}

void IntervalMonitor::updateIntervalLogic(ValueConfigurationDoubleInterval* vcdi, double inputValue, const std::string& timestamp)
{
  HEEX_LOG(debug) << "[" << vcdi->getCvUuid() << "] IntervalMonitor::updateValue() : " << (vcdi->getLowValue() <= inputValue) << " " << (inputValue <= vcdi->getHighValue()) << " "
                  << (_isOn[vcdi->getCvUuid()] == false) << std::endl;
  HEEX_LOG(debug) << "[" << vcdi->getCvUuid() << "] IntervalMonitor::updateValue() : " << vcdi->getLowValue() << " <= " << inputValue << " <= " << vcdi->getHighValue()
                  << std::endl;

  if (vcdi->getLowValue() <= inputValue && vcdi->getHighValue() >= inputValue && _isOn[vcdi->getCvUuid()] == false)
  {
    HEEX_LOG(debug) << "[" << vcdi->getCvUuid() << "] IntervalMonitor::updateValue() : Monitor is ON!" << std::endl;
    this->sendSignalOn(vcdi->getCvUuid(), timestamp);
  }
  else if ((vcdi->getLowValue() <= inputValue && vcdi->getHighValue() >= inputValue) == false && _isOn[vcdi->getCvUuid()] == true)
  {
    HEEX_LOG(debug) << "[" << vcdi->getCvUuid() << "] IntervalMonitor::updateValue() : Monitor is Off" << std::endl;
    this->signalOff((timestamp.size() == 0 ? this->getTimestampStr() : timestamp), vcdi->getCvUuid());
    _isOn[vcdi->getCvUuid()] = false;
  }
}

void IntervalMonitor::updateThresholdLogic(ValueConfigurationThreshold* vcthr, double inputValue, const std::string& timestamp)
{
  HEEX_LOG(debug) << "[" << vcthr->getCvUuid() << "] IntervalMonitor::updateValue() : "
                  << "defined value = " << vcthr->getValue() << ", inputValue =  " << inputValue << std::endl; // DEBUG
  switch (vcthr->getComparison())
  {
    case ValueConfigurationThreshold::Comparison::Equal:
      if (vcthr->getValue() == inputValue && _isOn[vcthr->getCvUuid()] == false)
      {
        HEEX_LOG(debug) << "[" << vcthr->getCvUuid() << "] IntervalMonitor::updateValue() : Monitor is ON!" << std::endl;
        this->sendSignalOn(vcthr->getCvUuid(), timestamp);
      }
      else if (vcthr->getValue() != inputValue && _isOn[vcthr->getCvUuid()] == true)
      {
        HEEX_LOG(debug) << "[" << vcthr->getCvUuid() << "] IntervalMonitor::updateValue() : Monitor is OFF!" << std::endl;
        this->signalOff((timestamp.size() == 0 ? this->getTimestampStr() : timestamp), vcthr->getCvUuid());
        _isOn[vcthr->getCvUuid()] = false;
      }

      break;
    case ValueConfigurationThreshold::Comparison::Above:
      if (vcthr->getValue() < inputValue && _isOn[vcthr->getCvUuid()] == false)
      {
        HEEX_LOG(debug) << "[" << vcthr->getCvUuid() << "] IntervalMonitor::updateValue() : Monitor is ON!" << std::endl;
        this->sendSignalOn(vcthr->getCvUuid(), timestamp);
      }
      else if (vcthr->getValue() >= inputValue && _isOn[vcthr->getCvUuid()] == true)
      {
        HEEX_LOG(debug) << "[" << vcthr->getCvUuid() << "] IntervalMonitor::updateValue() : Monitor is OFF!" << std::endl;
        this->signalOff((timestamp.size() == 0 ? this->getTimestampStr() : timestamp), vcthr->getCvUuid());
        _isOn[vcthr->getCvUuid()] = false;
      }
      break;
    case ValueConfigurationThreshold::Comparison::Below:
      if (vcthr->getValue() > inputValue && _isOn[vcthr->getCvUuid()] == false)
      {
        HEEX_LOG(debug) << "[" << vcthr->getCvUuid() << "] IntervalMonitor::updateValue() : Monitor is ON!" << std::endl;
        this->sendSignalOn(vcthr->getCvUuid(), timestamp);
      }
      else if (vcthr->getValue() <= inputValue && _isOn[vcthr->getCvUuid()] == true)
      {
        HEEX_LOG(debug) << "[" << vcthr->getCvUuid() << "] IntervalMonitor::updateValue() : Monitor is OFF!" << std::endl;
        this->signalOff((timestamp.size() == 0 ? this->getTimestampStr() : timestamp), vcthr->getCvUuid());
        _isOn[vcthr->getCvUuid()] = false;
      }
      break;
    default:
      HEEX_LOG(error) << "[" << vcthr->getCvUuid() << "] BAD ValueConfiguration::Comparison type." << std::endl;
      break;
  }
}

void IntervalMonitor::onServerDisconnection()
{
  Agent::onServerDisconnection();
  /// We clear our track of the detection status of each condition values as we will get new ones whenever the server is back up.
  _isOn.clear();
}

void IntervalMonitor::sendSignalOn(const std::string cvUuid, std::string timestamp)
{
  if (this->getSignalType() == MonitorSignalType::OnOff)
  {
    this->signalOnOff((timestamp.size() == 0 ? this->getTimestampStr() : timestamp), cvUuid);
  }
  else
  {
    this->signalOn((timestamp.size() == 0 ? this->getTimestampStr() : timestamp), cvUuid);
    _isOn[cvUuid] = true;
  }
}

void IntervalMonitor::setSignalUnit(std::string unit)
{
  _signalUnit    = units::unit_cast_from_string(unit);
  _signalUnitStr = unit;

  // If input and target units are not compatibles, don't accept unit conversion and return
  _acceptUnitConversion = this->areUnitsCompatible();
}

std::string IntervalMonitor::getSignalUnit()
{
  return _signalUnitStr;
}

void IntervalMonitor::onConfigurationChanged(const std::string& msg)
{
  Monitor::onConfigurationChanged(msg);

  const std::unordered_map<std::string, std::vector<std::string>>::iterator unitIt = _constantsValues.find(Heex::HEEX_UNIT_KEY);

  // If unit constant is defined, support unit conversion (by setting the target unit)
  if (unitIt != _constantsValues.end())
  {
    _targetUnitStr = unitIt->second.at(0);
    _targetUnit    = units::unit_cast_from_string(_targetUnitStr);
    HEEX_LOG(info) << "Setting required unit to " << _targetUnitStr << " for " << this->getUuid();
  }

  // whenever a new target unit constant is received we will recheck the compatibility
  _acceptUnitConversion = this->areUnitsCompatible();
}

bool IntervalMonitor::areUnitsCompatible()
{
  if (_targetUnitStr.length() == 0)
  {
    return false;
  }
  if (!_signalUnit.has_same_base(_targetUnit))
  {
    HEEX_LOG(error) << "Cannot convert " << _signalUnitStr << " to " << _targetUnitStr << " as they are not compatible";
    return false;
  }
  return true;
}
