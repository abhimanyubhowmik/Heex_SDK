///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#ifndef HEEX_SDK_NO_BOOST_JSON

  #include "ZoneMonitor.h"

  #include "HeexUtils.h"
  #include "HeexUtilsGnss.h"
  #include "Tools.h"

ZoneMonitor::ZoneMonitor(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion, bool autoStartCom)
    : MonitorV2Interface(uuid, serverIp, serverPort, implementationVersion, autoStartCom)
{
}

ValueConfiguration* ZoneMonitor::handleValueConfiguration(const std::string& cmd, ValueConfHeader& header)
{
  HEEX_LOG(trace) << "ZoneMonitor::handleValueConfiguration()" << std::endl;
  if (HeexUtils::caseInsensitiveStringCompare(header.type, "zone"))
  {
    ValueConfigurationZone* vcz = new ValueConfigurationZone(header);
    const bool isVcValid        = vcz->isValid();
    if (!isVcValid)
    {
      HEEX_LOG(error) << "ZoneMonitor::handleValueConfiguration() Invalid input command : '" << cmd << "'" << std::endl;
      delete vcz;
      return nullptr;
    }
    HEEX_LOG(debug) << "[" << vcz->getCvUuid() << "] ZoneMonitor::handleValueConfiguration(): " << vcz->getCircles().size() + vcz->getPolygons().size() << " zones" << std::endl;
    /// This map will be used to keep track of the previous position with regards to the zone
    _isOn[vcz->getCvUuid()]      = false;
    _wasInside[vcz->getCvUuid()] = false;
    return vcz;
  }
  else
  {
    HEEX_LOG(error) << "ZoneMonitor::handleValueConfiguration() Unreconized type : '" << header.type << "'" << std::endl;
  }
  return nullptr;
}

void ZoneMonitor::onConfigurationChanged(const std::string& msg)
{
  Monitor::onConfigurationChanged(msg);
}

void ZoneMonitor::updateValue(double latitude, double longitude)
{
  this->updateValue(latitude, longitude, this->getTimestampStr());
}

void ZoneMonitor::updateValue(double latitude, double longitude, const std::string& timestamp)
{
  HEEX_LOG(trace) << "ZoneMonitor::updateValue()" << std::endl;

  // Check timestamp validity. Warn if invalid but passthrough
  if (!Heex::Tools::checkTimestamp(timestamp))
  {
    HEEX_LOG(warning) << "Received invalid timestamp: " << timestamp << std::endl;
  }

  // Handle all ValueConfigurations registered
  for (std::vector<ValueConfiguration*>::iterator it = _valueConfigurations.begin(); it != _valueConfigurations.end(); ++it)
  {
    if (((*it) != nullptr) && HeexUtils::caseInsensitiveStringCompare((*it)->getType(), "zone"))
    {
      ValueConfigurationZone* vcz = dynamic_cast<ValueConfigurationZone*>((*it));
      bool isInside               = false;
      ///<! Currently we only support a single behavior for all the zones. Future developpement may set this per zone, but for now we shall
      ///<! define the behavior at this level and assume all the behaviors per zone have the same value and allow overwrite at every loop.
      ZoneBehavior behavior       = ZoneBehavior::In;
      if (vcz != nullptr)
      {
        for (const ZoneCircle& c : vcz->getCircles())
        {
          behavior = c.behavior;
          if (!isInside && HeexUtils::isInsideCircle(latitude, longitude, c.center.first, c.center.second, c.radius))
          {
            HEEX_LOG(debug) << "[" << vcz->getCvUuid() << "] ZoneMonitor::updateValue: point (" << latitude << ", " << longitude << ") is inside the circle" << std::endl;
            isInside = true;
            break;
          }
        }
        for (const ZonePolygon& p : vcz->getPolygons())
        {
          behavior = p.behavior;
          if (!isInside && HeexUtils::isPointInPoly(latitude, longitude, p.points))
          {
            HEEX_LOG(debug) << "[" << vcz->getCvUuid() << "] ZoneMonitor::updateValue: point (" << latitude << ", " << longitude << ") is inside the polygon" << std::endl;
            isInside = true;
            break;
          }
        }

        // clang-format off
        if ( (isInside && (behavior == ZoneBehavior::In))                                               || 
             (isInside && (behavior == ZoneBehavior::Enter) && (_wasInside[vcz->getCvUuid()] == false)) || 
             (!isInside && (behavior == ZoneBehavior::Out))                                             || 
             (!isInside && (behavior == ZoneBehavior::Exit) && (_wasInside[vcz->getCvUuid()] == true))
           )
        // clang-format on
        {
          HEEX_LOG(debug) << "[" << vcz->getCvUuid() << "] ZoneMonitor::updateValue() : Monitor is ON!" << std::endl;
          this->signalOn((timestamp.size() == 0 ? this->getTimestampStr() : timestamp), vcz->getCvUuid());
          _isOn[vcz->getCvUuid()] = true;
        }
        else if (_isOn[vcz->getCvUuid()]) // If signal was on, we send signalOff once.
        {
          HEEX_LOG(debug) << "[" << vcz->getCvUuid() << "] ZoneMonitor::updateValue() : Monitor is OFF!" << std::endl;
          this->signalOff((timestamp.size() == 0 ? this->getTimestampStr() : timestamp), vcz->getCvUuid());
          _isOn[vcz->getCvUuid()] = false;
        }
        _wasInside[vcz->getCvUuid()] = isInside;
      }
      else
      {
        HEEX_LOG(error) << "ZoneMonitor::updateValue() : Non-valid ValueConfigurationZone!" << std::endl;
      }
    }
  }
}

void ZoneMonitor::onServerDisconnection()
{
  Agent::onServerDisconnection();
  /// We clear our track of the detection status of each condition values as we will get new ones whenever the server is back up.
  _isOn.clear();
  _wasInside.clear();
}

#endif
