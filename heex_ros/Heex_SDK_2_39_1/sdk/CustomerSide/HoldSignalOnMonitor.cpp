///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file HoldSignalOnMonitor.cpp
///
/// @brief Library source file that defines and implements a HoldSignalOnMonitor structure that send a `signalOn` and `singalOnOff` with a state-machine logic to hold the `On` state during a given time.
///
#include "HoldSignalOnMonitor.h"

#include <boost/bind/bind.hpp> // Should already included from Monitor headers
#include <boost/date_time/posix_time/posix_time.hpp>
#include <chrono>
#include <sstream>

HoldSignalOnMonitor::HoldSignalOnMonitor(
    const std::string& uuid,
    const std::string& serverIp,
    const unsigned int& serverPort,
    const u_int delay,
    const std::string& implementationVersion)
    : Monitor(uuid, serverIp, serverPort, implementationVersion),
      _delay(boost::posix_time::microseconds(delay)),
      _isOn(false)
{
}

HoldSignalOnMonitor::HoldSignalOnMonitor(const std::string& uuid, const std::string& configurationFile, const u_int delay, const std::string& implementationVersion)
    : Monitor(uuid, configurationFile, implementationVersion),
      _delay(boost::posix_time::microseconds(delay)),
      _isOn(false)
{
}

void HoldSignalOnMonitor::signalOn()
{
  this->signalOn(this->getTimestampStr(), _uuid);
}

void HoldSignalOnMonitor::signalOn(const std::string& timestamp)
{
  this->signalOn(timestamp, _uuid);
}

void HoldSignalOnMonitor::signalOn(const std::string& timestamp, const std::string& uuid)
{
  // Check on Monitor state and register time up to delay. Can't retriggered if already in On state is intended.
  if (!_isOn)
  {
    _isOn              = true;
    /// Refers to provided timestamp if not empty string
    _lastestActivation = (timestamp.size() == 0 ? this->getTimestamp() : boost::posix_time::from_iso_extended_string(timestamp));

    if (uuid.size() != 0)
    {
      _cvUuid = uuid;
    }
    else
    {
      _cvUuid = "";
    }

    std::stringstream cmd;
    cmd << "SignalOn " << ((uuid.size() == 0) ? _uuid : uuid) << " " << (timestamp.size() == 0 ? this->convertTimestampToStr(_lastestActivation) : timestamp) << " "
        << Agent::encodeIncidents() << "\n";
    if (_tcpC != nullptr)
    {
      _tcpC->send(cmd.str());
    }
    else
    {
      HEEX_LOG(error) << "Could not send message : " << cmd.str();
    }

    // Start waiting thread
    _thread =
        std::make_unique<std::thread>(std::thread(boost::bind(&HoldSignalOnMonitor::sendSignalOffAfterDelay, this, this->convertTimestampToStr(_lastestActivation + _delay))));
  }
}

void HoldSignalOnMonitor::sendSignalOffAfterDelay(const std::string& timestamp)
{
  if (_isOn)
  {
    HEEX_LOG(debug) << "HoldSignalOnMonitor " << _uuid << " Waiting for " << _delay.total_microseconds() << "µs" << std::endl; //DEBUG
    // Check if release only when timer is expired
    std::this_thread::sleep_for(std::chrono::microseconds(_delay.total_microseconds()));

    this->signalOff((timestamp.size() == 0 ? this->getTimestampStr() : timestamp));
    _isOn = false;
  }
}

void HoldSignalOnMonitor::signalOff(const std::string& timestamp)
{
  std::stringstream cmd;
  cmd << "SignalOff " << ((_cvUuid.size() == 0) ? _uuid : _cvUuid) << " " << (timestamp.size() == 0 ? this->getTimestampStr() : timestamp) << " " << Agent::encodeIncidents()
      << "\n";
  if (_tcpC != nullptr)
  {
    _tcpC->send(cmd.str());
  }
  else
  {
    HEEX_LOG(error) << "Could not send message : " << cmd.str();
  }
}
