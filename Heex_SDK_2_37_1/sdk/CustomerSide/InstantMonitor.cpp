///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file InstantMonitor.cpp
///
/// @brief Library source file that defines and implements a InstantMonitor structure that only sends `signalOnOff` to the Heex Smart Data Engin
///
#include "InstantMonitor.h"

#include <sstream>

InstantMonitor::InstantMonitor(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion, bool autoStartCom)
    : Monitor(uuid, serverIp, serverPort, implementationVersion, autoStartCom)
{
}

InstantMonitor::InstantMonitor(const std::string& uuid, const std::string& configurationFile, const std::string& implementationVersion)
    : Monitor(uuid, configurationFile, implementationVersion)
{
}

void InstantMonitor::signalOnOff()
{
  this->signalOnOff(this->getTimestampStr(), _uuid);
}

void InstantMonitor::signalOnOff(const std::string& timestamp)
{
  this->signalOnOff(timestamp, _uuid);
}

void InstantMonitor::signalOnOff(const std::string& timestamp, const std::string& uuid)
{
  std::stringstream cmd;
  cmd << "SignalOnOff " << ((uuid.size() == 0) ? _uuid : uuid) << " " << (timestamp.size() == 0 ? this->getTimestampStr() : timestamp) << " " << Agent::encodeIncidents() << "\n";
  if (_tcpC != nullptr)
  {
    _tcpC->send(cmd.str());
  }
  else
  {
    HEEX_LOG(error) << "Could not send message : " << cmd.str();
  }
}
