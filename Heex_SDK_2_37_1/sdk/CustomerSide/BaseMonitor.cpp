﻿///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file BaseMonitor.cpp
///
/// @brief Library source file that defines and implements a Base Monitor structure that can send `signalOn` or `signalOff` signals to the Heex Smart Data Engine.
///
#include "BaseMonitor.h"

#include <sstream>

BaseMonitor::BaseMonitor(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion, bool autoStartCom)
    : Monitor(uuid, serverIp, serverPort, implementationVersion, autoStartCom)
{
}

BaseMonitor::BaseMonitor(const std::string& uuid, const std::string& configurationFile, const std::string& implementationVersion)
    : Monitor(uuid, configurationFile, implementationVersion)
{
}

void BaseMonitor::signalOn()
{
  this->signalOn(this->getTimestampStr(), _uuid);
}

void BaseMonitor::signalOn(const std::string& timestamp)
{
  this->signalOn(timestamp, _uuid);
}

void BaseMonitor::signalOn(const std::string& timestamp, const std::string& uuid)
{
  std::stringstream cmd;
  cmd << "SignalOn " << ((uuid.size() == 0) ? _uuid : uuid) << " " << (timestamp.size() == 0 ? this->getTimestampStr() : timestamp) << " " << Agent::encodeIncidents() << "\n";
  if (_tcpC != nullptr)
  {
    _tcpC->send(cmd.str());
  }
  else
  {
    HEEX_LOG(error) << "Could not send message : " << cmd.str();
  }
}

void BaseMonitor::signalOff()
{
  this->signalOff(this->getTimestampStr(), _uuid);
}

void BaseMonitor::signalOff(const std::string& timestamp)
{
  this->signalOff(timestamp, _uuid);
}

void BaseMonitor::signalOff(const std::string& timestamp, const std::string& uuid)
{
  std::stringstream cmd;
  cmd << "SignalOff " << ((uuid.size() == 0) ? _uuid : uuid) << " " << (timestamp.size() == 0 ? this->getTimestampStr() : timestamp) << " " << Agent::encodeIncidents() << "\n";
  if (_tcpC != nullptr)
  {
    _tcpC->send(cmd.str());
  }
  else
  {
    HEEX_LOG(error) << "Could not send message : " << cmd.str();
  }
}

void BaseMonitor::signalOnOff()
{
  this->signalOnOff(this->getTimestampStr(), _uuid);
}

void BaseMonitor::signalOnOff(const std::string& timestamp)
{
  this->signalOnOff(timestamp, _uuid);
}

void BaseMonitor::signalOnOff(const std::string& timestamp, const std::string& uuid)
{
  std::stringstream cmd;
  cmd << "SignalOnOff " << ((uuid.size() == 0) ? _uuid : uuid) << " " << (timestamp.size() == 0 ? this->getTimestampStr() : timestamp) << " " << Agent::encodeIncidents() << "\n";
  _tcpC->send(cmd.str());
}