///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "MonitorV2.h"

MonitorV2::MonitorV2(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion, bool autoStartCom)
    : BaseMonitor(uuid, serverIp, serverPort, implementationVersion, autoStartCom)
{
}

void MonitorV2::updateValue(bool, const std::string&)
{
  HEEX_LOG(error) << "This monitorV2 does not have a suitable implementation for the following method : updateValue(bool)";
}

void MonitorV2::updateValue(std::string, const std::string&)
{
  HEEX_LOG(error) << "This monitorV2 does not have a suitable implementation for the following method : updateValue(string)";
}

void MonitorV2::updateValue(double, const std::string&)
{
  HEEX_LOG(error) << "This monitorV2 does not have a suitable implementation for the following method : updateValue(double)";
}

void MonitorV2::updateValue(double, double, const std::string&)
{
  HEEX_LOG(error) << "This monitorV2 does not have a suitable implementation for the following method : updateValue(double double)";
}
