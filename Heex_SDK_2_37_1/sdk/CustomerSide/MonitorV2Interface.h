///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include <iostream>

#include "MonitorV2.h"

template <class T> struct MonitorV2Interface : public MonitorV2
{
  MonitorV2Interface(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion, bool autoStartCom)
      : MonitorV2(uuid, serverIp, serverPort, implementationVersion, autoStartCom)
  {
    (void)_registered;
  }

  virtual ~MonitorV2Interface()
  {
    if (!_registered)
    {
      HEEX_LOG(debug) << "Not registered MonitorV2Interface";
    }
  } // registerd_ needs to be accessed somewhere, otherwise it is opted away

  static bool registerType()
  {
    std::map<std::string, CreateFunc>& registry = getRegistry(); // NOLINT
    registry[T::getFactoryName()]               = T::createFunc;
    return true;
  }

  static const bool _registered;

protected:
  virtual void doNotInheritFromMonitorV2DirectlyButFromMonitorV2InterfaceInstead() final {}
};

template <class T> const bool MonitorV2Interface<T>::_registered = MonitorV2Interface<T>::registerType();
