///
/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
///

#pragma once

#include <string>

#include "BooleanMonitor.h"

namespace Heex
{
namespace Sample
{
class sampleMonitorConfChangeCallback : public BooleanMonitor
{
public:
  /// A default constructor with forced initialization values
  sampleMonitorConfChangeCallback(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort);

  /// [Sample Key Feature] Overridable callback function that is be called during each configuration change (called during onConfigurationChanged).
  /// @brief In this sample, every time there is a new reconfiguration (and at initial configuration), we shall print out all the monitor's valueConfigurations and Constant values.
  virtual void onConfigurationChangedCallback() override;
};
} // namespace Sample
} // namespace Heex
