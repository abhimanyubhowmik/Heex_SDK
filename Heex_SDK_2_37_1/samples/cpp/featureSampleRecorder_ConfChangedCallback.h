///
/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include <string>

#include "Recorder.h"

namespace Heex
{
namespace Sample
{
class SampleRecorderConfChangedCallback : public Recorder
{
  /// @brief This SampleRecorderConfChangedCallback class, which inherits from Heex's agent Recorder class, is created to
  /// demonstrate how the feature onConfigurationChangedCallback() can be used. The recorder behavior's part is the same as
  /// the one in the basicSampleRecorder
public:
  /// A default constructor with forced initialization values
  SampleRecorderConfChangedCallback(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort);

private:
  /// Override the virtual function of the Recorder class to define the logic of getting data values advertized by the SampleRecorder.
  virtual bool generateRequestedValues(const Heex::RecorderArgs::RecorderContextValueArgs& query, std::vector<Heex::RecorderArgs::ContextValue>& contextValues) override;

  /// Override the virtual function of the Recorder class to define the logic of getting data file paths.
  virtual bool generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath) override;

  /// [Tested Feature] Overridable callback function that is be called during each configuration change.
  /// @brief In this sample, every time there is a new reconfiguration (and at initial configuration), we shall print out all the monitor's valueConfigurations and Constant values.
  virtual void onConfigurationChangedCallback() override;
};
} // namespace Sample
} // namespace Heex
