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
/// @brief This SampleRecorder class, which inherits from Heex's agent Recorder class, is created to demonstrate how a
/// basic example of a recorder can be implemented and used
class SampleRecorder : public Recorder
{
public:
  /// A default constructor with forced initialization values
  SampleRecorder(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort);

private:
  /// Override the virtual function of the Recorder class to define the logic of getting data values advertized by the SampleRecorder.
  virtual bool generateRequestedValues(const Heex::RecorderArgs::RecorderContextValueArgs& query, std::vector<Heex::RecorderArgs::ContextValue>& contextValues) override;

  /// Override the virtual function of the Recorder class to define the logic of getting data file paths.
  virtual bool generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath) override;
};
} // namespace Sample
} // namespace Heex
