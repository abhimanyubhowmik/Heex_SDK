#pragma once

#include <string>

#include "Recorder.h"
#include "RecorderArgs.h"

namespace Heex
{
namespace GetStarted
{
class GetStartedRecorder : public Recorder
{
public:
  /// A default constructor with forced initialization values
  GetStartedRecorder(const std::string& uuid, const std::string& serverIp, unsigned int& serverPort);

private:
  /// Override the pure virtual function of the Recorder class to define the logic of getting data values advertized by the GetStartedRecorder. See Recorder::generateRequestedValues for more documentation.
  virtual bool generateRequestedValues(const Heex::RecorderArgs::RecorderContextValueArgs& query, std::vector<Heex::RecorderArgs::ContextValue>& contextValues);

  /// Override the pure virtual function of the Recorder class to define the logic of getting data file paths. See Recorder::generateRequestedValues for more documentation.
  virtual bool generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath);
};
} // namespace GetStarted
} // namespace Heex
