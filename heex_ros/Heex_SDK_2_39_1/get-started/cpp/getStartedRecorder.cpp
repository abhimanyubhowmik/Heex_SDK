#include "getStartedRecorder.h"

#include <iostream>
#include <thread>

Heex::GetStarted::GetStartedRecorder::GetStartedRecorder(const std::string& uuid, const std::string& serverIp, unsigned int& serverPort) : Recorder(uuid, serverIp, serverPort)
{
  HEEX_LOG(info) << "GetStartedRecorder | " << uuid << " " << serverIp << " " << serverPort << " " << std::endl;
}

bool Heex::GetStarted::GetStartedRecorder::generateRequestedValues(
    const Heex::RecorderArgs::RecorderContextValueArgs& /*query*/,
    std::vector<Heex::RecorderArgs::ContextValue>& /*contextValues*/)
{
  // TODO: Uncomment and fill the variable `contextValues` with all the values for each of the ContextValue keys in the Recorder query. We recommend to use the addContextValue method. The keys for a Recorder are listed in the Recorder implementation panel.
  // Use this->getContextValueKeys(contextValues) to obtain all required keys from the query (optional check).
  // Use this->addContextValue(contextValues, key, value) to assign the value to the key. Values are defaulted to an empty string.
  // Use this->addGNSSContextValue(contextValues, "position", latitude, longitude) to assign separate latitude and longitude to the key "position".
  // Access query info for Recorder uuid (query.uuid), Event uuid (query.eventUuid) and requested timestamp (query.timestamp). See RecorderQueryContextValueArgs structure.

  // Return true when all the context keys have their values added in `contextValues`.
  return true;
}

bool Heex::GetStarted::GetStartedRecorder::generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& /*query*/, std::string& /*filepath*/)
{
  // TODO: Uncomment and fill the `filepath` variable with your filepath value pointing to your generated event recording part.
  // You can provided a filepath with the desired option. Each of the options has a specific syntax to match a use case:
  //  - 1: Send the file only. Return the direct path to file (no modification required).
  //  - 2: Send the folder and all its content. Return the direct path to folder (no modification required, all folder content will be included).
  //  - 3: Send only the content of the folder. Return the path to the folder with "/*" as an extra suffix (wildcard at the end to include only files within the parent directory).
  // Access query info for Recorder uuid (query.uuid), Event uuid (query.eventUuid), requested timestamp (query.timestamp), and the intervals of time before and after the timestamp (query.recordIntervalStart and query.recordIntervalEnd). See RecorderEventRecordingPartArgs structure.

  // Return true when the path to the event recording part have been replaced in `filepath`. File(s) shall have been completely generated.
  return false;
}

int main()
{
  //
  // Configure the Recorder
  //
  // TODO: Replace following Recorder UUID with the one provided (by the Web Platform) for your Trigger
  const std::string recorderUuid = "R-a14b74aa-70c9-4f18-83ad-1b688c130678(1.0.0)";
  const std::string serverIp     = "127.0.0.1";
  unsigned int serverPort        = 4243;
  Heex::GetStarted::GetStartedRecorder GetStartedRecorderInstance(recorderUuid, serverIp, serverPort);

  //
  // Keep Recorder active while connection with the SDE is active
  //
  bool timeToQuit       = false;
  bool hasBeenConnected = false;
  while (timeToQuit == false)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (GetStartedRecorderInstance.isConnected() == false)
    {
      if (hasBeenConnected == true)
      {
        timeToQuit = true;
      }
    }
    else
    {
      hasBeenConnected = true;
    }
  }

  return 0;
}
