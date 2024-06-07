///
/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "featureSampleRecorder_ConfChangedCallback.h"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>

#include "HeexUtilsFileOperations.h"
#include "HeexUtilsLog.h"

//! constructor
Heex::Sample::SampleRecorderConfChangedCallback::SampleRecorderConfChangedCallback(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort)
    : Recorder(uuid, serverIp, serverPort)
{
  HEEX_LOG(info) << "sampleRecorderConfChangedCallback | instance created: " << uuid << " " << serverIp << ":" << serverPort << std::endl;
}

//! [Tested Feature] Callback function that shall be called during each configuration change
void Heex::Sample::SampleRecorderConfChangedCallback::onConfigurationChangedCallback()
{
  // NOTE: This function allows user to run commands every time the recorder has been reconfigured
  // NOTE: In this sample, we shall get the Recorder's new constant values and valueConfigurations, and print them out, but many other use cases are possible.
  // Let's get the new ValueConfigurations and Constant Values
  std::vector<ValueConfiguration*> valueConfs                           = this->getValueConfigurations();
  std::unordered_map<std::string, std::vector<std::string>> constValues = this->getConstantValues();

  // Now we print out all the ValueCOnfigurations if there are any
  if (valueConfs.size() > 0)
  {
    HEEX_LOG(info) << "\nRecorders's value confs are: " << std::endl;
    for (int i = 0; i < valueConfs.size(); i++)
    {
      HEEX_LOG(info) << "\nValue Configuration " << i << " :" << std::endl;
      HEEX_LOG(info) << " - Name     : " << valueConfs[i]->getName() << std::endl;
      HEEX_LOG(info) << " - Type     : " << valueConfs[i]->getType() << std::endl;
      HEEX_LOG(info) << " - UUID     : " << valueConfs[i]->getUuid() << std::endl;
      HEEX_LOG(info) << " - IsValid? : " << valueConfs[i]->isValid() << std::endl;
    }
  }

  // And finally we print out the Constant Values if there are any
  if (constValues.size() > 0)
  {
    HEEX_LOG(info) << "Recorder's constant values are: " << std::endl;
    for (const auto& it : constValues)
    {
      std::stringstream keys;
      if (!it.second.empty())
      {
        for (const auto& val : it.second)
        {
          keys << val << "  ,  ";
        }
        // Get the last value without the trailing comma
        keys << it.second.back();
      }
      HEEX_LOG(info) << " - " << it.first << " : " << keys.str() << std::endl;
    }
  }
}

//! Override the virtual function of the Recorder class to define the logic of getting data values advertized by the SampleRecorder.
bool Heex::Sample::SampleRecorderConfChangedCallback::generateRequestedValues(
    const Heex::RecorderArgs::RecorderContextValueArgs& /*query*/,
    std::vector<Heex::RecorderArgs::ContextValue>& contextValues)
{
  // NOTE: this function has the same basic behavior as what was done during the onboarding's 'get-started' example.
  //! this will add the given position as a context values.
  const double latitude  = 37.795739190321555;
  const double longitude = -122.42351359240077;
  this->addGNSSContextValue(contextValues, "position", latitude, longitude);

  // Adding additional custom context values
  this->addContextValue(contextValues, "myCustomKey", "myCustomValue");

  return true;
}

//! Override the virtual function of the Recorder class to define the logic of getting data file paths.
bool Heex::Sample::SampleRecorderConfChangedCallback::generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath)
{
  // NOTE: In this example, we shall create and upload a simple text file that contains "This is Smart Data" and a timestamp in it.
  // NOTE: we are handeling all paths to comply with UTF-8 chars, but if you are only using ASCII characters it is not mandatory

  // Step 1: Define the dedicated file name for this recorder.
  // Generate a unique name to avoid file name collision when multiple recorders write in same folder
  std::string recordingFilename = "file-" + query.uuid + query.eventUuid + ".txt";
  recordingFilename             = HeexUtils::FileOperations::getUtf8EncodedPath(boost::filesystem::system_complete(recordingFilename));

  // Step 2: Write data in the recording's text file
  HEEX_LOG(info) << "sampleRecorderConfChangedCallback | Creating file at : "
                 << HeexUtils::FileOperations::getUtf8EncodedPath(boost::filesystem::system_complete(recordingFilename)) << std::endl;
  std::ofstream ofs = HeexUtils::FileOperations::getOfstream(recordingFilename);
  ofs << "Empty recording mock data for timestamp: " << query.timestamp << "\n";
  ofs.close();
  HEEX_LOG(info) << "sampleRecorderConfChangedCallback | File created." << std::endl;

  // Step 3: Return the path to the event recording file.
  // NOTE: the user has 3 options during this step. Instead of pointing to the given file, you can chose to point to a whole folder,
  // NOTE: or you can ask for the content of a whole folder to be uploaded without the folder itself, by adding a /* at the end of the path.
  filepath = HeexUtils::FileOperations::getUtf8EncodedPath(boost::filesystem::canonical(recordingFilename)); // absolute path
  HEEX_LOG(info) << "sampleRecorderConfChangedCallback | Recording file: " << filepath << std::endl;

  // The filepath been successfully generated and assigned to the `filepath` variable.
  return true;
}

int main(int argc, char** argv)
{
  HEEX_LOG(info) << "Heex SDK version : " << Heex::HEEX_SYSTEM_SDK_VERSION << std::endl;

  // TODO: Replace this left Recorder UUID with the one provided (by the Web Platform) for your Trigger or give a UUID as an input to the generated executable
  std::string recorderUuid = "R-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)";
  if (argc > 1) // Parse the UUID as an optional parameter.
  {
    recorderUuid = argv[1];
  }
  const std::string serverIp    = "127.0.0.1";
  const unsigned int serverPort = 4243;

  HEEX_LOG(info) << argv[0] << " will be exposed as " << recorderUuid << " tries to connect to " << serverIp << ":" << serverPort << std::endl;

  // configure and launch the Monitor
  // NOTE: at the creation of the instance, the initial configuration will trigger the onConfigurationChangedCallback to be called.
  // NOTE: Any future reconfiguration (ie: OTA) will also trigger it.
  Heex::Sample::SampleRecorderConfChangedCallback sampleRecorderInstance(recorderUuid, serverIp, serverPort);
  sampleRecorderInstance.awaitReady(); // awaits the connection with the kernel.

  while (true) // wait for request indefinitly, until user break or kernel break
  {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  return EXIT_SUCCESS;
}
