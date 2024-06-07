///
/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "basicSampleRecorder.h"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <thread>

#include "HeexUtilsFileOperations.h"
#include "HeexUtilsLog.h"

Heex::Sample::SampleRecorder::SampleRecorder(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort) : Recorder(uuid, serverIp, serverPort)
{
  HEEX_LOG(info) << "SampleRecorder | instance created: " << uuid << " " << serverIp << ":" << serverPort << std::endl;
}

//! Override the virtual function of the Recorder class to define the logic of getting data values advertized by the SampleRecorder.
bool Heex::Sample::SampleRecorder::generateRequestedValues(
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
bool Heex::Sample::SampleRecorder::generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath)
{
  // NOTE: In this basic example, we shall create a simple text file that contains "This is Smart Data" and a timestamp in it.
  // NOTE: We will then send the whole folder, 'RecordingCache', containing this text file to the cloud as our recording sample
  // NOTE: we are handeling all paths to comply with UTF-8 chars, but if you are only using ASCII characters it is not mandatory

  const boost::filesystem::path relativePath{HeexUtils::FileOperations::getCorrectlyEncodedPath("RecorderCache")};
  boost::system::error_code ec;

  // Step 1: First we create the directory if it doesn't exist
  boost::filesystem::create_directory(relativePath, ec);
  if (ec.value() != boost::system::errc::success && ec.value() != boost::system::errc::file_exists)
  {
    HEEX_LOG(error) << "SampleRecorder | Can't create folder at " + HeexUtils::FileOperations::getUtf8EncodedPath(relativePath.parent_path()) +
                           ". Filesystem error caused by: " + ec.message()
                    << std::endl;
    return false;
  }
  ec.clear();
  HEEX_LOG(info) << "SampleRecorder | Using operating directory: " << HeexUtils::FileOperations::getUtf8EncodedPath(boost::filesystem::system_complete(relativePath)) << std::endl;

  // Step 2: Define the dedicated file name for this recorder.
  // Generate a unique name to avoid file name collision when multiple recorders write in same folder
  std::string recordingEventPartFilename = "file-" + query.uuid + query.eventUuid + ".txt";
  boost::filesystem::path recFile        = relativePath / recordingEventPartFilename;

  // Step 3: Write data in the recording's text file
  HEEX_LOG(info) << "SampleRecorder | Creating file at : " << HeexUtils::FileOperations::getUtf8EncodedPath(boost::filesystem::system_complete(recFile)) << std::endl;
  std::ofstream ofs = HeexUtils::FileOperations::getOfstream(recFile);
  ofs << "Empty recording mock data for timestamp: " << query.timestamp << "\n";
  ofs.close();
  HEEX_LOG(info) << "SampleRecorder | File created." << std::endl;

  // Step 4: Return the path to the event recording folder.
  // NOTE: the user has 3 options during this step. Instead of sending the file only, you can chose to point to a whole folder,
  // NOTE: or you can ask for the content of the whole folder to be uploaded without the folder itself, by adding a /* at the end of the path.
  filepath = HeexUtils::FileOperations::getUtf8EncodedPath(boost::filesystem::canonical(recFile)); // absolute path
  HEEX_LOG(info) << "SampleRecorder | Recording folder: " << filepath << std::endl;

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
  Heex::Sample::SampleRecorder sampleRecorderInstance(recorderUuid, serverIp, serverPort);
  sampleRecorderInstance.awaitReady(); // awaits the connection with the kernel.

  while (true) // wait for request indefinitly, until user break or kernel break
  {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  return EXIT_SUCCESS;
}
