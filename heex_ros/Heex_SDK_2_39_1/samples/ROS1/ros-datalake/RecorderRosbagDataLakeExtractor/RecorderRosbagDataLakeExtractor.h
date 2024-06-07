///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file RecorderRosbagDataLakeExtractor.h
/// @author Matthieu Carré (matthieu@heex.io)
/// @brief Header file for the Recorder that implements the extraction services for the HeexSDK. It implements the rosbag offline extraction vith position value as ContextValues and path to the extracted bagfile as EventRecordingPart.
/// @version 0.1
/// @date 2022-04-21
#pragma once

#include "Recorder.h"

class RecorderRosbagDataLakeExtractor : public Recorder
{
public:
  /// @brief Construct a new Recorder Rosbag Data Lake Extractor object
  ///
  /// @param uuid Unique identifier for the Recorder. Shall be a string starting with 'R-' and finishing with a version-4 UUID.
  /// @param serverIp Ip of the Heex Core to which the Recorder will connect, identify and notify
  /// @param serverPort Port of the Heex Core dedicated to the Writer Wrapper (WW) module
  /// @param recorderInputBag Filepath to the input bag
  /// @param recorderOutputDir Folderpath to the output directory
  /// @param recorderPositionTopic Name of the position Ros topic to extract position from
  /// @param topics_tf Names of the Ros topics to extract and place at the start of the bag (eg:)
  RecorderRosbagDataLakeExtractor(
      const std::string& uuid,
      const std::string& serverIp,
      const unsigned int& serverPort,
      const std::string& recorderInputBag,
      std::string& recorderOutputDir,
      std::string& recorderPositionTopic,
      std::vector<std::string> topics_tf);

protected:
  //
  // Methods overriding the default Recorder operations for ContextValues and EventRecordingPart generation
  //

  /// @brief Returns the value as Heex::RecorderArgs::ContextValue for the advertized context by the Recorder (e.g. position). This method is virtual and by default return an empty string standing for error. It requires to be defined within any subclasses to the address the logic of getting data value advertized by your Recorder.
  /// Any value with an empty string will be recognized as an error.
  ///
  /// @param query Current RecorderContextValueArgs-structured query that holds the arguments of the request of ContextValue extraction.
  /// @param contextValues The pass-by-value contextValues variable that contains the key and values of ContextValues advertized by the Recorder. Edit it using the addContextValue(contextValues, "key", "value"). E.g. A context value named "position" would return the value "48.8582651,2.2938142".
  ///
  /// @return true Notify that the generation has completed with success.
  /// @return false Notify that the generation has failed and ignore the result.
  virtual bool generateRequestedValues(const Heex::RecorderArgs::RecorderContextValueArgs& query, std::vector<Heex::RecorderArgs::ContextValue>& contextValues);

  /// @brief Returns the value as std::string of the filepath pointing to the extracted data file or folder. This method is virtual and requires to be defined within any subclasses to the logic of getting data filepath advertized by your Recorder.
  /// Any empty string will be recognized as an error.
  ///
  /// @param query Current RecorderContextValueArgs-typed query that holds the arguments of the request of ContextValue extraction.
  /// @param filepath The pass-by-value filepath variable shall be set to the event recording part filepath. It shall correspond to the actual recorded data on the machine. E.g. A filepath would return the value "/tmp/my_file.txt". We encourage to limit the use of special character like spaces within the filepath. Special characters ";" and ":" are stricly forbidden.
  ///
  /// @return true Notify that the generation has completed with success.
  /// @return false Notify that the generation has failed and ignore the result.
  virtual bool generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath);

  //
  // Utility methods
  //

  /// @brief Extract a recording (DataFile) from the datalake (path set in .conf file) using the Rosbag API
  ///
  /// @param query Query element containing information relative to the extraction to perform
  /// @return std::string Return the filepath to the generated bag cut as specified in the query.
  std::string executeLakeExtractorRosbagAPI(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query);

  /// @brief Get the Position Value From File object
  ///
  /// @param query Query element containing information relative to the extraction to perform
  /// @return std::string Return the position the closest to the timestamp specified in the query.
  std::string getPositionValueFromFile(const Heex::RecorderArgs::RecorderContextValueArgs& query);

  //
  // Attributes
  //
  std::string _recorderInputBag;  ///<Path to the datalake bag
  std::string _recorderOutputDir; ///<Path to the output directory
  /// Name of the topic containing the position values of type sensor_msgs/NavSatFix.h
  std::string _recorderPositionTopic;
  /// List topics that need their messages to be extracted and copied at the start of the bag (eg. tf_static).
  std::vector<std::string> _topics_tf;
};
