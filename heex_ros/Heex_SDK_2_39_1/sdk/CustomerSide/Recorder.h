///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file Recorder.h
/// @brief Library header file that contains the basic structure for any Recorders to appropriately register and communicate with the Smart Data Engine. Works for both Real-time and Datalake applications. *
#pragma once

#include <mutex>
#include <vector>

#include "RecorderArgs.h"
#include "RecorderV2.h"

/// @brief This Recorder class defines the basic structure of any basic Recorder **agents** to deploy recording solutions with an one-way interface between the customer edge system (data streams from vehicle, replay, simulation, etc.) to the Heex Smart Data Engine. This class shall not be used directly but aims to be extended with methods to provided data extraction functionnalities on advertized contexts orchestrated by the the Heex SDE. This distinct scope of control offers modularity in the way of managing a set of possible recording strategies (Recorder sub-classes) and components as reusable and/or reconfigurable code by the Heex Smart Data Engine.
/// Advertize contexts can be as values with the `ContextValue` query type (e.g. value for position such as "48.8582651,2.2938142") or as filepath with the `DataFilePath` querty type (e.g. filepath point to a file that stores all the values of the position during the recording time interval stored).
///
/// This file defines the main class a Recorder component shall inherit to gain all Heex utilitary process functions and communication stacks. The recording logic is at the discretion of its sub-classes with the definition and implementation of the generateRequestedValues() and generateRequestedFilePaths() functions.
/// If one of the generateRequestedValues() and generateRequestedFilePaths() function aren't used in the sub-class of Recorder, consider defining an empty implementation.
///
/// The Recorder is in charge of managing its output by creating it and deleting it when requested. Some circular file management system may be required for a lightweigth use.
///
/// The time-related methods in the Recorder class aim to provide a default timestamping system that match Heex Messages specifications. They can be overridden for systems with a different clock or a dedicated time service.
class Recorder : public RecorderV2
{
public:
  /// @brief Constructor for Recorder with its full configuration. It requires the uuid and the networking settings for initialization.
  ///
  /// @param uuid Unique identifier for the Recorder. Shall be a string starting with 'R-' and finishing with a version-4 UUID.
  /// @param serverIp Ip of the Heex Core to which the Recorder will connect, identify and notify
  /// @param serverPort Port of the Heex Core dedicated to the Writer Wrapper (WW) module
  /// @param implementationVersion Implementation version of the recorder. Transmitted to the SDE during agent identification process.
  Recorder(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION);

  /// @brief Deconstructor for Recorder to free memory appropriately. In this case, it destroys the TcpClient pointer.
  ~Recorder();

  ///
  /// @brief Check whether the recorder is performing requests or not
  ///
  /// @return true if the recorder is still performing requests
  /// @return false if the recorder is done performing requests
  bool isPerformingCallback();

  const std::vector<Heex::RecorderArgs::RecorderRangesValues>& getRecorderRangesRegistry();

protected:
  /// @brief Returns the value as std::string for the advertized context by the Recorder (e.g. position). This method is virtual and by default return an empty string standing for error. It requires to be defined within any subclasses to the address the logic of getting data value advertized by your Recorder.
  /// Any value with an empty string will be recognized as an error.
  ///
  /// @param query Current RecorderContextValueArgs-structured query that holds the arguments of the request of ContextValue extraction.
  /// @param contextValues The pass-by-reference contextValues variable that contains the key and values of ContextValues advertized by the Recorder. Edit it using the addContextValue(contextValues, "key", "value"). E.g. A context value named "position" would return the value "48.8582651,2.2938142".
  ///
  /// @return true Notify that the generation has completed with success.
  /// @return false Notify that the generation has failed and ignore the result.
  virtual bool generateRequestedValues(const Heex::RecorderArgs::RecorderContextValueArgs& query, std::vector<Heex::RecorderArgs::ContextValue>& contextValues);

  /// @brief Returns the value as std::string of the filepath pointing to the extracted data file or folder. This method is virtual and requires to be defined within any subclasses to the logic of getting data filepath advertized by your Recorder.
  /// Any empty string will be recognized as an error.
  ///
  /// @param query Current RecorderContextValueArgs-typed query that holds the arguments of the request of ContextValue extraction.
  /// @param filepath The pass-by-reference filepath variable shall be set to the event recording part filepath. It shall correspond to the actual recorded data on the machine. E.g. A filepath would return the value "/tmp/my_file.txt". We encourage to limit the use of special character like spaces within the filepath. Special characters ";" and ":" are stricly forbidden.
  ///
  /// @return true Notify that the generation has completed with success.
  /// @return false Notify that the generation has failed and ignore the result.
  virtual bool generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath);

  /// method that handle compatibility of RecorderV1 using RecorderV2 calls.
  virtual bool generateInstantRecordings(const Heex::RecorderArgs::RecorderArgsV2& query);

  /// method that handle compatibility of RecorderV1 using RecorderV2 calls.
  virtual bool generateTimeFrameRecordings(const Heex::RecorderArgs::RecorderArgsV2& query);

  /// @brief Generate a vector to store the Context Values. Uses the method addContextValue(contextValues, key, value)
  ///
  /// @param query Current RecorderContextValueArgs-typed query that holds the arguments of the request of ContextValue extraction.
  /// @return std::vector<std::string> Context Values with empty value field.
  std::vector<Heex::RecorderArgs::ContextValue> prepareContextValues(const Heex::RecorderArgs::RecorderContextValueArgs& query);

  /// @brief Add the value to the provided key in the ContextValue variable. Add key and value at the end of contextValues if key does not exist.
  ///
  /// @param res ContextValue variable. Pass-by-reference.
  /// @param key Key to add value for
  /// @param value Value to add
  /// @return true The value has been added with success for the provided key.
  /// @return false The value hasn't been added as an error has occurred.
  bool addContextValue(std::vector<Heex::RecorderArgs::ContextValue>& res, const std::string key, const std::string value);

  /// @brief Add the GNSS position for the "position" key using latitude and longitude values. Generate under-the-hood a formatted string following heex formatting and add it to the ContextValue variable.
  ///
  /// @param res ContextValue variable. Pass-by-reference.
  /// @param latitude Latitude
  /// @param longitude Longitude
  /// @return true The value has been added with success for the position key.
  /// @return false The value hasn't been added as an error has occurred.
  bool addGNSSContextValue(std::vector<Heex::RecorderArgs::ContextValue>& res, const std::string key, const double latitude, const double longitude);

  /// @brief Return the keys required by the query
  ///
  /// @param query Query to extract keys from
  /// @return std::vector<std::string> List the required keys
  std::vector<std::string> getContextValueKeys(const Heex::RecorderArgs::RecorderContextValueArgs& query);

  /// @brief Update the recording ranges with the ones provided. Use this function if you want to return information regarding how your Recorder has cut time.
  ///
  /// @param query Original Query
  /// @param realRecordIntervalStart Value for the left side of the recording range (start).
  /// @param realRecordIntervalEnd  Value for the right side of the recording range (end).
  /// @return true The value has been added with success for the provided query.
  /// @return false The value hasn't been added as an error has occurred.
  bool setRealRecordIntervalRange(
      const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query,
      const std::string& realRecordIntervalStart,
      const std::string& realRecordIntervalEnd);

  /// Create an answer in the answer queue for the provided query
  void addNewEventRecordingPartAnswer(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query);

  /// Extract and remove the answer from the EventRecordingPart answers preparation queue. Removal can be disabled.
  const Heex::RecorderArgs::RecorderEventRecordingPartArgs popEventRecordingPartAnswer(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, bool removeFlag);

  /// @brief Return a copy of the current answer in preparation for the provided Query.
  ///
  /// @param query Original Query
  /// @return const Heex::RecorderArgs::RecorderContextValueArgs&
  const Heex::RecorderArgs::RecorderEventRecordingPartArgs getEventRecordingPartAnswer(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query);

  ///
  /// @brief Format and send the AnswerEventRecordingPart msg made from the original RecorderEventRecordingPartArgs query and the provided value as answer. Message is sent to WW using the Recorder TCP Client.
  ///
  /// @param fileQuery RecorderEventRecordingPartArgs Query to build answer on.
  /// @param encodedFilepaths Encoded string containing all of the ContextValues encoded. Delimiters are not encoded.
  virtual void sendEventRecordingPartAnswer(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& fileQuery, const std::string& encodedFilepaths);

  /// @brief SDE specify that all agent configurations values needed
  virtual void onConfigurationChanged(const std::string&) override;

  /// Receive RQueryContextValue message from TCP Server WW. Parse query and run _onContextValueRequestFunc on it.
  virtual void onContextValueRequestCallback(const std::string& msg);

  /// Receive RQueryEventRecordingPart message from TCP Server WW. Parse query and run _onDataFilepathFunc on it.
  virtual void onEventRecordingPartRequestCallback(const std::string& msg);

  size_t getSizeOfRecordingPartAnswerQueue();

private:
  ///
  /// @brief Format and send the AnswerContextValue msg made from the original RecorderContextValueArgs query and the provided value as answer. Message is sent to WW using the Recorder TCP Client.
  ///
  /// @param query RecorderContextValueArgs Query to build answer on.
  /// @param encodedValues Encoded string containing all of the ContextValues encoded. Delimiters are not encoded.
  virtual void sendContextValueAnswer(const Heex::RecorderArgs::RecorderContextValueArgs& query, const std::string& encodedValues);

  /// Parse Recorder message for QueryContextValue into a RecorderContextValueArgs object and checks its consistency
  Heex::RecorderArgs::RecorderContextValueArgs parseRecorderContextValueQuery(const std::string& msg);

  /// Parse Recorder message for QueryEventRecordingPart into a RecorderEventRecordingPartArgs object and checks its consistency
  Heex::RecorderArgs::RecorderEventRecordingPartArgs parseRecorderEventRecordingPartQuery(const std::string& msg);

  /// Is the recorder performing contextValues requests
  std::atomic<bool> _isPerformingContextValue{};

  /// Is the recorder performing eventRecordingParts requests
  std::atomic<bool> _isPerformingEventRecordingParts{};

  /// Queue containing the EventRecordingPart answer. They are a intial copy of the query with on-going computation and awaits to be modified, if needed, then answered.
  std::vector<Heex::RecorderArgs::RecorderEventRecordingPartArgs> _eventRecordingPartAnswerQueue;

  /// Mutex on EventRecordingPart answer queue operation
  std::mutex _eventRecordingPartAnswerQueue_m;

  /// map containing the recording start and end for each trigger where the recorder is used
  std::vector<Heex::RecorderArgs::RecorderRangesValues> _recorderRangesRegistry;
};
