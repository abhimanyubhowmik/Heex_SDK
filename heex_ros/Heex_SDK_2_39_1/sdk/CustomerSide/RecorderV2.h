///
/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include <mutex>
#include <vector>

#include "Agent.h"
#include "RecorderArgs.h"

/// @brief This RecorderV2 class defines an advance recorder structure that enables sending events metadata quickly and in a second time the recordings.
class RecorderV2 : public Agent
{
public:
  /// @brief Constructor for RecorderV2 with its full configuration. It requires the uuid and the networking settings for initialization.
  ///
  /// @param uuid Unique identifier for the RecorderV2. Shall be a string starting with 'R-' and finishing with a version-4 UUID.
  /// @param serverIp Ip of the Heex Core to which the RecorderV2 will connect, identify and notify
  /// @param serverPort Port of the Heex Core dedicated to the Writer Wrapper (WW) module
  /// @param implementationVersion Implementation version of the recorderV2. Transmitted to the SDE during agent identification process.
  RecorderV2(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION);

  ~RecorderV2();

protected:
  virtual void onInstantRequestCallback(const std::string& msg);
  virtual void onTimeFrameRequestCallback(const std::string& msg);

  /// Override the virtual function of the Recorder class to define the logic of getting data values
  /// (mostly scalars but could also a files such as a frame video capture) at the exact moment of the event.
  /// This function should return as quickly as possible
  virtual bool generateInstantRecordings(const Heex::RecorderArgs::RecorderArgsV2& query);

  /// Override the virtual function of the Recorder class to define the logic of getting data values
  /// (mostly files) during a time frame around the event (e.g. a few seconds of video recording).
  virtual bool generateTimeFrameRecordings(const Heex::RecorderArgs::RecorderArgsV2& query);

  virtual void sendCommandRecordingAnswer(const std::string& command, const Heex::RecorderArgs::RecorderArgsV2& answerArgs, const std::string& encodedValues);

  /// @brief Add the value to the provided key in the ContextValue variable. Add key and value at the end of contextValues if key does not exist.
  ///
  /// @param res ContextValue variable. Pass-by-reference.
  /// @param key Key to add value for
  /// @param value Value to add
  /// @return true The value has been added with success for the provided key.
  /// @return false The value hasn't been added as an error has occurred.
  bool addContextValue(std::vector<Heex::RecorderArgs::ContextValue>& res, const std::string key, const std::string value);

  /// @brief Add the value to the provided key in the ContextValue variable. Add key and value at the end of contextValues if key does not exist.
  ///
  /// @param query that links to the answer ContextValue.
  /// @param key Key to add value for
  /// @param value Value to add
  /// @return true The value has been added with success for the provided key.
  /// @return false The value hasn't been added as an error has occurred.
  bool addContextValue(const Heex::RecorderArgs::RecorderArgsV2& query, const std::string key, const std::string value);

  /// @brief Add the value to the provided key in the ContextValue variable. Add key and value at the end of contextValues if key does not exist.
  ///
  /// @param query that links to the answer ContextValue.
  /// @param recordingPartPath path of the recording part to add.
  /// @return true The value has been added with success for the provided key.
  /// @return false The value hasn't been added as an error has occurred.
  bool addRecordingPart(const Heex::RecorderArgs::RecorderArgsV2& query, const std::string recordingPartPath);

  /// @brief Return the keys required by the query
  ///
  /// @param query Query to extract keys from
  /// @return std::vector<std::string> List the required keys
  std::vector<std::string> getContextValueKeys(const Heex::RecorderArgs::RecorderArgsV2& query);

  /// @brief SDE specify that all agent configurations values needed
  virtual void onConfigurationChanged(const std::string&) override;

  bool isPerformingCallback();
  void addNewRecordingAnswer(const Heex::RecorderArgs::RecorderArgsV2& query);
  bool setRealRecordIntervalRange(const Heex::RecorderArgs::RecorderArgsV2& query, const std::string& realRecordIntervalStart, const std::string& realRecordIntervalEnd);
  const Heex::RecorderArgs::RecorderArgsV2 getRecordingAnswer(const Heex::RecorderArgs::RecorderArgsV2& query);
  const Heex::RecorderArgs::RecorderArgsV2 popRecordingAnswer(const Heex::RecorderArgs::RecorderArgsV2& query, bool removeFlag);
  const std::vector<Heex::RecorderArgs::RecorderRangesValues>& getRecorderRangesRegistry();
  virtual size_t getSizeOfRecordingAnswerQueue();

  /// Is the recorder performing instant recording requests
  std::atomic<bool> _isPerformingInstantRecording{};

  /// Is the recorder performing time frame recording requests
  std::atomic<bool> _isPerformingTimeFrameRecording{};

  /// Queue containing the EventRecordingPart answer. They are a intial copy of the query with on-going computation and awaits to be modified, if needed, then answered.
  std::vector<Heex::RecorderArgs::RecorderArgsV2> _recordingAnswerQueue;

  /// Mutex on EventRecordingPart answer queue operation
  std::mutex _recordingAnswerQueue_m;

  /// map containing the recording start and end for each trigger where the recorder is used
  std::vector<Heex::RecorderArgs::RecorderRangesValues> _recorderRangesRegistry;
};
