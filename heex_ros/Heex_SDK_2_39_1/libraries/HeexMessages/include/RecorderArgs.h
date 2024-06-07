///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "Incident.h"

namespace Heex
{
namespace RecorderArgs
{

/// Enum to describe the type of message.
enum RecorderCmdType
{
  QueryContextValue,
  QueryEventRecordingPart,
  AnswerContextValue,
  AnswerEventRecordingPart,
  QueryV2,
  AnswerV2
}; // RecorderCmdType

/// @brief Structure of a contextValue
///
struct ContextValue
{
  std::string key;
  std::string value;

  ContextValue(std::string k = std::string(), std::string v = std::string()) : key(k), value(v) {}

  inline bool operator==(const ContextValue cv2) const { return (key == cv2.key) && (value == cv2.value); }

  friend std::ostream& operator<<(std::ostream& os, const ContextValue& cv)
  {
    os << cv.key << ":" << cv.value << ";";
    return os;
  }
};

struct RecordingFilepath
{
  std::string recorderUuid;
  std::string filepath;

  RecordingFilepath(std::string r = std::string(), std::string f = std::string()) : recorderUuid(r), filepath(f) {}

  inline bool operator==(const RecordingFilepath rf2) const { return (recorderUuid == rf2.recorderUuid) && (filepath == rf2.filepath); }

  friend std::ostream& operator<<(std::ostream& os, const RecordingFilepath& rf)
  {
    os << rf.recorderUuid << ":" << rf.filepath << ";";
    return os;
  }
};

/// @brief Args structure for RecorderQueryContextValue message used in the Recorder-HeexSDE interprocess communication.
/// This structure is used to store the result of the parsing of Recorder ContextValue messages. Heex SDE and Recorder shared that formalism for QueryContextValue and AnswerContextValue commands.
struct RecorderContextValueArgs
{
  bool valid{false};
  std::string uuid;
  std::string eventUuid;
  std::string timestamp;
  std::vector<ContextValue> contextValues; ///<Contains the Context Values store as [key:value;[key:value]*].
  std::string unparsedArgs;
  std::vector<Heex::Incident> incidents;
}; // RecoderQueryContextValueArgs

/// @brief Args structure for RecorderDataFilePath message used in the Recorder-HeexSDE interprocess communication.
/// This structure is used to store the result of the parsing of Recorder DataFilePath messages. Heex SDE and Recorder shared that formalism for QueryDataFile and AnswerEventRecordingPart commands.
struct RecorderEventRecordingPartArgs
{
  bool valid{false};
  std::string uuid;
  std::string eventUuid;
  std::string timestamp;
  std::string recordIntervalStart;
  std::string recordIntervalEnd;
  std::string value;
  std::string unparsedArgs;
  std::vector<Heex::Incident> incidents;
}; // RecorderEventRecordingPartArgs

/// @brief Args structure for V2 message used in the Recorder-HeexSDE interprocess communication.
/// This structure is used to store the result of the parsing of Recorder V2 messages. Heex SDE and Recorder shared that formalism for Query and Answer commands.
struct RecorderArgsV2
{
  bool valid{false};
  std::string uuid;
  std::string eventUuid;
  std::string timestamp;
  std::string recordIntervalStart;
  std::string recordIntervalEnd;
  std::vector<ContextValue> contextValues; ///<Contains the Context Values store as [key:value;[key:value]*].
  std::string unparsedArgs;
  std::vector<Heex::Incident> incidents;
}; // RecorderArgsV2

/// @brief Structure for recording ranges values
struct RecorderRangesValues
{
  bool valid{false};
  std::string uuid;              //<! Trigger uuid
  double recordIntervalStart{0}; //<! recordIntervalStart
  double recordIntervalEnd{0};   //<! recordIntervalEnd
};                               // RecorderRangesValues

// Ensure backward compatibility for Recorder made with the HeexSDK v1.3.0 and inferior
typedef RecorderContextValueArgs RecorderDataValueArgs;
typedef RecorderEventRecordingPartArgs RecoderDataFilePathArgs;
// Ensure backward compatibility for Recorder made with the HeexSDK v2.2.0 and inferior
typedef RecorderContextValueArgs RecoderQueryDataValueArgs;
typedef RecorderEventRecordingPartArgs RecorderDataFilePathArgs;
} // namespace RecorderArgs
} // namespace Heex
