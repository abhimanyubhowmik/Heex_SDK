///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "AgentArgs.h"
#include "RecorderArgs.h"

namespace Heex
{
namespace RecorderTools
{
const std::string HEEX_NO_CONTEXT_VALUE           = "HEEX_NO_CONTEXT_VALUE";
const std::string HEEX_EMPTY_CONTEXT_VALUE        = std::string();
const std::string HEEX_EMPTY_FILEPATH             = std::string();
const std::string HEEX_RECORDING_PART_KEY_PREFIX  = "RecordingPart_";
const std::string HEEX_QUERY_INSTANT_RECORDING    = "QueryInstantRecording";
const std::string HEEX_QUERY_TIME_FRAME_RECORDING = "QueryTimeFrameRecording";

namespace ContextValueMsgIndex
{
/// Enum to specify the position of data in the message QueryEventContextValue or AnswerEventContextValue
enum ContextValueMsgIndex
{
  Uuid         = 0, ///<The query and answer payload starts here
  Timestamp    = 1,
  EventUuid    = 2,
  ContextValue = 3, ///<The query payload ends here
  Incident     = 4, ///<The answer payload ends here
};
} // namespace ContextValueMsgIndex

namespace RecordingPartMsgIndex
{
/// Enum to specify the position of data in the message QueryEventRecordingPart or AnswerEventRecordingPart
enum RecordingPartMsgIndex
{
  Uuid                = 0, ///<The query and answer payload starts here
  Timestamp           = 1,
  EventUuid           = 2,
  RecordIntervalStart = 3,
  RecordIntervalEnd   = 4, ///<The query payload ends here
  RecordingsFilepath  = 5,
  Incident            = 6, ///<The answer payload ends here
};
} // namespace RecordingPartMsgIndex

namespace RecorderV2ArgsIndex
{
/// Enum to specify the position of data in the recorderV2 messages Query and Answer
enum RecorderV2ArgsIndex
{
  Uuid                = 0, ///<The query and answer payload starts here
  Timestamp           = 1,
  EventUuid           = 2,
  RecordIntervalStart = 3,
  RecordIntervalEnd   = 4, ///<The query payload ends here
  ContextValue        = 5, ///<The query payload ends here
  Incident            = 6, ///<The answer payload ends here
};
} // namespace RecorderV2ArgsIndex

namespace RecordingRangesMsgIndex
{
/// Enum to specify the position of data in the message HEEX_RECORDING_RANGE_KEY
enum RecordingRangesMsgIndex
{
  Uuid                = 0,
  RecordIntervalStart = 1,
  RecordIntervalEnd   = 2, // The message payloads end here
};
} // namespace RecordingRangesMsgIndex

/// @brief Method to parse regular ContextValue messages. Type to parse, either Query or Answer, is provided in the second parameter.
/// Return the content of the query or answer in the RecorderContextValueArgs struct format. Performs consistency checks on each of the provided fields to parse.
///
/// @param msg Content of the message to parse.
/// @param msgType Type of message. Determines if it contains an extra field for storing the value.
Heex::RecorderArgs::RecorderContextValueArgs parseRecorderContextValueMsg(const std::string& msg, Heex::RecorderArgs::RecorderCmdType msgType);

/// @brief Method to parse regular DataFilePath messages. Type to parse, either Query or Answer, is provided in the second parameter.
/// Return the content of the query or answer in the RecorderEventRecordingPartArgs struct format. Performs consistency checks on each of the provided fields to parse.
///
/// @param msg Content of the message to parse.
/// @param msgType Type of message. Determines if it contains an extra field for storing the value.
Heex::RecorderArgs::RecorderEventRecordingPartArgs parseRecorderDataFilePathMsg(const std::string& msg, Heex::RecorderArgs::RecorderCmdType msgType);

/// @brief Method to parse V2 messages. Type to parse, either Query or Answer, is provided in the second parameter.
/// Return the content of the query or answer in the RecorderArgs struct format. Performs consistency checks on each of the provided fields to parse.
///
/// @param msg Content of the message to parse.
/// @param msgType Type of message. Determines if it contains an extra field for storing the value.
Heex::RecorderArgs::RecorderArgsV2 parseRecorderV2Msg(const std::string& msg, Heex::RecorderArgs::RecorderCmdType msgType);

/// @brief Method to parse regular recording ranges values.
/// @return a struct with the trigger uuid and recording start and end
/// @param value Content of the message to parse.
Heex::RecorderArgs::RecorderRangesValues parseRecorderRangesMsg(const std::string& value);

/// @brief Encode method to generate from a vector into a single encoded string for ContextValues.
///
/// @param contextValVec Vector of ContextValue structures.
/// @return std::string String to store Context Values (key, value). Keys and values are stored as [key:value[;key:value]].
std::string encodeContextValuesAsString(const std::vector<Heex::RecorderArgs::ContextValue>& contextValVec);

/// @brief Decode method to parse the content of an encoded string into a vector for ContextValues.
///
/// @param contextValStr String to convert that stores Context Values (key, value). Keys and values are stored as [key:value[;key:value]]. Should only contain escaped characters (cf. encode and decode functions)
/// @return std::vector<ContextValue> Return a vector of ContextValue structures.
std::vector<Heex::RecorderArgs::ContextValue> decodeContextValuesAsVector(const std::string& contextValStr);

/// @brief
///
/// @param recordingsFilepath
/// @return std::string
std::string encodeRecordingsFilepathAsString(const std::vector<Heex::RecorderArgs::RecordingFilepath>& recordingsFilepathVec);

/// @brief Decode method to parse the content of an encoded string into a vector for RecordingFilepath.
///
/// @param recordingsFilepath
/// @return std::vector<Heex::RecorderArgs::RecordingFilepath>
std::vector<Heex::RecorderArgs::RecordingFilepath> decodeRecordingsFilepathAsVector(const std::string& recordingsFilepathStr);

/// @brief Add the value to the provided key in the ContextValue variable. Add key and value at the end of contextValues if key does not exist.
///
/// @param res ContextValue variable. Pass-by-reference.
/// @param key Key to add value for
/// @param value Value to add
/// @return true The value has been added with success for the provided key.
/// @return false The value hasn't been added as an error has occurred.
bool addContextValue(std::vector<Heex::RecorderArgs::ContextValue>& res, const std::string key, const std::string value);
/// @brief Return only recording parts from a contextValue list.
std::vector<Heex::RecorderArgs::ContextValue> filterRecordingParts(std::vector<Heex::RecorderArgs::ContextValue> input);
} // namespace RecorderTools
} // namespace Heex
