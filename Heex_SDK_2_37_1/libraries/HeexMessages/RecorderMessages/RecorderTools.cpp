///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "../include/RecorderTools.h"

#include <sstream>

#include "../include/Tools.h"
#include "HeexUtils.h"
#include "HeexUtilsLog.h"

// TODO What about renaming this into AgentTools

Heex::RecorderArgs::RecorderContextValueArgs Heex::RecorderTools::parseRecorderContextValueMsg(const std::string& msg, Heex::RecorderArgs::RecorderCmdType msgType)
{
  std::vector<std::string> params = HeexUtils::split(msg, ' ');
  Heex::RecorderArgs::RecorderContextValueArgs result;
  result.valid = true;

  // Warning : Update the value with the value used to check if you update enum RecordingPartMsgIndex
  const unsigned int minLenContextValueQuery  = Heex::RecorderTools::ContextValueMsgIndex::ContextValue + 1U;
  const unsigned int minLenContextValueAnswer = Heex::RecorderTools::ContextValueMsgIndex::Incident + 1U;

  // DEBUG
  if (msgType == Heex::RecorderArgs::RecorderCmdType::QueryContextValue)
  {
    HEEX_LOG(debug) << "parseRecorderContextValueMsg | Receiving QueryContextValue msg: " << msg << std::endl; // DEBUG
    HEEX_LOG(debug) << "Arg number : " << params.size();
    HEEX_LOG(debug) << "Number should be at least " << minLenContextValueQuery;
  }
  else if (msgType == Heex::RecorderArgs::RecorderCmdType::AnswerContextValue)
  {
    HEEX_LOG(debug) << "parseRecorderContextValueMsg | Receiving AnswerContextValue msg: " << msg << std::endl; // DEBUG
    HEEX_LOG(debug) << "Arg number : " << params.size();
    HEEX_LOG(debug) << "Number should be at least " << minLenContextValueAnswer;
  }

  // First Basic Check for Msg: Size
  // Vector length expected is PARAM_NUMBER_IN_CONTEXTVALUE_MSG and PARAM_NUMBER_IN_CONTEXTVALUE_MSG+1 is we have a value field, below is considered error.
  // Above is accepted but not parsed as only stored in the UnparsedArgs field.

  if ((msgType == Heex::RecorderArgs::RecorderCmdType::QueryContextValue && params.size() < minLenContextValueQuery) ||
      (msgType == Heex::RecorderArgs::RecorderCmdType::AnswerContextValue && params.size() < minLenContextValueAnswer))
  {
    HEEX_LOG(debug) << "Bad param number : " << params.size();
    result.valid = false;
    return result;
  }
  HEEX_LOG(debug) << "Good param number";

  // Parse Incidents first so we can get the incidents anyway.
  if (msgType == Heex::RecorderArgs::RecorderCmdType::AnswerContextValue)
  {
    std::string serializedIncidents(params[Heex::RecorderTools::ContextValueMsgIndex::Incident]);
    Heex::Tools::decodeReplacementCodeWithEscapeCharacters(serializedIncidents);
    if (serializedIncidents != Heex::RecorderTools::HEEX_NO_CONTEXT_VALUE)
    {
      result.incidents = Heex::Incident::deserializeIncidents(serializedIncidents);
    }
  }

  // Parse uuid (recorder)
  result.valid = Heex::Tools::checkUuid(params[Heex::RecorderTools::ContextValueMsgIndex::Uuid], Heex::Tools::UuidPrefixType::Recorder);
  if (!result.valid)
  {
    return result;
  }
  result.uuid = params[Heex::RecorderTools::ContextValueMsgIndex::Uuid];
  HEEX_LOG(debug) << " uuid\t -> " << result.uuid << std::endl; // DEBUG

  // Parse timestamp
  result.valid = Heex::Tools::checkTimestamp(params[Heex::RecorderTools::ContextValueMsgIndex::Timestamp]);
  if (!result.valid)
  {
    return result;
  }
  result.timestamp = params[Heex::RecorderTools::ContextValueMsgIndex::Timestamp];
  HEEX_LOG(debug) << " T\t -> " << result.timestamp << std::endl; // DEBUG

  // Parse eventUuid
  result.valid = Heex::Tools::checkUuid(params[Heex::RecorderTools::ContextValueMsgIndex::EventUuid], Heex::Tools::UuidPrefixType::Event);
  if (!result.valid)
  {
    return result;
  }
  result.eventUuid = params[Heex::RecorderTools::ContextValueMsgIndex::EventUuid];
  HEEX_LOG(debug) << " Event\t -> " << result.eventUuid << std::endl; // DEBUG

  // Check and Parse ContextValues
  result.valid = Heex::Tools::checkCoupleValuesStr(params[Heex::RecorderTools::ContextValueMsgIndex::ContextValue]);
  if (!result.valid)
  {
    return result;
  }
  std::vector<Heex::RecorderArgs::ContextValue> contextValuesTmp;
  try
  {
    contextValuesTmp = Heex::RecorderTools::decodeContextValuesAsVector(params[Heex::RecorderTools::ContextValueMsgIndex::ContextValue]); // Can throw
  }
  catch (const std::runtime_error& e)
  {
    // Custom thrown error by decodeContextValuesAsVector()
    HEEX_LOG(error) << "[ERROR] WriterWrapper::onRecorderContextValueMsg " << e.what() << ". Discarding the whole Context Values." << '\n';
    result.valid = false;
    return result;
  }
  catch (const std::exception& e)
  {
    // Other may throw as out of bound or bad alloc due to missing or wrongly placed separators in decodeContextValuesAsVector()
    HEEX_LOG(error) << "[ERROR] WriterWrapper::onRecorderContextValueMsg " << e.what() << ". Discarding the whole Context Values." << '\n';
    result.valid = false;
    return result;
  }

  result.contextValues = contextValuesTmp;
  result.valid         = Heex::Tools::checkCoupleValuesVector(result.contextValues);
  if (!result.valid)
  {
    return result;
  }
  HEEX_LOG(debug) << " Value keys\t -> ";         // DEBUG
  Heex::Tools::printVector(result.contextValues); // DEBUG

  // Store unparsedArgs
  int lengthOfParsedArgs = 0;
  if (msgType == Heex::RecorderArgs::RecorderCmdType::QueryContextValue)
  {
    lengthOfParsedArgs = minLenContextValueQuery;
  }
  else
  {
    lengthOfParsedArgs = minLenContextValueAnswer;
  }
  std::vector<std::string> unparsedArgsTmp = std::vector<std::string>(params.begin() + lengthOfParsedArgs, params.end());
  result.unparsedArgs                      = "";
  for (unsigned int i = 0; i < unparsedArgsTmp.size(); i++)
  {
    result.unparsedArgs.append(unparsedArgsTmp[i]);
    if (i != unparsedArgsTmp.size() - 1)
    {
      result.unparsedArgs.append(" ");
    }
  }

  // Do all the checks before this last validation
  result.valid = true;

  return result;
}

Heex::RecorderArgs::RecorderEventRecordingPartArgs Heex::RecorderTools::parseRecorderDataFilePathMsg(const std::string& msg, Heex::RecorderArgs::RecorderCmdType msgType)
{
  HEEX_LOG(debug) << "parseRecorderDataFilePathMsg | Receiving DataFilePath msg: " << msg << std::endl; // DEBUG

  std::vector<std::string> params = HeexUtils::split(msg, ' ');
  Heex::RecorderArgs::RecorderEventRecordingPartArgs result;
  result.valid = true;

  HEEX_LOG(debug) << "Arg number : " << params.size();

  // Warning : Update the value with the value used to check if you update enum RecordingPartMsgIndex
  const unsigned int minLenRecordingPartQuery  = Heex::RecorderTools::RecordingPartMsgIndex::RecordIntervalEnd + 1U;
  const unsigned int minLenRecordingPartAnswer = Heex::RecorderTools::RecordingPartMsgIndex::Incident + 1U;

  // First Basic Check for Msg: Size
  // Vector length expected is PARAM_NUMBER_IN_EVENTRECORDINGPART_MSG and PARAM_NUMBER_IN_EVENTRECORDINGPART_MSG+1 is we have a value field, below is considered error.
  // Above is accepted but not parsed as only stored in the UnparsedArgs field.

  if ((msgType == Heex::RecorderArgs::RecorderCmdType::QueryEventRecordingPart && params.size() < minLenRecordingPartQuery) ||
      (msgType == Heex::RecorderArgs::RecorderCmdType::AnswerEventRecordingPart && params.size() < minLenRecordingPartAnswer))
  {
    result.valid = false;
    return result;
  }

  // Parse Incidents first so we can get the incidents anyway.
  if (msgType == Heex::RecorderArgs::RecorderCmdType::AnswerEventRecordingPart)
  {
    std::string serializedIncidents(params[Heex::RecorderTools::RecordingPartMsgIndex::Incident]);
    Heex::Tools::decodeReplacementCodeWithEscapeCharacters(serializedIncidents);
    if (serializedIncidents != Heex::RecorderTools::HEEX_NO_CONTEXT_VALUE)
    {
      result.incidents = Heex::Incident::deserializeIncidents(serializedIncidents);
    }
  }

  // Parse uuid (recorder)
  result.valid = Heex::Tools::checkUuid(params[Heex::RecorderTools::RecordingPartMsgIndex::Uuid], Heex::Tools::UuidPrefixType::Recorder);
  if (!result.valid)
  {
    return result;
  }
  result.uuid = params[Heex::RecorderTools::RecordingPartMsgIndex::Uuid];
  HEEX_LOG(debug) << " uuid\t -> " << result.uuid << std::endl; // DEBUG

  // Parse timestamp
  result.valid = Heex::Tools::checkTimestamp(params[Heex::RecorderTools::RecordingPartMsgIndex::Timestamp]);
  if (!result.valid)
  {
    return result;
  }
  result.timestamp = params[Heex::RecorderTools::RecordingPartMsgIndex::Timestamp];
  HEEX_LOG(debug) << " T\t -> " << result.timestamp << std::endl; // DEBUG

  // Parse eventUuid
  result.valid = Heex::Tools::checkUuid(params[Heex::RecorderTools::RecordingPartMsgIndex::EventUuid], Heex::Tools::UuidPrefixType::Event);
  if (!result.valid)
  {
    return result;
  }
  result.eventUuid = params[Heex::RecorderTools::RecordingPartMsgIndex::EventUuid];
  HEEX_LOG(debug) << " Event\t -> " << result.eventUuid << std::endl; // DEBUG

  // Parse recordIntervalStart
  result.valid = Heex::Tools::checkTimeIntervalDuration(params[Heex::RecorderTools::RecordingPartMsgIndex::RecordIntervalStart]);
  if (!result.valid)
  {
    return result;
  }
  result.recordIntervalStart = params[Heex::RecorderTools::RecordingPartMsgIndex::RecordIntervalStart];
  HEEX_LOG(debug) << " TS\t -> " << result.recordIntervalStart << std::endl; // DEBUG

  // Parse recordIntervalEnd
  result.valid = Heex::Tools::checkTimeIntervalDuration(params[Heex::RecorderTools::RecordingPartMsgIndex::RecordIntervalEnd]);
  if (!result.valid)
  {
    return result;
  }
  result.recordIntervalEnd = params[Heex::RecorderTools::RecordingPartMsgIndex::RecordIntervalEnd];
  HEEX_LOG(debug) << " TE\t -> " << result.recordIntervalEnd << std::endl; // DEBUG

  int lengthOfParsedArgs = minLenRecordingPartQuery;
  if (msgType == Heex::RecorderArgs::RecorderCmdType::AnswerEventRecordingPart)
  {
    lengthOfParsedArgs = minLenRecordingPartAnswer;

    // Check and Parse value
    result.valid = Heex::Tools::checkRecordingsFilepath(params[Heex::RecorderTools::RecordingPartMsgIndex::RecordingsFilepath]);
    if (!result.valid)
    {
      return result;
    }
    result.value = params[Heex::RecorderTools::RecordingPartMsgIndex::RecordingsFilepath];
    HEEX_LOG(debug) << " Value\t -> " << result.value << std::endl; // DEBUG
  }

  std::vector<std::string> unparsedArgsTmp = std::vector<std::string>(params.begin() + lengthOfParsedArgs, params.end());
  result.unparsedArgs                      = "";
  for (unsigned int i = 0; i < unparsedArgsTmp.size(); i++)
  {
    result.unparsedArgs.append(unparsedArgsTmp[i]);
    if (i != unparsedArgsTmp.size() - 1)
    {
      result.unparsedArgs.append(" ");
    }
  }
  HEEX_LOG(debug) << " Unparsed\t -> " << result.unparsedArgs << std::endl; // DEBUG

  // Do all the checks before this last validation
  result.valid = true;

  return result;
}

Heex::RecorderArgs::RecorderArgsV2 Heex::RecorderTools::parseRecorderV2Msg(const std::string& msg, Heex::RecorderArgs::RecorderCmdType msgType)
{
  HEEX_LOG(debug) << "parseRecorderMsg | Receiving DataFilePath msg: '" << msg << "'"; // DEBUG

  std::vector<std::string> params = HeexUtils::split(msg, ' ');
  Heex::RecorderArgs::RecorderArgsV2 result;
  result.valid = true;

  HEEX_LOG(debug) << "Arg number : " << params.size();

  // Warning : Update the value with the value used to check if you update enum RecorderV2ArgsIndex
  const unsigned int minLenQuery  = Heex::RecorderTools::RecorderV2ArgsIndex::ContextValue + 1U;
  const unsigned int minLenAnswer = Heex::RecorderTools::RecorderV2ArgsIndex::Incident + 1U;
  int lengthOfParsedArgs          = minLenQuery;

  // First Basic Check for Msg: Size
  // Vector length expected is PARAM_NUMBER_IN_EVENTRECORDINGPART_MSG and PARAM_NUMBER_IN_EVENTRECORDINGPART_MSG+1 is we have a value field, below is considered error.
  // Above is accepted but not parsed as only stored in the UnparsedArgs field.

  if ((msgType == Heex::RecorderArgs::RecorderCmdType::QueryV2 && params.size() < minLenQuery) ||
      (msgType == Heex::RecorderArgs::RecorderCmdType::AnswerV2 && params.size() < minLenAnswer))
  {
    result.valid = false;
    HEEX_LOG(warning) << "parseRecorderMsg arg number error, expected : " << (msgType == Heex::RecorderArgs::RecorderCmdType::QueryV2 ? minLenQuery : minLenAnswer);
    return result;
  }

  // Parse Incidents first so we can get the incidents anyway.
  if (msgType == Heex::RecorderArgs::RecorderCmdType::AnswerV2)
  {
    lengthOfParsedArgs = minLenAnswer;
    std::string serializedIncidents(params[Heex::RecorderTools::RecorderV2ArgsIndex::Incident]);
    Heex::Tools::decodeReplacementCodeWithEscapeCharacters(serializedIncidents);
    if (serializedIncidents != Heex::RecorderTools::HEEX_NO_CONTEXT_VALUE)
    {
      result.incidents = Heex::Incident::deserializeIncidents(serializedIncidents);
    }
  }

  // Parse uuid (recorder)
  result.valid = Heex::Tools::checkUuid(params[Heex::RecorderTools::RecorderV2ArgsIndex::Uuid], Heex::Tools::UuidPrefixType::Recorder);
  if (!result.valid)
  {
    HEEX_LOG(warning) << "parseRecorderMsg unvalid uuid : " << params[Heex::RecorderTools::RecorderV2ArgsIndex::Uuid];
    return result;
  }
  result.uuid = params[Heex::RecorderTools::RecorderV2ArgsIndex::Uuid];
  HEEX_LOG(debug) << " uuid\t -> " << result.uuid << std::endl; // DEBUG

  // Parse timestamp
  result.valid = Heex::Tools::checkTimestamp(params[Heex::RecorderTools::RecorderV2ArgsIndex::Timestamp]);
  if (!result.valid)
  {
    HEEX_LOG(warning) << "parseRecorderMsg unvalid timestamp : " << params[Heex::RecorderTools::RecorderV2ArgsIndex::Timestamp];
    return result;
  }

  result.timestamp = params[Heex::RecorderTools::RecorderV2ArgsIndex::Timestamp];
  HEEX_LOG(debug) << " T\t -> " << result.timestamp << std::endl; // DEBUG

  // Parse eventUuid
  result.valid = Heex::Tools::checkUuid(params[Heex::RecorderTools::RecorderV2ArgsIndex::EventUuid], Heex::Tools::UuidPrefixType::Event);
  if (!result.valid)
  {
    HEEX_LOG(warning) << "parseRecorderMsg unvalid event uuid : " << params[Heex::RecorderTools::RecorderV2ArgsIndex::EventUuid];
    return result;
  }

  result.eventUuid = params[Heex::RecorderTools::RecorderV2ArgsIndex::EventUuid];
  HEEX_LOG(debug) << " Event\t -> " << result.eventUuid << std::endl; // DEBUG

  // Parse recordIntervalStart
  result.valid = Heex::Tools::checkTimeIntervalDuration(params[Heex::RecorderTools::RecorderV2ArgsIndex::RecordIntervalStart]);
  if (!result.valid)
  {
    HEEX_LOG(warning) << "parseRecorderMsg unvalid start interval : " << params[Heex::RecorderTools::RecorderV2ArgsIndex::RecordIntervalStart];
    return result;
  }

  result.recordIntervalStart = params[Heex::RecorderTools::RecorderV2ArgsIndex::RecordIntervalStart];
  HEEX_LOG(debug) << " TS\t -> " << result.recordIntervalStart << std::endl; // DEBUG

  // Parse recordIntervalEnd
  result.valid = Heex::Tools::checkTimeIntervalDuration(params[Heex::RecorderTools::RecorderV2ArgsIndex::RecordIntervalEnd]);
  if (!result.valid)
  {
    HEEX_LOG(warning) << "parseRecorderMsg unvalid end interval : " << params[Heex::RecorderTools::RecorderV2ArgsIndex::RecordIntervalEnd];
    return result;
  }

  result.recordIntervalEnd = params[Heex::RecorderTools::RecorderV2ArgsIndex::RecordIntervalEnd];
  HEEX_LOG(debug) << " TE\t -> " << result.recordIntervalEnd << std::endl; // DEBUG

  // Check and Parse ContextValues
  result.valid = Heex::Tools::checkCoupleValuesStr(params[Heex::RecorderTools::RecorderV2ArgsIndex::ContextValue]);
  if (!result.valid)
  {
    HEEX_LOG(warning) << "parseRecorderMsg unvalid contextValue : " << params[Heex::RecorderTools::RecorderV2ArgsIndex::ContextValue];
    return result;
  }

  std::vector<Heex::RecorderArgs::ContextValue> contextValuesTmp;
  try
  {
    contextValuesTmp = Heex::RecorderTools::decodeContextValuesAsVector(params[Heex::RecorderTools::RecorderV2ArgsIndex::ContextValue]); // Can throw
  }
  catch (const std::runtime_error& e)
  {
    // Custom thrown error by decodeContextValuesAsVector()
    HEEX_LOG(error) << "[ERROR] WriterWrapper::onRecorderContextValueMsg " << e.what() << ". Discarding the whole Context Values." << '\n';
    result.valid = false;
    return result;
  }
  catch (const std::exception& e)
  {
    // Other may throw as out of bound or bad alloc due to missing or wrongly placed separators in decodeContextValuesAsVector()
    HEEX_LOG(error) << "[ERROR] WriterWrapper::onRecorderContextValueMsg " << e.what() << ". Discarding the whole Context Values." << '\n';
    result.valid = false;
    return result;
  }

  result.contextValues = contextValuesTmp;
  result.valid         = Heex::Tools::checkCoupleValuesVector(result.contextValues);
  if (!result.valid)
  {
    return result;
  }
  HEEX_LOG(debug) << " Value keys\t -> ";         // DEBUG
  Heex::Tools::printVector(result.contextValues); // DEBUG

  std::vector<std::string> unparsedArgsTmp = std::vector<std::string>(params.begin() + lengthOfParsedArgs, params.end());
  result.unparsedArgs                      = "";
  for (unsigned int i = 0; i < unparsedArgsTmp.size(); i++)
  {
    result.unparsedArgs.append(unparsedArgsTmp[i]);
    if (i != unparsedArgsTmp.size() - 1)
    {
      result.unparsedArgs.append(" ");
    }
  }
  HEEX_LOG(debug) << " Unparsed\t -> " << result.unparsedArgs << std::endl; // DEBUG

  // Do all the checks before this last validation
  result.valid = true;

  return result;
}

std::string Heex::RecorderTools::encodeContextValuesAsString(const std::vector<Heex::RecorderArgs::ContextValue>& contextValVec)
{
  std::vector<std::vector<std::string>> tmp;
  std::string tmpKey;
  std::string tmpVal;
  for (const Heex::RecorderArgs::ContextValue& contextValue : contextValVec)
  {
    tmpKey = contextValue.key;
    tmpVal = contextValue.value;
    Heex::Tools::encodeEscapeCharactersWithReplacementCode(tmpKey);
    Heex::Tools::encodeEscapeCharactersWithReplacementCode(tmpVal);
    tmp.push_back(std::vector<std::string>{tmpKey, tmpVal});
  }

  return HeexUtils::encodeVectorAsString(tmp, Heex::HEEX_KEY_VALUE_SEPARATOR, Heex::HEEX_COUPLE_SEPARATOR);
}

std::vector<Heex::RecorderArgs::ContextValue> Heex::RecorderTools::decodeContextValuesAsVector(const std::string& contextValStr)
{
  const std::vector<std::vector<std::string>> tmp = HeexUtils::decodeStringAsVector(contextValStr, Heex::HEEX_KEY_VALUE_SEPARATOR, Heex::HEEX_COUPLE_SEPARATOR);

  std::vector<Heex::RecorderArgs::ContextValue> res;
  std::string tmpKey;
  std::string tmpVal;

  for (std::vector<std::string> contextValueAsVect : tmp)
  {
    if (contextValueAsVect[0] == Heex::RecorderTools::HEEX_NO_CONTEXT_VALUE)
    {
      continue;
    }

    if (contextValueAsVect.size() == 2)
    {
      // key
      tmpKey = contextValueAsVect[0];
      Heex::Tools::decodeReplacementCodeWithEscapeCharacters(tmpKey);
      // value
      tmpVal = contextValueAsVect[1];
      Heex::Tools::decodeReplacementCodeWithEscapeCharacters(tmpVal);
      // Push to res
      res.push_back(Heex::RecorderArgs::ContextValue(tmpKey, tmpVal));
    }
    else if (contextValueAsVect.size() == 1)
    {
      // key
      tmpKey = contextValueAsVect[0];
      Heex::Tools::decodeReplacementCodeWithEscapeCharacters(tmpKey);
      // value : Value is empty. Push an empty string.
      res.push_back(Heex::RecorderArgs::ContextValue(tmpKey));
    }
    else
    {
      throw std::runtime_error("Vector of Context values contains an unmanaged number of elements.");
    }
  }

  return res;
}

std::string Heex::RecorderTools::encodeRecordingsFilepathAsString(const std::vector<Heex::RecorderArgs::RecordingFilepath>& recordingsFilepathVec)
{
  std::vector<std::vector<std::string>> tmp;
  std::string tmpKey;
  std::string tmpVal;
  for (const Heex::RecorderArgs::RecordingFilepath& recordingFilepath : recordingsFilepathVec)
  {
    tmpKey = recordingFilepath.recorderUuid;
    tmpVal = recordingFilepath.filepath;
    Heex::Tools::encodeEscapeCharactersWithReplacementCode(tmpKey);
    Heex::Tools::encodeEscapeCharactersWithReplacementCode(tmpVal);
    tmp.push_back(std::vector<std::string>{tmpKey, tmpVal});
  }

  return HeexUtils::encodeVectorAsString(tmp, Heex::HEEX_KEY_VALUE_SEPARATOR, Heex::HEEX_COUPLE_SEPARATOR);
}

std::vector<Heex::RecorderArgs::RecordingFilepath> Heex::RecorderTools::decodeRecordingsFilepathAsVector(const std::string& recordingsFilepathStr)
{
  const std::vector<std::vector<std::string>> tmp = HeexUtils::decodeStringAsVector(recordingsFilepathStr, Heex::HEEX_KEY_VALUE_SEPARATOR, Heex::HEEX_COUPLE_SEPARATOR);

  std::vector<Heex::RecorderArgs::RecordingFilepath> res;
  std::string tmpRecorderUuid;
  std::string tmpFilepath;

  for (std::vector<std::string> recordingFilepathVec : tmp)
  {
    if (recordingFilepathVec.size() >= 1)
    {
      // key
      tmpRecorderUuid = recordingFilepathVec[0];
      Heex::Tools::decodeReplacementCodeWithEscapeCharacters(tmpRecorderUuid);
      // NOTE HeexUtils::split (called by decodeStringAsVector) does not handle empty string after separator(:), i.e  "r1:" will resulted in recordingFilepathVec with size 1
      // value
      if (recordingFilepathVec.size() >= 2)
      {
        tmpFilepath = recordingFilepathVec[1];
      }
      else
      {
        tmpFilepath = Heex::RecorderTools::HEEX_EMPTY_FILEPATH;
      }
      Heex::Tools::decodeReplacementCodeWithEscapeCharacters(tmpFilepath);
      // Push to res
      res.push_back(Heex::RecorderArgs::RecordingFilepath(tmpRecorderUuid, tmpFilepath));
    }
    else
    {
      HEEX_LOG(error) << "Vector of recordings filepath contains an unmanaged number of elements.";
    }
  }

  return res;
}

bool Heex::RecorderTools::addContextValue(std::vector<Heex::RecorderArgs::ContextValue>& res, const std::string key, const std::string value)
{
  // Heex::RecorderTools::HEEX_NO_CONTEXT_VALUE is forbidden
  if (key == Heex::RecorderTools::HEEX_NO_CONTEXT_VALUE || key == std::string())
  {
    HEEX_LOG(error) << "::addContextValue Cannot add the value << " << value << " for key " << key << ". " << key << "is a forbidden string." << '\n';
    return false;
  }
  // Try to find a matching key in the ContextValues vector
  for (unsigned int i = 0; i < res.size(); ++i)
  {
    if (res[i].key == key)
    {
      if (res[i].value != value)
      {
        HEEX_LOG(info) << "::addContextValue Replacing older value " << res[i].value << " by " << value << " for key " << key << '\n';
      }
      res[i].value = value;
      return true;
    }
  }
  // Add at the end of contextValues
  res.push_back(Heex::RecorderArgs::ContextValue(key, value));
  HEEX_LOG(debug) << "::addContextValue Adding key " << key << " and value " << value << " at the end of contextValues" << std::endl;
  return true;
}

Heex::RecorderArgs::RecorderRangesValues Heex::RecorderTools::parseRecorderRangesMsg(const std::string& value)
{
  // Expect to parse string formatted as "TriggerUUID recordIntervalStart recordIntervalEnd".i.e T-UUID -10 10
  Heex::RecorderArgs::RecorderRangesValues result;
  std::vector<std::string> params = HeexUtils::split(value, ' ');

  // Warning : Update the value with the value used to check if you update enum RecordingRangesMsgIndex
  const unsigned int minLenRecordingRangesQuery = Heex::RecorderTools::RecordingRangesMsgIndex::RecordIntervalEnd + 1U;

  if (params.size() < minLenRecordingRangesQuery)
  {
    HEEX_LOG(debug) << "::parseRecorderRangesMsg Bad msg format";
    result.valid = false;
    return result;
  }

  if (Heex::Tools::checkUuid(params[Heex::RecorderTools::RecordingRangesMsgIndex::Uuid], Heex::Tools::UuidPrefixType::Trigger) == false)
  {
    HEEX_LOG(debug) << "::parseRecorderRangesMsg incorrect uuid format";
    result.valid = false;
    return result;
  }
  result.uuid = params[Heex::RecorderTools::RecordingRangesMsgIndex::Uuid];

  Heex::Tools::decodeReplacementCodeWithEscapeCharacters(params[Heex::RecorderTools::RecordingRangesMsgIndex::RecordIntervalStart]);
  Heex::Tools::decodeReplacementCodeWithEscapeCharacters(params[Heex::RecorderTools::RecordingRangesMsgIndex::RecordIntervalEnd]);
  const double recordIntervalStart = std::atof(params[Heex::RecorderTools::RecordingRangesMsgIndex::RecordIntervalStart].c_str());
  const double recordIntervalEnd   = std::atof(params[Heex::RecorderTools::RecordingRangesMsgIndex::RecordIntervalEnd].c_str());
  result.recordIntervalStart       = recordIntervalStart;
  result.recordIntervalEnd         = recordIntervalEnd;

  result.valid = true;
  return result;
}

std::vector<Heex::RecorderArgs::ContextValue> Heex::RecorderTools::filterRecordingParts(std::vector<Heex::RecorderArgs::ContextValue> input)
{
  std::vector<Heex::RecorderArgs::ContextValue> out;
  for (std::vector<Heex::RecorderArgs::ContextValue>::iterator it = input.begin(); it != input.end(); ++it)
  {
    if (it->key.find(Heex::RecorderTools::HEEX_RECORDING_PART_KEY_PREFIX) == 0)
    {
      out.push_back(*it);
    }
  }
  return out;
}
