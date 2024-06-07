///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
///
/// @file RecorderV2.cpp
/// @brief Library source file that contains the basic structure for any RecorderV2s to appropriately register and communicate with the Smart Data Engine. Works for both Real-time and Datalake applications.
#include "RecorderV2.h"

#include "HeexUtilsFileOperations.h"
#include "HeexUtilsLog.h"
#include "RecorderTools.h"
#include "Tools.h"

RecorderV2::RecorderV2(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion)
    : Agent(uuid, serverIp, serverPort, implementationVersion)
{
  _agentType = Heex::HEEX_AGENT_TYPE_RECORDERV2;

  if (_tcpC != nullptr)
  {
    _tcpC->subscribeToCmd(Heex::RecorderTools::HEEX_QUERY_INSTANT_RECORDING, std::bind(&RecorderV2::onInstantRequestCallback, this, std::placeholders::_1));
    _tcpC->subscribeToCmd(Heex::RecorderTools::HEEX_QUERY_TIME_FRAME_RECORDING, std::bind(&RecorderV2::onTimeFrameRequestCallback, this, std::placeholders::_1));
  }
}

RecorderV2::~RecorderV2() = default;

void RecorderV2::onInstantRequestCallback(const std::string& msg)
{
  HEEX_LOG(trace) << "RecorderV2::onInstantRequestCallback : " << msg;
  _isPerformingInstantRecording        = true;
  // Backup empty contextValues
  const std::string emptyContextValues = Heex::RecorderTools::HEEX_EMPTY_CONTEXT_VALUE;

  // Parsing and decoding
  const Heex::RecorderArgs::RecorderArgsV2 query = Heex::RecorderTools::parseRecorderV2Msg(msg, Heex::RecorderArgs::RecorderCmdType::QueryV2);
  if (query.valid == false)
  {
    HEEX_LOG(error) << this->getUuid() << "::onInstantRequestCallback bad msg formating.";
    this->reportIncident("onInstantRequestCallback bad msg formating");
    this->sendCommandRecordingAnswer("AnswerInstantRecording", query, emptyContextValues);
    _isPerformingInstantRecording = false;
    return;
  }

  // Add a new answer copying the initial query to answers preparation queue
  this->addNewRecordingAnswer(query);

  // Generation
  const bool isGen = this->generateInstantRecordings(query);
  if (isGen == false)
  {
    HEEX_LOG(error) << this->getUuid() << "::onInstantRequestCallback bad query generation.";
    this->reportIncident("onInstantRequestCallback query context values generation failed");
    this->sendCommandRecordingAnswer("AnswerInstantRecording", query, emptyContextValues);
    _isPerformingInstantRecording = false;
    return;
  }

  const Heex::RecorderArgs::RecorderArgsV2 answer             = this->getRecordingAnswer(query);
  std::vector<Heex::RecorderArgs::ContextValue> contextValues = answer.contextValues;
  if (contextValues.size() == 0)
  {
    contextValues.push_back(Heex::RecorderArgs::ContextValue(Heex::RecorderTools::HEEX_NO_CONTEXT_VALUE, ""));
  }

  // Encoding
  std::string res;
  try
  {
    res = Heex::RecorderTools::encodeContextValuesAsString(contextValues);
  }
  catch (const std::exception& e)
  {
    HEEX_LOG(error) << this->getUuid() << "::onInstantRequestCallback bad contextValues encoding: " << e.what();
    this->reportIncident("onInstantRequestCallback bad contextValues encoding");

    this->sendCommandRecordingAnswer("AnswerInstantRecording", query, emptyContextValues);
    _isPerformingInstantRecording = false;
    return;
  }

  // Sending
  this->sendCommandRecordingAnswer("AnswerInstantRecording", answer, res);

  _isPerformingInstantRecording = false;
}

void RecorderV2::onTimeFrameRequestCallback(const std::string& msg)
{
  HEEX_LOG(debug) << "RecorderV2::onTimeFrameRequestCallback : " << msg;
  _isPerformingTimeFrameRecording      = true;
  // Backup empty contextValues
  const std::string emptyContextValues = Heex::RecorderTools::HEEX_EMPTY_CONTEXT_VALUE;

  // Parsing and decoding
  const Heex::RecorderArgs::RecorderArgsV2 query = Heex::RecorderTools::parseRecorderV2Msg(msg, Heex::RecorderArgs::RecorderCmdType::QueryV2);
  if (query.valid == false)
  {
    HEEX_LOG(error) << this->getUuid() << "::onTimeFrameRequestCallback bad msg formating.";
    this->reportIncident("onTimeFrameRequestCallback bad msg formating");
    this->sendCommandRecordingAnswer("AnswerTimeFrameRecording", query, emptyContextValues);
    _isPerformingTimeFrameRecording = false;
    return;
  }

  // Add a new answer copying the initial query to answers preparation queue
  this->addNewRecordingAnswer(query);

  // Generation
  const bool isGen = this->generateTimeFrameRecordings(query);
  if (isGen == false)
  {
    HEEX_LOG(error) << this->getUuid() << "::onTimeFrameRequestCallback bad query generation.";
    this->reportIncident("onTimeFrameRequestCallback query context values generation failed");
    this->sendCommandRecordingAnswer("AnswerTimeFrameRecording", query, emptyContextValues);
    _isPerformingTimeFrameRecording = false;
    return;
  }

  const Heex::RecorderArgs::RecorderArgsV2 answer             = this->getRecordingAnswer(query);
  std::vector<Heex::RecorderArgs::ContextValue> contextValues = answer.contextValues;
  if (contextValues.size() == 0)
  {
    contextValues.push_back(Heex::RecorderArgs::ContextValue(Heex::RecorderTools::HEEX_NO_CONTEXT_VALUE, ""));
  }

  // Encoding
  std::string res;
  try
  {
    res = Heex::RecorderTools::encodeContextValuesAsString(contextValues);
  }
  catch (const std::exception& e)
  {
    HEEX_LOG(error) << this->getUuid() << "::onTimeFrameRequestCallback bad contextValues encoding: " << e.what();
    this->reportIncident("onTimeFrameRequestCallback bad contextValues encoding");

    this->sendCommandRecordingAnswer("AnswerTimeFrameRecording", query, emptyContextValues);
    _isPerformingTimeFrameRecording = false;
    return;
  }

  // Sending
  this->sendCommandRecordingAnswer("AnswerTimeFrameRecording", answer, res);

  _isPerformingTimeFrameRecording = false;
}

void RecorderV2::sendCommandRecordingAnswer(const std::string& command, const Heex::RecorderArgs::RecorderArgsV2& answerArgs, const std::string& encodedValues)
{
  std::stringstream cmd;
  cmd << command << " " << answerArgs.uuid << " " << answerArgs.timestamp << " " << answerArgs.eventUuid << " " << answerArgs.recordIntervalStart << " "
      << answerArgs.recordIntervalEnd << " " << encodedValues << " " << Agent::encodeIncidents();
  if (_tcpC != nullptr)
  {
    _tcpC->send(cmd.str());
  }
  else
  {
    HEEX_LOG(error) << "Could not send message : " << cmd.str();
  }

  HEEX_LOG(trace) << this->getUuid() << "::sendCommandAnswer | Message: " << cmd.str() << std::endl; // DEBUG
}

// This method aims to be overloaded by customer implementations.
bool RecorderV2::generateInstantRecordings(const Heex::RecorderArgs::RecorderArgsV2& query)
{
  HEEX_LOG(warning) << this->getUuid() << "::generateInstantRecordings | Could not generate instant recordings for event " << query.eventUuid << " at time " << query.timestamp
                    << " method is not implemented." << std::endl;
  // Don't use this class directly to implement you own recorder.
  // You need to inherate from the RecorderV2 class and override this function.
  // See the getStartedRecorder examples.
  return false;
}

// This method aims to be overloaded by customer implementations.
bool RecorderV2::generateTimeFrameRecordings(const Heex::RecorderArgs::RecorderArgsV2& query)
{
  HEEX_LOG(warning) << this->getUuid() << "::generateTimeFrameRecordings | Could not generate time frame recordings for event " << query.eventUuid << " at time " << query.timestamp
                    << " method is not implemented." << std::endl;
  // Don't use this class directly to implement you own recorder.
  // You need to inherate from the RecorderV2 class and override this function.
  // See the getStartedRecorder examples.
  return false;
}

bool RecorderV2::addContextValue(std::vector<Heex::RecorderArgs::ContextValue>& res, const std::string key, const std::string value)
{
  return Heex::RecorderTools::addContextValue(res, key, value);
}

bool RecorderV2::addContextValue(const Heex::RecorderArgs::RecorderArgsV2& query, const std::string key, const std::string value)
{
  _recordingAnswerQueue_m.lock();

  std::vector<Heex::RecorderArgs::RecorderArgsV2>::iterator answerIt = _recordingAnswerQueue.end();
  for (std::vector<Heex::RecorderArgs::RecorderArgsV2>::iterator it = _recordingAnswerQueue.begin(); it != _recordingAnswerQueue.end(); ++it)
  {
    // Try to match the query using two of its immutable fields
    if (it->eventUuid == query.eventUuid && it->timestamp == query.timestamp)
    {
      answerIt = it;
    }
  }

  // Guard if answer not found for the given query
  if (answerIt == _recordingAnswerQueue.end())
  {
    HEEX_LOG(error) << "RecorderV2::addContextValue | Failed to retrieve answer from the queue for event " << query.eventUuid << '\n';
    _recordingAnswerQueue_m.unlock();
    return false;
  }

  // Modify the fields
  const bool ret = this->addContextValue(answerIt->contextValues, key, value);

  _recordingAnswerQueue_m.unlock();
  return ret;
}

bool RecorderV2::addRecordingPart(const Heex::RecorderArgs::RecorderArgsV2& query, const std::string recordingPartPath)
{
  return this->addContextValue(query, Heex::RecorderTools::HEEX_RECORDING_PART_KEY_PREFIX + query.uuid, recordingPartPath);
}

std::vector<std::string> RecorderV2::getContextValueKeys(const Heex::RecorderArgs::RecorderArgsV2& query)
{
  std::vector<std::string> result;
  for (const Heex::RecorderArgs::ContextValue& cv : query.contextValues)
  {
    result.push_back(cv.key);
  }
  return result;
}

void RecorderV2::onConfigurationChanged(const std::string& msg)
{
  Agent::onConfigurationChanged(msg);
}

bool RecorderV2::isPerformingCallback()
{
  // A callback operation can be performed either for ContextValue or for the EventRecordingParts. Some extra checks have been added with Answer queue for EventRecordingPart.
  _recordingAnswerQueue_m.lock();
  const bool r = _isPerformingInstantRecording || _isPerformingTimeFrameRecording || _recordingAnswerQueue.size() != 0;
  _recordingAnswerQueue_m.unlock();
  return r;
}

void RecorderV2::addNewRecordingAnswer(const Heex::RecorderArgs::RecorderArgsV2& query)
{
  //TODO : Do check if already exist ?
  _recordingAnswerQueue_m.lock();
  _recordingAnswerQueue.push_back(query);
  _recordingAnswerQueue_m.unlock();
}

bool RecorderV2::setRealRecordIntervalRange(const Heex::RecorderArgs::RecorderArgsV2& query, const std::string& realRecordIntervalStart, const std::string& realRecordIntervalEnd)
{
  _recordingAnswerQueue_m.lock();

  std::vector<Heex::RecorderArgs::RecorderArgsV2>::iterator answerIt = _recordingAnswerQueue.end();
  for (std::vector<Heex::RecorderArgs::RecorderArgsV2>::iterator it = _recordingAnswerQueue.begin(); it != _recordingAnswerQueue.end(); ++it)
  {
    // Try to match the query using two of its immutable fields
    if (it->eventUuid == query.eventUuid && it->timestamp == query.timestamp)
    {
      answerIt = it;
    }
  }

  // Guard if answer not found for the given query
  if (answerIt == _recordingAnswerQueue.end())
  {
    HEEX_LOG(error) << "RecorderV2::setRealRecordIntervalRange | Failed to retrieve answer from the queue for event " << query.eventUuid << '\n';
    _recordingAnswerQueue_m.unlock();
    return false;
  }

  // Modify the fields
  answerIt->recordIntervalStart = realRecordIntervalStart;
  answerIt->recordIntervalEnd   = realRecordIntervalEnd;
  HEEX_LOG(debug) << this->getUuid() << "::setRealRecordIntervalRange | RecordInterval fields have been changed to [" << answerIt->recordIntervalStart << ";"
                  << answerIt->recordIntervalStart << "]" << std::endl; // DEBUG

  _recordingAnswerQueue_m.unlock();
  return true;
}

const Heex::RecorderArgs::RecorderArgsV2 RecorderV2::getRecordingAnswer(const Heex::RecorderArgs::RecorderArgsV2& query)
{
  return this->popRecordingAnswer(query, false);
}

const Heex::RecorderArgs::RecorderArgsV2 RecorderV2::popRecordingAnswer(const Heex::RecorderArgs::RecorderArgsV2& query, bool removeFlag)
{
  _recordingAnswerQueue_m.lock();
  std::vector<Heex::RecorderArgs::RecorderArgsV2>::const_iterator answerIt = _recordingAnswerQueue.cend();
  for (std::vector<Heex::RecorderArgs::RecorderArgsV2>::const_iterator it = _recordingAnswerQueue.cbegin(); it != _recordingAnswerQueue.cend(); ++it)
  {
    // Try to match the query using two of its immutable fields
    if (it->eventUuid == query.eventUuid && it->timestamp == query.timestamp)
    {
      answerIt = it;
    }
  }

  // Guard if answer not found for the given query. Returns an invalid empty answer if this is the case.
  if (answerIt == _recordingAnswerQueue.cend())
  {
    HEEX_LOG(error) << "RecorderV2::popRecordingAnswer | Failed to retrieve answer from the queue for event " << query.eventUuid << '\n';
    _recordingAnswerQueue_m.unlock();
    Heex::RecorderArgs::RecorderArgsV2 invalidAnswer;
    invalidAnswer.valid = false;
    return invalidAnswer;
  }
  const Heex::RecorderArgs::RecorderArgsV2 answer = *answerIt;

  // Remove it if enabled.
  if (removeFlag)
  {
    _recordingAnswerQueue.erase(answerIt); // Remove it from the queue
  }

  _recordingAnswerQueue_m.unlock();
  return answer;
}

const std::vector<Heex::RecorderArgs::RecorderRangesValues>& RecorderV2::getRecorderRangesRegistry()
{
  return _recorderRangesRegistry;
}

size_t RecorderV2::getSizeOfRecordingAnswerQueue()
{
  _recordingAnswerQueue_m.lock();
  const size_t res = _recordingAnswerQueue.size();
  _recordingAnswerQueue_m.unlock();
  return res;
}
