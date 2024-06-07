///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
///
/// @file Recorder.cpp
/// @brief Library source file that contains the basic structure for any Recorders to appropriately register and communicate with the Smart Data Engine. Works for both Real-time and Datalake applications.
#include "Recorder.h"

#include "HeexUtilsFileOperations.h"
#include "HeexUtilsLog.h"
#include "RecorderTools.h"
#include "Tools.h"

Recorder::Recorder(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion)
    : RecorderV2(uuid, serverIp, serverPort, implementationVersion)
{
  _agentType = Heex::HEEX_AGENT_TYPE_DEFAULT;

  if (_tcpC != nullptr)
  {
    _tcpC->subscribeToCmd("QueryContextValue", std::bind(&Recorder::onContextValueRequestCallback, this, std::placeholders::_1));
    _tcpC->subscribeToCmd("QueryEventRecordingPart", std::bind(&Recorder::onEventRecordingPartRequestCallback, this, std::placeholders::_1));
  }

  _isPerformingContextValue        = false;
  _isPerformingEventRecordingParts = false;

  // Log a warning message if the Recorder UUID doesn't match the expected type and excluding the configuration value prefix.
  std::string warningMsg;
  if (!Heex::Tools::checkAgentNonConsistentUuidType(this->getUuid(), Heex::Tools::UuidPrefixType::Recorder, {"SV"}, warningMsg))
  {
    HEEX_LOG(warning) << warningMsg;
  }
}

Recorder::~Recorder() = default;

std::vector<Heex::RecorderArgs::ContextValue> Recorder::prepareContextValues(const Heex::RecorderArgs::RecorderContextValueArgs& query)
{
  return query.contextValues;
}

bool Recorder::addContextValue(std::vector<Heex::RecorderArgs::ContextValue>& res, const std::string key, const std::string value)
{
  return Heex::RecorderTools::addContextValue(res, key, value);
}

bool Recorder::addGNSSContextValue(std::vector<Heex::RecorderArgs::ContextValue>& res, const std::string key, const double latitude, const double longitude)
{
  std::stringstream posContextValue;
  posContextValue << std::fixed;
  posContextValue << std::setprecision(10);
  posContextValue << latitude << ", " << longitude;
  return Heex::RecorderTools::addContextValue(res, key, posContextValue.str());
}

std::vector<std::string> Recorder::getContextValueKeys(const Heex::RecorderArgs::RecorderContextValueArgs& query)
{
  std::vector<std::string> result;
  for (const Heex::RecorderArgs::ContextValue& cv : query.contextValues)
  {
    result.push_back(cv.key);
  }
  return result;
}

void Recorder::onConfigurationChanged(const std::string& msg)
{
  _recorderRangesRegistry.clear();

  Agent::onConfigurationChanged(msg);
  const std::vector<std::string> dynamicConfigValues = this->retrieveDynamicConfigValues(Heex::HEEX_RECORDING_RANGE_KEY);
  for (const std::string& value : dynamicConfigValues)
  {
    const Heex::RecorderArgs::RecorderRangesValues res = Heex::RecorderTools::parseRecorderRangesMsg(value);
    if (res.valid == true)
    {
      _recorderRangesRegistry.push_back(res);
    }
  }
}
///
/// ContextValue Query & Answer

void Recorder::onContextValueRequestCallback(const std::string& msg)
{
  // Set _isPerformingContextValue to true
  _isPerformingContextValue            = true;
  // Backup empty contextValues
  const std::string emptyContextValues = Heex::RecorderTools::HEEX_EMPTY_CONTEXT_VALUE;

  // Parsing and decoding
  const Heex::RecorderArgs::RecorderContextValueArgs query = Heex::RecorderTools::parseRecorderContextValueMsg(msg, Heex::RecorderArgs::RecorderCmdType::QueryContextValue);
  if (query.valid == false)
  {
    HEEX_LOG(error) << this->getUuid() << "::onContextValueRequestCallback bad msg formating.";
    this->reportIncident("onContextValueRequestCallback bad msg formating");
    this->sendContextValueAnswer(query, emptyContextValues);
    // Set _isPerformingContextValue to false
    _isPerformingContextValue = false;
    return;
  }

  // Generation
  std::vector<Heex::RecorderArgs::ContextValue> contextValues = this->prepareContextValues(query);
  const bool isGen                                            = this->generateRequestedValues(query, contextValues);
  if (isGen == false)
  {
    HEEX_LOG(error) << this->getUuid() << "::onContextValueRequestCallback bad query generation.";
    this->reportIncident("onContextValueRequestCallback query context values generation failed");
    this->sendContextValueAnswer(query, emptyContextValues);
    // Set _isPerformingContextValue to false
    _isPerformingContextValue = false;
    return;
  }

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
    HEEX_LOG(error) << this->getUuid() << "::onContextValueRequestCallback bad contextValues encoding: " << e.what();
    this->reportIncident("onContextValueRequestCallback bad contextValues encoding");

    this->sendContextValueAnswer(query, emptyContextValues);
    // Set _isPerformingContextValue to false
    _isPerformingContextValue = false;
    return;
  }

  // Sending
  this->sendContextValueAnswer(query, res);

  // Set _isPerformingContextValue to false
  _isPerformingContextValue = false;
}

void Recorder::sendContextValueAnswer(const Heex::RecorderArgs::RecorderContextValueArgs& valueQuery, const std::string& encodedValues)
{
  // Sending the command well-formatted for the ContextValueAnswer
  std::stringstream cmd;
  cmd << "AnswerContextValue"
      << " " << valueQuery.uuid << " " << valueQuery.timestamp << " " << valueQuery.eventUuid << " " << encodedValues << " " << Agent::encodeIncidents();
  if (_tcpC != nullptr)
  {
    _tcpC->send(cmd.str());
  }
  else
  {
    HEEX_LOG(error) << "Could not send message : " << cmd.str();
  }

  HEEX_LOG(debug) << this->getUuid() << "::sendContextValueAnswer | Message: " << cmd.str() << std::endl; // DEBUG
}

///
/// DataFilePath Query & Answer

void Recorder::onEventRecordingPartRequestCallback(const std::string& msg)
{
  // Set _isPerformingEventRecordingParts to true
  _isPerformingEventRecordingParts = true;
  // Backup filepath
  const std::string emptyFilepath  = Heex::RecorderTools::HEEX_EMPTY_FILEPATH;

  // Parsing and decoding
  const Heex::RecorderArgs::RecorderEventRecordingPartArgs query =
      Heex::RecorderTools::parseRecorderDataFilePathMsg(msg, Heex::RecorderArgs::RecorderCmdType::QueryEventRecordingPart);
  if (query.valid == false)
  {
    HEEX_LOG(error) << this->getUuid() << "::onEventRecordingPartRequestCallback bad msg formating.";
    this->reportIncident("onEventRecordingPartRequestCallback bad msg formating");

    this->sendEventRecordingPartAnswer(query, emptyFilepath);
    // Set _isPerformingEventRecordingParts to false
    _isPerformingEventRecordingParts = false;
    return;
  }

  // Add a new answer copying the initial EventRecordingPart query to answers preparation queue
  this->addNewEventRecordingPartAnswer(query);

  // Generation
  std::string filepath;
  const bool isGen = this->generateRequestedFilePaths(query, filepath);

  if (isGen == false)
  {
    // Pop answer from queue on failure: Prevent leaving without removing it as it can comprmiise the isPerformingCallback() call.
    const Heex::RecorderArgs::RecorderEventRecordingPartArgs answer = this->popEventRecordingPartAnswer(query, true);

    HEEX_LOG(error) << this->getUuid()
                    << "::onEventRecordingPartRequestCallback ERROR : generateRequestedFilePaths has provided an "
                       "incorrect output or is not implemented."
                    << filepath;
    this->reportIncident("onEventRecordingPartRequestCallback generateRequestedFilePaths has provided an incorrect output");

    this->sendEventRecordingPartAnswer(query, emptyFilepath);
    // Set _isPerformingEventRecordingParts to false
    _isPerformingEventRecordingParts = false;
    return;
  }

  //Check if there is home directory represented by ~, if so expand it
  if (!filepath.empty() && filepath[0] == '~')
  {
    HeexUtils::FileOperations::expandTilde(filepath);
  }

  // Encoding
  Heex::Tools::encodeEscapeCharactersWithReplacementCode(filepath);

  // Extract and remove the answer from the EventRecordingPart answers preparation queue
  const Heex::RecorderArgs::RecorderEventRecordingPartArgs answer = this->popEventRecordingPartAnswer(query, true);
  if (answer.valid == false)
  {
    HEEX_LOG(error) << this->getUuid() << "::onEventRecordingPartRequestCallback | Failed to retrieve answer from the queue for event " << query.eventUuid << '\n';
    this->reportIncident("onEventRecordingPartRequestCallback : Failed to retrieve answer from the queue");
    this->sendEventRecordingPartAnswer(query, emptyFilepath);
    // Set _isPerformingEventRecordingParts to false
    _isPerformingEventRecordingParts = false;
    return;
  }

  // Sending
  this->sendEventRecordingPartAnswer(answer, filepath);

  // Set _isPerformingEventRecordingParts to false
  _isPerformingEventRecordingParts = false;
}

void Recorder::sendEventRecordingPartAnswer(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& fileQuery, const std::string& encodedFilepaths)
{
  // Sending the command well-formatted for the ContextValueAnswer
  std::stringstream cmd;
  cmd << "AnswerEventRecordingPart"
      << " " << fileQuery.uuid << " " << fileQuery.timestamp << " " << fileQuery.eventUuid << " " << fileQuery.recordIntervalStart << " " << fileQuery.recordIntervalEnd << " "
      << encodedFilepaths << " " << Agent::encodeIncidents();
  if (_tcpC != nullptr)
  {
    _tcpC->send(cmd.str());
  }
  else
  {
    HEEX_LOG(error) << "Could not send message : " << cmd.str();
  }

  HEEX_LOG(debug) << this->getUuid() << "::sendEventRecordingPartAnswer | Message: " << cmd.str() << std::endl; // DEBUG
}

bool Recorder::generateRequestedValues(const Heex::RecorderArgs::RecorderContextValueArgs& /*query*/, std::vector<Heex::RecorderArgs::ContextValue>& /*contextValue*/)
{
  // Don't use this class directly to implement you own recorder.
  // You need to inherate from the Recorder class and override this function.
  // See the getStartedRecorder examples and the Recorder documentation in the README.md file.
  return true;
}

bool Recorder::generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& /*filepath*/)
{
  HEEX_LOG(error) << this->getUuid() << "::generateRequestedFilePaths | Error while trying to generate DataFilePath for event " << query.eventUuid << " at time " << query.timestamp
                  << ". Will be discarded." << std::endl;
  // Don't use this class directly to implement you own recorder.
  // You need to inherate from the Recorder class and override this function.
  // See the getStartedRecorder examples and the Recorder documentation in the README.md file.
  return false;
}

bool Recorder::generateInstantRecordings(const Heex::RecorderArgs::RecorderArgsV2& queryV2)
{
  HEEX_LOG(debug) << this->getUuid() << "::generateInstantRecordings | Call bridge on legacy requestedValue to generate instant recordings for event " << queryV2.eventUuid
                  << " at time " << queryV2.timestamp << "." << std::endl;

  Heex::RecorderArgs::RecorderContextValueArgs queryV1;
  queryV1.valid         = queryV2.valid;
  queryV1.uuid          = queryV2.uuid;
  queryV1.eventUuid     = queryV2.eventUuid;
  queryV1.timestamp     = queryV2.timestamp;
  queryV1.contextValues = queryV2.contextValues;
  queryV1.unparsedArgs  = queryV2.unparsedArgs;
  queryV1.incidents     = queryV2.incidents;

  std::vector<Heex::RecorderArgs::ContextValue> answerCv = queryV2.contextValues;
  const bool ret                                         = this->generateRequestedValues(queryV1, answerCv);

  for (const Heex::RecorderArgs::ContextValue& newCv : answerCv)
  {
    RecorderV2::addContextValue(queryV2, newCv.key, newCv.value);
  }

  return ret;
}

bool Recorder::generateTimeFrameRecordings(const Heex::RecorderArgs::RecorderArgsV2& queryV2)
{
  HEEX_LOG(debug) << this->getUuid()
                  << "::generateTimeFrameRecordings | Call bridge on legacy requestedFilePaths to generate time frame "
                     "recordings for event "
                  << queryV2.eventUuid << " at time " << queryV2.timestamp << "." << std::endl;

  std::vector<Heex::RecorderArgs::ContextValue> emptyCv;
  /// Push special string indicating that no contextValues was set. Used purely for uniform format
  emptyCv.push_back(Heex::RecorderArgs::ContextValue(Heex::RecorderTools::HEEX_NO_CONTEXT_VALUE, std::string()));

  Heex::RecorderArgs::RecorderEventRecordingPartArgs queryV1;
  queryV1.valid               = queryV2.valid;
  queryV1.uuid                = queryV2.uuid;
  queryV1.eventUuid           = queryV2.eventUuid;
  queryV1.timestamp           = queryV2.timestamp;
  queryV1.recordIntervalStart = queryV2.recordIntervalStart;
  queryV1.recordIntervalEnd   = queryV2.recordIntervalEnd;
  queryV1.unparsedArgs        = queryV2.unparsedArgs;
  queryV1.incidents           = queryV2.incidents;

  std::string filepath;
  const bool ret = this->generateRequestedFilePaths(queryV1, filepath);

  if (filepath.empty() == false)
  {
    RecorderV2::addRecordingPart(queryV2, filepath);
  }

  return ret;
}

bool Recorder::isPerformingCallback()
{
  // A callback operation can be performed either for ContextValue or for the EventRecordingParts. Some extra checks have been added with Answer queue for EventRecordingPart.
  _eventRecordingPartAnswerQueue_m.lock();
  const bool r = _isPerformingContextValue || _isPerformingEventRecordingParts || _eventRecordingPartAnswerQueue.size() != 0;
  _eventRecordingPartAnswerQueue_m.unlock();
  return r;
}

void Recorder::addNewEventRecordingPartAnswer(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query)
{
  //TODO : Do check if already exist ?
  _eventRecordingPartAnswerQueue_m.lock();
  _eventRecordingPartAnswerQueue.push_back(query);
  _eventRecordingPartAnswerQueue_m.unlock();
}

bool Recorder::setRealRecordIntervalRange(
    const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query,
    const std::string& realRecordIntervalStart,
    const std::string& realRecordIntervalEnd)
{
  _eventRecordingPartAnswerQueue_m.lock();

  std::vector<Heex::RecorderArgs::RecorderEventRecordingPartArgs>::iterator answerIt = _eventRecordingPartAnswerQueue.end();
  for (std::vector<Heex::RecorderArgs::RecorderEventRecordingPartArgs>::iterator it = _eventRecordingPartAnswerQueue.begin(); it != _eventRecordingPartAnswerQueue.end(); ++it)
  {
    // Try to match the query using two of its immutable fields
    if (it->eventUuid == query.eventUuid && it->timestamp == query.timestamp)
    {
      answerIt = it;
    }
  }

  // Guard if answer not found for the given query
  if (answerIt == _eventRecordingPartAnswerQueue.end())
  {
    HEEX_LOG(error) << this->getUuid() << "::setRealRecordIntervalRange | Failed to retrieve answer from the queue for event " << query.eventUuid << '\n';
    _eventRecordingPartAnswerQueue_m.unlock();
    return false;
  }

  // Modify the fields
  answerIt->recordIntervalStart = realRecordIntervalStart;
  answerIt->recordIntervalEnd   = realRecordIntervalEnd;
  HEEX_LOG(debug) << this->getUuid() << "::setRealRecordIntervalRange | RecordInterval fields have been changed to [" << answerIt->recordIntervalStart << ";"
                  << answerIt->recordIntervalStart << "]" << std::endl; // DEBUG

  _eventRecordingPartAnswerQueue_m.unlock();
  return true;
}

const Heex::RecorderArgs::RecorderEventRecordingPartArgs Recorder::getEventRecordingPartAnswer(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query)
{
  return this->popEventRecordingPartAnswer(query, false);
}

const Heex::RecorderArgs::RecorderEventRecordingPartArgs Recorder::popEventRecordingPartAnswer(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, bool removeFlag)
{
  _eventRecordingPartAnswerQueue_m.lock();
  std::vector<Heex::RecorderArgs::RecorderEventRecordingPartArgs>::const_iterator answerIt = _eventRecordingPartAnswerQueue.cend();
  for (std::vector<Heex::RecorderArgs::RecorderEventRecordingPartArgs>::const_iterator it = _eventRecordingPartAnswerQueue.cbegin(); it != _eventRecordingPartAnswerQueue.cend();
       ++it)
  {
    // Try to match the query using two of its immutable fields
    if (it->eventUuid == query.eventUuid && it->timestamp == query.timestamp)
    {
      answerIt = it;
    }
  }

  // Guard if answer not found for the given query. Returns an invalid empty answer if this is the case.
  if (answerIt == _eventRecordingPartAnswerQueue.cend())
  {
    HEEX_LOG(error) << this->getUuid() << "::onEventRecordingPartRequestCallback | Failed to retrieve answer from the queue for event " << query.eventUuid << '\n';
    _eventRecordingPartAnswerQueue_m.unlock();
    Heex::RecorderArgs::RecorderEventRecordingPartArgs invalidAnswer;
    invalidAnswer.valid = false;
    return invalidAnswer;
  }
  const Heex::RecorderArgs::RecorderEventRecordingPartArgs answer = *answerIt;

  // Remove it if enabled.
  if (removeFlag)
  {
    _eventRecordingPartAnswerQueue.erase(answerIt); // Remove it from the queue
  }

  _eventRecordingPartAnswerQueue_m.unlock();
  return answer;
}

const std::vector<Heex::RecorderArgs::RecorderRangesValues>& Recorder::getRecorderRangesRegistry()
{
  return _recorderRangesRegistry;
}

size_t Recorder::getSizeOfRecordingPartAnswerQueue()
{
  _eventRecordingPartAnswerQueue_m.lock();
  const size_t res = _eventRecordingPartAnswerQueue.size();
  _eventRecordingPartAnswerQueue_m.unlock();
  return res;
}
