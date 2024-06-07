///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file RecorderInThePast.cpp
/// @brief Class that allows the recorder to provide recordings in the past for recorders that does do support it naturally
/// The Recorder asks at regular intervals the client function to generate recording continously.
/// The recording are store in a buffer and the older not used by a query are deleted.
/// The recording are send to the SDE using symlinks
/// @date 2022-12-07
///
#include "RecorderInThePast.h"

#include <cmath>
#include <sstream>

#include "HeexUtilsFileOperations.h"
#include "HeexUtilsLog.h"
#include "RecorderTools.h"
#include "Tools.h"

///
/// Possible improvements
/// Evaluate the possibility to refactor the RecorderInThePast as class that is used in Recorder class (like snapshotter class)
///  * Depending on the values of recording ranges, recorder will either launch or stop the thread that runs the object
///
/// End of the program : The program does not stop untill all chunks have been deleted, this will not work in a non test setting
/// A solution would be to delete all files that can be delete, and dump the others files(chunks) into a persited json file.
/// In addition the chunks to delete, the program will also delete the files(chunks) that are the json file.

/// INFO
/// If the operations done after the thread returns from generatedRequestedFilepaths results in chunk not overlapping,
///  one needs to increase the worker size
const int RecorderInThePast::NB_WORKER                          = 3;
const int RecorderInThePast::CHUNK_BUFFER_SIZE                  = 2;
const double RecorderInThePast::LEN_RECORDING_CHUNK_ADJUSTEMENT = 0.1;

RecorderInThePast::RecorderInThePast(
    const std::string& uuid,
    const std::string& serverIp,
    const unsigned int& serverPort,
    const bool& isRecordingInThePastSupported,
    const std::string& implementationVersion)
    : Recorder(uuid, serverIp, serverPort, implementationVersion),
      _isRecordingInThePastSupported(isRecordingInThePastSupported)

{
  _garbageWaitDelay                  = 1;
  _isWorkerAllowed                   = false;
  _isRecordingInThePast              = false;
  _isGarbageCollectorWorkerAllowed   = true;
  _isPerformingGetFilePathFromBuffer = false;
  _garbageCollectorThread            = std::thread(&RecorderInThePast::deleteChunks, this);
}

void RecorderInThePast::stopChunkGeneratorsWorkers()
{
  _isWorkerAllowed = false;
  for (std::thread& t : _chunkThreads)
  {
    if (t.joinable())
    {
      t.join();
    }
  }
  _chunkThreads.clear();
  {
    const std::lock_guard<std::mutex> lk(_recordingChunkJobQueueMutex);
    std::queue<RecordingChunkJob> empty;
    std::swap(_recordingChunkJobQueue, empty);
  }
  HEEX_LOG(info) << "RecorderInThePast::stopChunkGeneratorsWorkers | Threads are shutdown ";
}

void RecorderInThePast::stopGarbageCollectorWorker()
{
  // Insert chunks of recording buffer into chunkGarbage for deletion
  _garbageBufferMutex.lock();
  std::vector<RecorderEventRecordingChunk>::iterator it = begin(_recordingBuffer);
  while (it != end(_recordingBuffer))
  {
    _chunkGarbage.push_back(*it);
    it = _recordingBuffer.erase(it);
  }
  _garbageBufferMutex.unlock();
  _isGarbageCollectorWorkerAllowed = false;

  // TODO (improve garbage collector) the call below will prevent the program to finish untill all uploads have been done.
  // Wait for garbage to delete all chunks
  HEEX_LOG(debug) << "RecorderInThePast::~RecorderInThePast | waiting for GC " << '\n';
  if (_garbageCollectorThread.joinable())
  {
    _garbageCollectorThread.join();
  }
  HEEX_LOG(debug) << "RecorderInThePast::~RecorderInThePast | GC finished " << '\n';
}
RecorderInThePast::~RecorderInThePast()
{
  this->stopChunkGeneratorsWorkers();
  this->stopGarbageCollectorWorker();
}

void RecorderInThePast::onConfigurationChanged(const std::string& msg)
{
  Recorder::onConfigurationChanged(msg);
  if (_isRecordingInThePastSupported == false)
  {
    this->configureWorkers();
  }
}

void RecorderInThePast::onIdentificationAccepted(const std::string& msg)
{
  Recorder::onIdentificationAccepted(msg);
  std::vector<std::string> configValues = this->retrieveStaticConfigValues(Heex::HEEX_SDE_WORKING_DIRECTORY);
  if (configValues.size() >= 1)
  {
    _sdeWorkingDirectory = configValues[0];
  }
}
void RecorderInThePast::configureWorkers()
{
  this->createWorkingDirectory();
  if (_chunksLinksLocation.empty())
  {
    HEEX_LOG(info) << "RecorderInThePast::configureWorkers | working folder creation failed or is not set. Workers "
                      "won't be launched";
    HEEX_LOG(info) << "RecorderInThePast::configureWorkers | Fallback mode (normal recording) is activated";
    _isRecordingInThePast = false;
    return;
  }

  // Reset
  this->stopChunkGeneratorsWorkers();
  _isRecordingInThePast = false;

  // Determine min and max recording ranges of the recorder
  double minRecordingStart                                                            = 0;
  double maxRecordingEnd                                                              = 0;
  const std::vector<Heex::RecorderArgs::RecorderRangesValues>& recorderRangesRegistry = this->getRecorderRangesRegistry();
  for (const Heex::RecorderArgs::RecorderRangesValues& range : recorderRangesRegistry)
  {
    minRecordingStart = std::min(minRecordingStart, range.recordIntervalStart);
    maxRecordingEnd   = std::max(maxRecordingEnd, range.recordIntervalEnd);
  }

  const double tmpLenRecordingChunk = std::abs(maxRecordingEnd - minRecordingStart);
  if (tmpLenRecordingChunk == 0 || minRecordingStart >= 0)
  {
    HEEX_LOG(info) << "RecorderInThePast::configureWorkers | recording intervals difference is null or min recording "
                      "start is positive";
    HEEX_LOG(info) << "RecorderInThePast::configureWorkers | Fallback mode (normal recording) is activated";
    return;
  }
  else
  {
    double seconds{};
    const double remaining = std::modf(tmpLenRecordingChunk, &seconds);
    const int milliseconds = static_cast<int>(remaining * 1000);
    _lenRecordingChunk     = boost::posix_time::seconds(static_cast<int>(seconds)) + boost::posix_time::milliseconds(milliseconds);
  }

  const RecordingChunkJob initJob{boost::posix_time::microsec_clock::universal_time()};
  {
    const std::lock_guard<std::mutex> lk(_recordingChunkJobQueueMutex);
    _recordingChunkJobQueue.push(initJob);
  }

  _isWorkerAllowed = true;

  for (int i = 0; i < NB_WORKER; ++i)
  {
    _chunkThreads.push_back(std::thread(&RecorderInThePast::doRecordingChunksJobs, this));
  }
  _isRecordingInThePast = true;
  HEEX_LOG(info) << "RecorderInThePast | configureWorkers: threads are launched";
}
void RecorderInThePast::doRecordingChunksJobs()
{
  // Variable to adjust the recording interval
  const double recordingIntervalAdjustement = LEN_RECORDING_CHUNK_ADJUSTEMENT * (_lenRecordingChunk.total_milliseconds());
  while (_isWorkerAllowed)
  {
    RecordingChunkJob jobTodo;
    bool jobFound = false;

    {
      // take a job in queue, add next job, execute current job
      const std::lock_guard<std::mutex> lk(_recordingChunkJobQueueMutex);
      if (!_recordingChunkJobQueue.empty())
      {
        // Take jobs, check the time, either sleep or launch
        jobTodo = _recordingChunkJobQueue.front();
        _recordingChunkJobQueue.pop();
        jobFound = true;

        RecordingChunkJob nextJobTodo{jobTodo};
        // Set the start before, as it might take time to wakeup the thread
        nextJobTodo.recordingStart = jobTodo.recordingStart + _lenRecordingChunk - boost::posix_time::milliseconds(static_cast<int>(recordingIntervalAdjustement));
        _recordingChunkJobQueue.push(nextJobTodo);
      }
    }
    if (jobFound == true)
    {
      Heex::RecorderArgs::RecorderEventRecordingPartArgs query;
      query.valid                    = true;
      query.recordIntervalStart      = std::to_string(0);
      const double recordingDuration = (2 * static_cast<double>(_lenRecordingChunk.total_milliseconds()) / 1000) + (recordingIntervalAdjustement / 1000);
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2) << recordingDuration;
      query.recordIntervalEnd = ss.str();
      std::string filepath    = "";

      const boost::posix_time::ptime currTime = boost::posix_time::microsec_clock::universal_time();
      if (jobTodo.recordingStart > currTime)
      {
        const long sleepTime = static_cast<long>((jobTodo.recordingStart - currTime).total_milliseconds());
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
      }

      query.timestamp = this->getTimestampStr();
      if (this->generateRequestedFilePaths(query, filepath) == false)
      {
        HEEX_LOG(debug) << "RecorderInThePast | doRecordingChunksJobs has provided an incorrect output or is not implemented." << filepath;
      }
      else
      {
        this->sendRecordingChunkToBuffer(query, filepath);
      }
    }
  }
}

void RecorderInThePast::deleteChunks()
{
  std::unique_lock<std::mutex> lk(_garbageBufferMutex, std::defer_lock);
  bool stopGarbageCollector = !_isGarbageCollectorWorkerAllowed;

  while (stopGarbageCollector == false)
  {
    lk.lock();
    std::vector<RecorderEventRecordingChunk> toDelete;
    std::vector<RecorderEventRecordingChunk>::iterator it = begin(_chunkGarbage);

    // mark all chunks that can be deleted
    while (it != end(_chunkGarbage))
    {
      if (this->canRemoveChunk((*it)) == true)
      {
        toDelete.push_back(*it);
        it = _chunkGarbage.erase(it);
      }
      else
      {
        ++it;
      }
    }

    // Stop only when container is empty and signal to stop is set
    if (_chunkGarbage.empty() && _isGarbageCollectorWorkerAllowed == false)
    {
      stopGarbageCollector = true;
    }
    else
    {
      stopGarbageCollector = false;
    }
    lk.unlock();

    it = begin(toDelete);
    while (it != end(toDelete))
    {
      boost::system::error_code ec;
      if (boost::filesystem::remove_all(it->filepath, ec) == false)
      {
        HEEX_LOG(debug) << "RecorderInThePast::deleteChunks | Can't delete" + HeexUtils::FileOperations::getUtf8EncodedPath(it->filepath) +
                               ". Filesystem error caused by: " + ec.message();
        it++;
      }
      else
      {
        it = toDelete.erase(it);
      }
    }

    // TODO Improve using condition variable for notification
    std::this_thread::sleep_for(std::chrono::seconds(_garbageWaitDelay));
  }
  HEEX_LOG(info) << "Garbage collector finished ";
}
void RecorderInThePast::sendRecordingChunkToBuffer(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath)
{
  RecorderEventRecordingChunk recordingChunk;
  const RecorderEventRecordingChunk recordingChunkToDelete;
  const char osPathDelimiter = '/';

  if (filepath.empty())
  {
    return;
  }
  try
  {
    // Check the case where the recording file path is a folder and take everything (thanks to '/*') inside it to build the archive
    if (filepath.size() > 2 && filepath.at(filepath.size() - 2) == osPathDelimiter && filepath.at(filepath.size() - 1) == '*')
    {
      filepath.pop_back(); // Remove *
      filepath.pop_back(); // Remove /
    }
    if (HeexUtils::FileOperations::isFileExistAndAccess(filepath) || HeexUtils::FileOperations::isFolderExistAndAccess(filepath))
    {
      HEEX_LOG(debug) << "Chunk start : " << query.timestamp;
      HEEX_LOG(debug) << "Chunk duration : " << query.recordIntervalEnd;
      recordingChunk.filepath            = boost::filesystem::path(HeexUtils::FileOperations::getCorrectlyEncodedPath(filepath));
      recordingChunk.recordIntervalStart = boost::posix_time::from_iso_extended_string(query.timestamp);

      recordingChunk.valid             = true;
      const int milliseconds           = static_cast<int>(atof(query.recordIntervalEnd.c_str()) * 1000);
      recordingChunk.recordIntervalEnd = boost::posix_time::from_iso_extended_string(query.timestamp) + boost::posix_time::milliseconds(milliseconds);

      RecorderEventRecordingChunk recordingChunkTmp;

      // insert into recording buffer
      {
        const std::lock_guard<std::mutex> lock(_recordingBufferMutex);

        if (_recordingBuffer.size() < CHUNK_BUFFER_SIZE)
        {
          _recordingBuffer.push_back(recordingChunk);
        }
        else
        {
          recordingChunkTmp = _recordingBuffer.front();
          // remove first element as it is the oldest
          _recordingBuffer.erase(begin(_recordingBuffer));
          _recordingBuffer.push_back(recordingChunk);
        }
        HEEX_LOG(debug) << "RecorderInThePast::sendRecordingChunkToBuffer | " << filepath << " is inserted into the buffer";
      }
      if (_isPerformingGetFilePathFromBuffer == true)
      {
        _newChunkCv.notify_one();
      }

      // insert into garbage buffer
      {
        const std::lock_guard<std::mutex> lock(_garbageBufferMutex);
        if (recordingChunkTmp.valid == true)
        {
          _chunkGarbage.push_back(recordingChunkTmp);
        }
      }
    }
    else
    {
      HEEX_LOG(debug) << "RecorderInThePast::sendRecordingChunkToBuffer | " << filepath << " is not accessible";
    }
  }
  catch (const std::exception& e)
  {
    HEEX_LOG(error) << "RecorderInThePast::sendRecordingChunkToBuffer | failed to convert " << filepath << " into boost path. Event uuid : " << query.eventUuid << "caused by "
                    << e.what();
    return;
  }
}

void RecorderInThePast::onEventRecordingPartRequestCallback(const std::string& msg)
{
  Heex::RecorderArgs::RecorderEventRecordingPartArgs query = Heex::RecorderTools::parseRecorderDataFilePathMsg(msg, Heex::RecorderArgs::RecorderCmdType::QueryEventRecordingPart);
  // Parsing and decoding
  if (query.valid == false)
  {
    HEEX_LOG(error) << this->getUuid() << "::onEventRecordingPartRequestCallback bad msg formating." << std::endl;
    return;
  }

  std::string filepath;
  bool res = false;

  // Add a new answer copying the initial EventRecordingPart query to answers preparation queue
  this->addNewEventRecordingPartAnswer(query);

  if (_isRecordingInThePast == true)
  {
    res = getFilePathFromBuffer(query, filepath);
  }
  else
  {
    res = this->generateRequestedFilePaths(query, filepath);
  }

  if (res == false)
  {
    // Pop answer from queue on failure: Prevent leaving without removing it as it can comprmise the isPerformingCallback() call.
    const Heex::RecorderArgs::RecorderEventRecordingPartArgs answer = this->popEventRecordingPartAnswer(query, true);
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
    return;
  }

  Recorder::sendEventRecordingPartAnswer(answer, filepath);
}

bool RecorderInThePast::getFilePathFromBuffer(Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath)
{
  std::unique_lock<std::mutex> lk(_recordingBufferMutex, std::defer_lock);
  bool foundChunk    = false;
  bool isLinkReady   = false;
  bool isWakenUpByCv = false;

  HEEX_LOG(debug) << " query timestamp : " << query.timestamp;
  while (foundChunk == false)
  {
    // if waken up by CV, mutex is already owned,
    if (isWakenUpByCv == false)
    {
      lk.lock();
    }
    if (!_recordingBuffer.empty())
    {
      int indexOfChunk           = -1;
      const ExtractChunksRet ret = this->extractChunks(query, indexOfChunk);
      if (ret == ExtractChunksRet::EChunkFound)
      {
        foundChunk = true;
        if (this->prepareFilePath(filepath, query.eventUuid, indexOfChunk) == true)
        {
          if (this->setQueryRealRecordIntervalRange(query, _recordingBuffer[indexOfChunk]) == true)
          {
            _recordingBuffer[indexOfChunk].eventsUuid.push_back(query.eventUuid);
            isLinkReady = true;
          }
        }
      }
      else if (ret == ExtractChunksRet::EChunkCoversStartInterval)
      {
        HEEX_LOG(info) << "RecorderInThePast::getFilePathFromBuffer | No chunk in buffer that covers the end interval.";
      }
      else
      {
        HEEX_LOG(info) << "RecorderInThePast::getFilePathFromBuffer | No chunk in buffer that covers the start interval.";
        lk.unlock();
        return false;
      }
    }
    if (foundChunk == false)
    {
      _isPerformingGetFilePathFromBuffer = true;
      _newChunkCv.wait_for(lk, std::chrono::milliseconds(_lenRecordingChunk.total_milliseconds()));
      isWakenUpByCv = true;
    }
  }
  lk.unlock();
  _isPerformingGetFilePathFromBuffer = false;
  return isLinkReady;
}

bool RecorderInThePast::prepareFilePath(std::string& filepath, std::string& eventUuid, int& indexOfChunk)
{
  // create a symlink
  try
  {
    boost::system::error_code ec;
    const bool isFolder = boost::filesystem::is_directory(_recordingBuffer[indexOfChunk].filepath, ec) == true ? true : false;
    if (ec.value() != boost::system::errc::success)
    {
      HEEX_LOG(error) << "RecorderInThePast::prepareFilePath | Can't check if " + HeexUtils::FileOperations::getUtf8EncodedPath(_recordingBuffer[indexOfChunk].filepath) +
                             " is a folder, caused by " + ec.message()
                      << std::endl;
      return false;
    }

    std::string newFilepathStr;
    if (_recordingBuffer[indexOfChunk].filepath.has_stem())
    {
      newFilepathStr = HeexUtils::FileOperations::getUtf8EncodedPath(_recordingBuffer[indexOfChunk].filepath.stem()) + "-" + eventUuid;
    }
    else
    {
      HEEX_LOG(debug) << "RecorderInThePast::prepareFilePath | filepath has no stem : " << filepath;
      return false;
    }

    const std::string copyFilePath = _chunksLinksLocation + "/" + newFilepathStr;
    const boost::filesystem::path newFilepath(HeexUtils::FileOperations::getCorrectlyEncodedPath(copyFilePath));
    if (isFolder == true)
    {
      boost::filesystem::create_directory_symlink(_recordingBuffer[indexOfChunk].filepath, newFilepath, ec);
    }
    else
    {
      boost::filesystem::create_symlink(_recordingBuffer[indexOfChunk].filepath, newFilepath, ec);
    }

    if (ec.value() != boost::system::errc::success)
    {
      HEEX_LOG(error) << "RecorderInThePast::prepareFilePath | Can't create symlink at " + HeexUtils::FileOperations::getUtf8EncodedPath(newFilepath) +
                             ". Filesystem error caused by: " + ec.message()
                      << std::endl;
      return false;
    }

    ec.clear();
    filepath = HeexUtils::FileOperations::getUtf8EncodedPath(newFilepath);
  }
  catch (const std::exception& e)
  {
    HEEX_LOG(error) << "RecorderInThePast::prepareFilePath | has raised an exception due to : " << e.what();
    return false;
  }

  return true;
}

bool RecorderInThePast::generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& /*filepath*/)
{
  HEEX_LOG(warning) << this->getUuid()
                    << "| RecorderInThePast::generateRequestedFilePaths | This happened during the split time where we "
                       "are waiting for threads to join. "
                    << query.eventUuid << " at time " << query.timestamp << ". Will be discarded." << std::endl;
  // Don't use this class directly to implement you own recorder.
  // You need to inherate from the Recorder class and override this function.
  // See the getStartedRecorder examples and the Recorder documentation in the README.md file.
  return false;
}

ExtractChunksRet RecorderInThePast::extractChunks(Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, int& indexOfChunk)
{
  int milliseconds = static_cast<int>(atof(query.recordIntervalStart.c_str()) * 1000);

  const boost::posix_time::ptime queryStart = boost::posix_time::from_iso_extended_string(query.timestamp) + boost::posix_time::milliseconds(milliseconds);

  milliseconds = static_cast<int>(atof(query.recordIntervalEnd.c_str()) * 1000);

  const boost::posix_time::ptime queryEnd = boost::posix_time::from_iso_extended_string(query.timestamp) + boost::posix_time::milliseconds(milliseconds);

  ExtractChunksRet ret = ExtractChunksRet::ENoChunk;

  for (std::vector<RecorderEventRecordingChunk>::iterator it = begin(_recordingBuffer); it != end(_recordingBuffer); ++it)
  {
    if (queryStart >= it->recordIntervalStart)
    {
      ret = ExtractChunksRet::EChunkCoversStartInterval;
    }

    if (queryStart >= it->recordIntervalStart && queryEnd <= it->recordIntervalEnd)
    {
      ret          = ExtractChunksRet::EChunkFound;
      indexOfChunk = static_cast<int>(std::distance(begin(_recordingBuffer), it));
      break;
    }
  }
  return ret;
}

bool RecorderInThePast::canRemoveChunk(RecorderEventRecordingChunk& chunk)
{
  std::string symlinkPath;
  const boost::filesystem::path itPath(chunk.filepath);

  bool canDelete = true;

  for (const std::string& e : chunk.eventsUuid)
  {
    if (chunk.filepath.has_stem())
    {
      symlinkPath = HeexUtils::FileOperations::getUtf8EncodedPath(chunk.filepath.stem()) + "-" + e;
      symlinkPath = _chunksLinksLocation + "/" + symlinkPath;
    }
    else
    {
      HEEX_LOG(debug) << "RecorderInThePast::canRemoveChunk | filepath has no stem : " << chunk.filepath;
    }

    if (boost::filesystem::exists(boost::filesystem::path(HeexUtils::FileOperations::getCorrectlyEncodedPath(symlinkPath))) == true)
    {
      canDelete = false;
      break;
    }
  }
  return canDelete;
}

bool RecorderInThePast::setQueryRealRecordIntervalRange(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, const RecorderEventRecordingChunk& chunk)
{
  std::string realRecordIntervalStart;
  std::string realRecordIntervalEnd;

  const boost::posix_time::ptime queryTime = boost::posix_time::from_iso_extended_string(query.timestamp);
  if (queryTime < chunk.recordIntervalStart || queryTime > chunk.recordIntervalEnd)
  {
    HEEX_LOG(debug) << "RecorderInThePast::setQueryRealRecordIntervalRange | Chunk provided does not cover the range of the query";
  }

  double recordingDuration = static_cast<double>((queryTime - chunk.recordIntervalStart).total_milliseconds()) / 1000;
  // the precision can be modified, use 2 as we expect recording range to have at most 2 decimals
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << "-" << recordingDuration;
  realRecordIntervalStart = ss.str();

  // reset stream
  ss.str(std::string());
  recordingDuration = 0;

  recordingDuration = static_cast<double>((chunk.recordIntervalEnd - queryTime).total_milliseconds()) / 1000;
  ss << std::fixed << std::setprecision(2) << recordingDuration;
  realRecordIntervalEnd = ss.str();

  return this->setRealRecordIntervalRange(query, realRecordIntervalStart, realRecordIntervalEnd);
}

void RecorderInThePast::createWorkingDirectory()
{
  if (_sdeWorkingDirectory.empty())
  {
    HEEX_LOG(debug) << "RecorderInThePast::createWorkingDirectory | SDE working directory is not set";
    return;
  }

  if (!_chunksLinksLocation.empty())
  {
    // directories were already created
    return;
  }

  boost::system::error_code ec;
  boost::filesystem::create_directory(_sdeWorkingDirectory, ec);
  if (ec.value() != boost::system::errc::success && ec.value() != boost::system::errc::file_exists)
  {
    HEEX_LOG(error) << "RecorderInThePast | Can't create folder at " + _sdeWorkingDirectory + ". Filesystem error caused by: " + ec.message();
    return;
  }
  ec.clear();
  std::string chunkFolder                               = "RecorderChunkCache";
  const boost::filesystem::path sdeWorkingDirectoryPath = _sdeWorkingDirectory;
  const boost::filesystem::path chunkFolderPath         = sdeWorkingDirectoryPath / chunkFolder;

  HEEX_LOG(debug) << "RecorderInThePast | Creating operating directory at  : " << HeexUtils::FileOperations::getUtf8EncodedPath(chunkFolderPath) << std::endl;
  boost::filesystem::create_directory(chunkFolderPath, ec);
  if (ec.value() != boost::system::errc::success && ec.value() != boost::system::errc::file_exists)
  {
    HEEX_LOG(error) << "RecorderInThePast | Can't create folder at " + HeexUtils::FileOperations::getUtf8EncodedPath(chunkFolderPath.parent_path()) +
                           ". Filesystem error caused by: " + ec.message();
    return;
  }
  ec.clear();

  const boost::filesystem::path recorderWorkingDirectory = chunkFolderPath / this->getUuid();
  HEEX_LOG(debug) << "RecorderInThePast | Creating recorder operating directory at  : " << HeexUtils::FileOperations::getUtf8EncodedPath(recorderWorkingDirectory) << std::endl;
  boost::filesystem::create_directory(recorderWorkingDirectory, ec);
  if (ec.value() != boost::system::errc::success && ec.value() != boost::system::errc::file_exists)
  {
    HEEX_LOG(error) << "RecorderInThePast | Can't create folder at " + HeexUtils::FileOperations::getUtf8EncodedPath(recorderWorkingDirectory) +
                           ". Filesystem error caused by: " + ec.message();
    return;
  }
  ec.clear();
  try
  {
    _chunksLinksLocation = HeexUtils::FileOperations::getUtf8EncodedPath(boost::filesystem::canonical(HeexUtils::FileOperations::getUtf8EncodedPath(recorderWorkingDirectory)));
  }
  catch (const std::exception& e)
  {
    HEEX_LOG(error) << "RecorderInThePast : transform path to canonical has failed due to : " << e.what();
    _chunksLinksLocation = "";
  }
}

void RecorderInThePast::cleanUp()
{
  boost::system::error_code ec;
  if (boost::filesystem::remove_all(_chunksLinksLocation, ec) == false)
  {
    HEEX_LOG(info) << "RecorderInThePast::cleanUp | Can't delete" + _chunksLinksLocation + ". Filesystem error caused by: " + ec.message();
  }
}

bool RecorderInThePast::isChunkBufferEmpty()
{
  const std::lock_guard<std::mutex> lock(_recordingBufferMutex);
  return _recordingBuffer.empty();
}
