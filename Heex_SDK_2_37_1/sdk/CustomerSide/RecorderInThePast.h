///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file RecorderInThePast.h
///
/// @brief Library header file that contains the basic structure for any Recorders to appropriately register and
/// communicate with the Smart Data Engine. Works for both Real-time and Datalake applications.
///
/// @author Henry Tanoh
///
/// @date 2022-12-07
///
#pragma once

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <condition_variable>
#include <queue>
#include <thread>

#include "HeexUtilsFileOperations.h"
#include "Recorder.h"

/// @brief structure to store information about recording chunk for recorder in the past
struct RecorderEventRecordingChunk
{
  bool valid{false};
  boost::posix_time::ptime recordIntervalStart;
  boost::posix_time::ptime recordIntervalEnd;
  boost::filesystem::path filepath;
  std::vector<std::string> eventsUuid; ///<List of events uuid that uses the chunk

  RecorderEventRecordingChunk() = default;

  friend std::ostream& operator<<(std::ostream& os, const RecorderEventRecordingChunk& chunk)
  {
    if (chunk.valid == false)
    {
      os << "Chunk is invalid";
      return os;
    }

    os << "[Event Uuid : ";
    for (const std::string& uuid : chunk.eventsUuid)
    {
      os << uuid << "-";
    }
    os << "] - ";
    os << "[recordIntervalStart : " << boost::posix_time::to_iso_extended_string(chunk.recordIntervalStart);
    os << "] - ";
    os << "[recordIntervalEnd : " << boost::posix_time::to_iso_extended_string(chunk.recordIntervalEnd);
    os << "] - ";
    os << "[filepath : " << HeexUtils::FileOperations::getUtf8EncodedPath(chunk.filepath) << "]";

    os << std::endl;

    return os;
  }
}; // RecorderEventRecordingChunk

/// @brief structure to store information about recording chunk job
struct RecordingChunkJob
{
  boost::posix_time::ptime recordingStart;
}; // RecordingChunkJob

/// @brief Return values for extractChunks
enum class ExtractChunksRet
{
  EChunkCoversStartInterval,
  EChunkFound,
  ENoChunk,
};

class RecorderInThePast : public Recorder
{
public:
  /// @brief Constructor for Recorder with its full configuration. It requires the uuid and the networking settings for initialization.
  ///
  /// @param uuid Unique identifier for the Recorder. Shall be a string starting with 'R-' and finishing with a version-4 UUID.
  /// @param serverIp Ip of the Heex Core to which the Recorder will connect, identify and notify
  /// @param serverPort Port of the Heex Core dedicated to the Writer Wrapper (WW) module
  /// @param isRecordingInThePastSupported Does the system support recording in the past ? . Set by the client during creation of recorder
  /// @param implementationVersion Implementation version of the recorder. Transmitted to the SDE during agent identification process.
  RecorderInThePast(
      const std::string& uuid,
      const std::string& serverIp,
      const unsigned int& serverPort,
      const bool& isRecordingInThePastSupported = false,
      const std::string& implementationVersion  = Heex::HEEX_DEFAULT_AGENT_VERSION);

  /// @brief Deconstructor for Recorder to free memory appropriately. In this case, it destroys the TcpClient pointer.
  virtual ~RecorderInThePast();

protected:
  virtual void onEventRecordingPartRequestCallback(const std::string& msg) override;
  virtual bool generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath) override;

  ///
  /// @brief Thread worker routine. Take a job in queue, put the next one and call client overloaded function to record chunk
  virtual void doRecordingChunksJobs();

  ///
  /// @brief Garbage Collector routine. For each chunk, delete if there is no event that is using it(symlink is present)
  virtual void deleteChunks();

  ///
  /// @brief returns the chunk that covers the range of the query
  ///
  /// @param query RecorderEventRecordingPartArgs Query to build answer on.
  /// @param filepath Event recording filepath
  virtual bool getFilePathFromBuffer(Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath);

  ///
  /// @brief create a symlink for the query
  /// @param filepath symlink filepath
  /// @param eventUuid
  /// @param indexOfChunk Index of relevant chunk for event
  virtual bool prepareFilePath(std::string& filepath, std::string& eventUuid, int& indexOfChunk);

  /// @brief Find the correct recording range for the query

  /// @param query Original Query
  /// @param chunk Chunk that covers the query range
  bool setQueryRealRecordIntervalRange(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, const RecorderEventRecordingChunk& chunk);

  ///
  /// @brief Insert chunk into buffer. remove the last one if size is reach. Last one is put in garbage buffer
  /// @param query Simulated Chunk query
  /// @param filepath Event recording filepath
  void sendRecordingChunkToBuffer(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath);

  ///
  /// @brief using the min and max recording ranges for the recorder, decide if recording in the past or not. If yes, launch chunks thread. If not, stop them
  void configureWorkers();

  virtual void onIdentificationAccepted(const std::string&) override;
  /// @brief SDE specify that all agent configurations values needed
  virtual void onConfigurationChanged(const std::string&) override;

  /// Check if recording buffer is empty
  bool isChunkBufferEmpty();
  /// Delete working folder of the recorder.
  void cleanUp();

  static const int NB_WORKER;
  static const int CHUNK_BUFFER_SIZE;
  /// Use this rate to record a bit before and after the expected time to take into accout drifts that can happens during the recording or when the threads are locked
  static const double LEN_RECORDING_CHUNK_ADJUSTEMENT;
  std::atomic<bool> _isRecordingInThePast{};

private:
  ///
  /// @brief Find the chunk that covers the query range
  /// @param indexOfChunk  index of chunk
  /// @return @eChunkFound if chunk found, @eChunkCoversStartInterval if there is chunk that covers the start, @eNoChunk else
  ExtractChunksRet extractChunks(Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, int& indexOfChunk);

  /// Create working directory for recorder to store simlinks
  void createWorkingDirectory();
  /// Check if there symlink with target chunk filepath, if no, can delete chunk filepath
  bool canRemoveChunk(RecorderEventRecordingChunk& chunk);

  /// Send signal to stop Chunk threads and wait for them to finish
  void stopChunkGeneratorsWorkers();

  /// Wait to delete all chunks created and join garbage thread
  void stopGarbageCollectorWorker();

  boost::posix_time::time_duration _lenRecordingChunk; ///< length to record ahead for event

  std::vector<std::thread> _chunkThreads;
  std::thread _garbageCollectorThread;

  /// Buffer to store the recording chunks that we consider relevant, not too old
  std::vector<RecorderEventRecordingChunk> _recordingBuffer;

  /// Buffer to store the recording chunks that we consider old, but might be in use by data collector,
  std::vector<RecorderEventRecordingChunk> _chunkGarbage;

  std::queue<RecordingChunkJob> _recordingChunkJobQueue;

  bool _isRecordingInThePastSupported;     ///<Does the system support recording in the past by default
  std::mutex _recordingChunkJobQueueMutex; ///<mutex for recording chunk jobs
  std::mutex _recordingBufferMutex;        ///<mutex for chunks buffer
  std::mutex _garbageBufferMutex;          ///<mutex for garbage chunks
  std::atomic<bool> _isWorkerAllowed{};
  std::atomic<bool> _isGarbageCollectorWorkerAllowed{};
  std::atomic<bool> _isPerformingGetFilePathFromBuffer{};
  std::condition_variable _newChunkCv; /// Condition variable that signals that a new chunk was added to the buffer

  std::string _chunksLinksLocation; ///<path where simlinks will be created
  std::string _sdeWorkingDirectory;
  unsigned int _garbageWaitDelay; ///<Delay between retry in seconds.

  //friend class TestRecorderInThePast;
};
