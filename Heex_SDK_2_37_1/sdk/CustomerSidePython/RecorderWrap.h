///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include "Recorder.h"
#include "RecorderArgs.h"
#if defined(_POSIX_VERSION)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#elif defined(_WIN32) || defined(_WIN64)
  #pragma warning(push)
  #pragma warning(disable : 4267)
#endif
#include <boost/python.hpp>
#if defined(_POSIX_VERSION)
  #pragma GCC diagnostic pop
#elif defined(_WIN32) || defined(_WIN64)
  #pragma warning(pop)
#endif
#include "ValueConfiguration.h"

class RecorderWrap : public Recorder, public boost::python::wrapper<Recorder>
{
public:
  /// @brief Constructor for Recorder with its full configuration. It requires the uuid and the networking settings for initialization.
  ///
  /// @param uuid Unique identifier for the Recorder. Shall be a string starting with 'R-' and finishing with a version-4 UUID.
  /// @param serverIp Ip of the Heex Core to which the Recorder will connect, identify and notify
  /// @param serverPort Port of the Heex Core dedicated to the Writer Wrapper (WW) module
  /// @param implementationVersion Implementation version of the recorder. Transmitted to the SDE during agent identification process.
  RecorderWrap(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion = Heex::HEEX_DEFAULT_AGENT_VERSION);

  /// @brief Deconstructor for Recorder to free memory appropriately. In this case, it destroys the TcpClient pointer.
  ~RecorderWrap();
  virtual bool generateRequestedValues(const Heex::RecorderArgs::RecorderContextValueArgs& query, std::vector<Heex::RecorderArgs::ContextValue>& contextValues) override;
  virtual bool generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath) override;
  virtual ValueConfiguration* handleValueConfiguration(const std::string& cmd, ValueConfHeader& header) override;
  virtual void onConfigurationChangedCallback() override;
  virtual void awaitReady() override;

  //
  // Recorder wrapper method visibility modification (protected to public so that python can have access)
  //

  /// Expose publicly the getContextValueKeys methods of Recorder class but doesn't override it.
  std::vector<std::string> getContextValueKeys(const Heex::RecorderArgs::RecorderContextValueArgs& query);
  /// Expose publicly the addContextValue methods of Recorder class but doesn't override it.
  bool addContextValue(std::vector<Heex::RecorderArgs::ContextValue>& res, const std::string key, const std::string value);
  /// Expose publicly the addGNSSContextValue methods of Recorder class but doesn't override it.
  bool addGNSSContextValue(std::vector<Heex::RecorderArgs::ContextValue>& res, const std::string key, const double latitude, const double longitude);
  /// Expose publicly the setRealRecordIntervalRange methods of Recorder class but doesn't override it.
  bool setRealRecordIntervalRange(
      const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query,
      const std::string& realRecordIntervalStart,
      const std::string& realRecordIntervalEnd);
  /// Expose publicly the getEventRecordingPartAnswer methods of Recorder class but doesn't override it.
  const Heex::RecorderArgs::RecorderEventRecordingPartArgs getEventRecordingPartAnswer(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query);
};
