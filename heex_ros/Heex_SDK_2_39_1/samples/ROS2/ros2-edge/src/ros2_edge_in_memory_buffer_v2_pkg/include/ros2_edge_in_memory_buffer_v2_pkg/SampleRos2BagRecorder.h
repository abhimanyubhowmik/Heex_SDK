/// Copyright (c) 2024 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#pragma once

#include "BagRecorder.h"
#include "Recorder.h"

class SampleRos2BagRecorder : public Recorder
{
public:
  /// @brief Construct a new Rosbag Recorder object. Inherits from Recorder requirements for uuid, serverIp and serverPort.
  ///
  /// @param uuid
  /// @param serverIp
  /// @param serverPort
  SampleRos2BagRecorder(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort);

  ///@brief Destroy the SampleRos2BagRecorder object
  ///
  ~SampleRos2BagRecorder() = default;

  /// @brief Override the awaitReady to make the node responsive to ros status as well as its connection to the Kernel
  ///
  virtual void awaitReady() override;

  /// @brief returns the BagRecorder node
  ///
  /// @return _node - std::shared_ptr<Heex::BagRecorder>
  std::shared_ptr<Heex::BagRecorder> getBagRecorderNode() { return _bagRecorderNode; };

protected:
  /// @brief Returns the value as Heex::RecorderArgs::ContextValue for the
  /// advertized context by the Recorder (e.g. position). This method is virtual
  /// and by default return an empty string standing for error. It requires to be
  /// defined within any subclasses to the address the logic of getting data
  /// value advertized by your Recorder. Any value with an empty string will be
  /// recognized as an error.
  ///
  /// @param query Current RecorderContextValueArgs-structured query that holds
  /// the arguments of the request of ContextValue extraction.
  /// @param contextValues The pass-by-value contextValues variable that contains
  /// the key and values of ContextValues advertized by the Recorder. Edit it
  /// using the addContextValue(contextValues, "key", "value"). E.g. A context
  /// value named "position" would return the value "48.8582651,2.2938142".
  ///
  /// @return true Notify that the generation has completed with success.
  /// @return false Notify that the generation has failed and ignore the result.
  virtual bool generateRequestedValues(const Heex::RecorderArgs::RecorderContextValueArgs& query, std::vector<Heex::RecorderArgs::ContextValue>& contextValues);

  /// @brief Returns the value as std::string of the filepath pointing to the
  /// extracted data file or folder. This method is virtual and requires to be
  /// defined within any subclasses to the logic of getting data filepath
  /// advertized by your Recorder. Any empty string will be recognized as an
  /// error.
  ///
  /// @param query Current RecorderContextValueArgs-typed query that holds the
  /// arguments of the request of ContextValuelue extraction.
  /// @param filepath The pass-by-value filepath variable shall be set to the
  /// event recording part filepath. It shall correspond to the actual recorded
  /// data on the machine. In our case, it will be the folderpath to the extracted ros bag
  /// E.g. A filepath would return the value "/tmp/my_folder". We encourage to limit the use of
  /// special character like spaces within the filepath. Special characters ";" and ":" are stricly
  /// forbidden.
  ///
  /// @return true Notify that the generation has completed with success.
  /// @return false Notify that the generation has failed and ignore the result.
  virtual bool generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath) override;

  ///@brief This function is called within the Recorder's class at every configuration change (also the initial one).
  ///       This is a virtual function that overwrites the original one (which has no behavior, is only called). In our
  ///       case we use this function to fetch from the recorder's configuration the lowest StartInterval and highest
  ///       EndInterval, so that we can adapt the buffer size accordingly and avoid buffering unecessary data.
  ///       The buffer is removing any message that is older then (MaxEnterval - MinInterval) + 5 seconds for the recorder
  ///       And for our current context value example, we set the buffer to abs(MinInterval) + 5
  ///
  virtual void onConfigurationChangedCallback() override;

private:
  std::vector<Heex::TopicDetails_t> _topicDetails; ///< list of topic details we want the bagRecorder to subscribe to (can be left empty to subscribe to all topics)

  std::string _positionTopicName; ///< topic name for the GPS position. This is for our sample specific demo that shall extract the position context values.

  std::shared_ptr<Heex::BagRecorder> _bagRecorderNode; ///< pointer to the BagRecorder Node
};
