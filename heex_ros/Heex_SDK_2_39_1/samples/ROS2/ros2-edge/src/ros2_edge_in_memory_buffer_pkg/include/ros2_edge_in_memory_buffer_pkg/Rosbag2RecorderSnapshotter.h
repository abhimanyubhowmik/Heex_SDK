/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "Recorder.h"
#include "Snapshotter.hpp"
#include "ros2SampleUtils.h"

class Rosbag2RecorderSnapshotter : public Recorder
{
public:
  /// @brief Construct a new Rosbag Recorder Snapshotter object. Inherits from Recorder requirements for uuid, serverIp and serverPort.
  ///
  /// @param uuid ROS node handle pointer
  /// @param serverIp
  /// @param serverPort
  Rosbag2RecorderSnapshotter(const std::string& serverIp, const unsigned int& serverPort);

  /// @brief Destroy the Rosbag Recorder Snapshotter object
  ~Rosbag2RecorderSnapshotter();

  /// @brief Override the awaitReady to make the node responsive to ros status and controls while pending connection with the Core
  virtual void awaitReady() override;

  /// @brief returns the snapshotter node
  ///
  /// @return _snap - std::shared_ptr<heex_rosbag2_snapshot::Snapshotter>
  std::shared_ptr<heex_rosbag2_snapshot::Snapshotter> get_node() { return _snap; };

protected:
  //
  // Methods overriding the default Recorder operations for ContextValues and EventRecordingPart generation
  //

  /// @brief Returns the value as Heex::RecorderArgs::ContextValue for the advertized context by the Recorder (e.g. position). This method is virtual and by default return an empty string standing for error. It requires to be defined within any subclasses to the address the logic of getting data value advertized by your Recorder.
  /// Any value with an empty string will be recognized as an error.
  ///
  /// @param query Current RecorderContextValueArgs-structured query that holds the arguments of the request of ContextValuelue extraction.
  /// @param contextValues The pass-by-value contextValues variable that contains the key and values of ContextValues advertized by the Recorder. Edit it using the addContextValue(contextValues, "key", "value"). E.g. A context value named "position" would return the value "48.8582651,2.2938142".
  ///
  /// @return true Notify that the generation has completed with success.
  /// @return false Notify that the generation has failed and ignore the result.
  virtual bool generateRequestedValues(const Heex::RecorderArgs::RecorderContextValueArgs& query, std::vector<Heex::RecorderArgs::ContextValue>& contextValues);

  /// @brief Returns the value as std::string of the filepath pointing to the extracted data file or folder. This method is virtual and requires to be defined within any subclasses to the logic of getting data filepath advertized by your Recorder.
  /// Any empty string will be recognized as an error.
  ///
  /// @param query Current RecorderContextValueArgs-typed query that holds the arguments of the request of ContextValuelue extraction.
  /// @param filepath The pass-by-value filepath variable shall be set to the event recording part filepath. It shall correspond to the actual recorded data on the machine. E.g. A filepath would return the value "/tmp/my_file.txt". We encourage to limit the use of special character like spaces within the filepath. Special characters ";" and ":" are stricly forbidden.
  ///
  /// @return true Notify that the generation has completed with success.
  /// @return false Notify that the generation has failed and ignore the result.
  virtual bool generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath);

  /// @brief Run the Snapshotter service in a dedicated thread. It starts managing cycling buffer(s) for the _topics.
  void runCyclingBuffer();

private:
  //
  // Attributes
  //
  std::shared_ptr<heex_rosbag2_snapshot::Snapshotter> _snap;
  std::shared_ptr<boost::thread> _snapThread;
  /// Cache the list of topics to record over the time interval (duration).
  std::vector<heex_rosbag2_snapshot::TopicDetails> _topics;
  /// Cache the list of topics to record once (latest).
  std::vector<heex_rosbag2_snapshot::TopicDetails> _topics_once_at_bagstart;
  std::string _topicNameGps; // Cache of GNSS topic name
  bool _all_topics;          // Cache if you want to record all available topics. _topics will be ignored for rosbag extraction.
};
