/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#include "Rosbag2RecorderSnapshotter.h"

#include <boost/thread.hpp>
#include <chrono>
#include <iostream>
#include <thread>

#include "HeexUtilsLog.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// CONSTANTS
static const std::string RECORDER_NODE_NAME = "ros2_edge_in_memory_buffer_recorderNode";

Rosbag2RecorderSnapshotter::Rosbag2RecorderSnapshotter(const std::string& serverIp, const unsigned int& serverPort)
    : Recorder("R-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)", serverIp, serverPort)
{
  // Set the different topics to record.
  /// Hard-coded topic names available in the example bag.
  /// Optionally, you can specify a QoS profile for each topic. If none specified, a default 'QoS{10}' shall be used as defined in TopicDetails constructor.
  rclcpp::QoS specific_qos_profile{100}; // this profile is compatible with the default one, but is here to show how to use specific profiles
  specific_qos_profile.keep_all();
  _topics = {
      heex_rosbag2_snapshot::TopicDetails("/gps/fix", "sensor_msgs/msg/NavSatFix"),
      heex_rosbag2_snapshot::TopicDetails("/demo/bool", "std_msgs/Bool", specific_qos_profile),
      heex_rosbag2_snapshot::TopicDetails("/imu", "sensor_msgs/msg/Imu")};

  _topicNameGps = "/gps/fix"; // Hard-code of the topic name containing GPS data (expecting msg of type sensor_msgs/NavSatFix.h)

  /// Hard-coded topic names whose latest message will be written at start of the extracted bag.
  _topics_once_at_bagstart = {heex_rosbag2_snapshot::TopicDetails("/tf_static", "tf2_msgs/msg/TFMessage")};

  // Configure Snapshotter with the topics to snapshot
  heex_rosbag2_snapshot::SnapshotterOptions options(rclcpp::Duration(15, 0));
  /// 1 : Select the list of topics to snapshot over the period of time.
  /// Choose one:
  /// 1.1 Set the _all_topics to `false` if you want to record only topics with in _topics list.
  /// 1.2 Set the _all_topics to `true` if you want to record all available topics. Topics in _topics won't be considered.
  /// NOTE: when using _all_topics = true, the recorder shall automatically set the QoS profile of each signal it subscribes to,
  /// NOTE: so that it takes the same profile as the published topic's QoS.
  _all_topics = false;
  if (!_all_topics)
  {
    // Configure and add all topics in _topics list with the default limit topic option
    options.all_topics_ = _all_topics;
    heex_rosbag2_snapshot::SnapshotterTopicOptions topicsOptions;
    for (auto topic : _topics)
    {
      options.topics_[topic] = topicsOptions;
    }
  }
  else
  {
    // Since we use the _all_topics and subscribe to all available topics, _topics is not used so we can clear it
    options.all_topics_ = _all_topics;
    _topics.clear();
  }
  /// 2 : Assign the list of topics to snapshot once and write only once at the start of any extracted bag.
  options.topics_once_at_bagstart_ = _topics_once_at_bagstart;

  rclcpp::NodeOptions node_options;
  _snap = std::make_shared<heex_rosbag2_snapshot::Snapshotter>(node_options, options, RECORDER_NODE_NAME);
}

Rosbag2RecorderSnapshotter::~Rosbag2RecorderSnapshotter() {}

bool Rosbag2RecorderSnapshotter::generateRequestedValues(const Heex::RecorderArgs::RecorderContextValueArgs& query, std::vector<Heex::RecorderArgs::ContextValue>& contextValues)
{
  // Prepare SnapshotRequest content
  rclcpp::Time event_time = ros2_sample_utils::iso_extended_str_to_rclcpptime(query.timestamp);
  rclcpp::Time query_time = event_time;
  rclcpp::Time msg_time;
  rclcpp::Duration search_offset(-2, 0); // This parameter shall vary according to the frequency of message publication

  auto req = std::make_shared<heex_rosbag2_snapshot::SnapshotInstantRequest>(
      rclcpp::SerializedMessage(),                                        // msg
      query_time,                                                         // query_time
      msg_time,                                                           // msg_time
      search_offset,                                                      // search_offset
      ros2_sample_utils::getSnapshottersTopicByName(_snap, _topicNameGps) // topic
  );

  auto res = std::make_shared<heex_rosbag2_snapshot::SnapshotResponse>(
      false,        // success
      std::string() // message
  );
  // Call to Snapshotter with the request element to extract smart data from the cycling buffer
  _snap->InstantMessageCb(nullptr, req, res); // Return when all past data have been written
  if (res->success)
  {
    std::shared_ptr<sensor_msgs::msg::NavSatFix> posMsgPtr = std::make_shared<sensor_msgs::msg::NavSatFix>();
    rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serializer;
    // Deserialize the message from the serialized message
    serializer.deserialize_message(&req->msg, posMsgPtr.get());
    if (posMsgPtr != nullptr)
    {
      std::vector<std::string> keys = this->getContextValueKeys(query);
      HEEX_LOG(info) << "SampleRecorder | Number of requested ContextValues for the SampleRecorder: " << keys.size();

      for (const std::string& key : keys)
      {
        if (key == "position")
        {
          HEEX_LOG(info) << "Position extracted from RosBag: " << posMsgPtr->latitude << ", " << posMsgPtr->longitude;
          // Add the value to the specified "position" key
          bool success = this->addGNSSContextValue(contextValues, key, posMsgPtr->latitude, posMsgPtr->longitude);
          if (success == false)
          {
            return false;
          }
        }
        if (key == "bag_timestamp")
        {
          // Add the value to the specified "bag_timestamp" key
          std::string ts = ros2_sample_utils::rclcpptime_to_iso_extended_str(req->msg_time);
          HEEX_LOG(info) << "Timestamp extracted from RosBag: " << ts.c_str();
          bool success = this->addContextValue(contextValues, key, ts);
          if (success == false)
          {
            return false;
          }
        }
      }

      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }

  return false;
}

bool Rosbag2RecorderSnapshotter::generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath)
{
  // Pre-compute variables
  rclcpp::Duration ts(std::atof(query.recordIntervalStart.c_str()), 0);
  rclcpp::Duration te(std::atof(query.recordIntervalEnd.c_str()), 0);

  // Prepare SnapshotRequest content
  rclcpp::Time event_time = ros2_sample_utils::iso_extended_str_to_rclcpptime(query.timestamp);
  rclcpp::Time start_time(event_time + ts);
  rclcpp::Time stop_time(event_time + te);
  std::string foldername = "/tmp/recording_" + query.eventUuid + "_" + query.uuid;

  auto req = std::make_shared<heex_rosbag2_snapshot::SnapshotRequest>(
      start_time,                                                                             // start_time
      stop_time,                                                                              // stop_time
      foldername,                                                                             // filename
      this->_all_topics ? std::vector<heex_rosbag2_snapshot::TopicDetails>{} : this->_topics, // topics
      start_time                                                                              // last_write_time
  );

  auto res = std::make_shared<heex_rosbag2_snapshot::SnapshotResponse>(
      false,        // success
      std::string() // message
  );

  HEEX_LOG(info) << "RosbagRecorderSnapshotter | Requesting messages extraction for [" << std::fixed << std::setprecision(10) << req->start_time.seconds() << "; "
                 << req->stop_time.seconds() << "]";

  // Call to Snapshotter with the request element to extract smart data from the cycling buffer
  this->_snap->triggerSnapshotCb(nullptr, req, res); // Return when all past data have been written

  // Manage request result with Heex recorder
  if (res->success)
  {
    filepath = foldername;
    return true;
  }

  // Error state
  HEEX_LOG(error) << "Rosbag2RecorderSnapshotter::generateRequestedFilePaths | An error has occurred: " << res->message;
  return false;
}

void Rosbag2RecorderSnapshotter::awaitReady()
{
  HEEX_LOG(info) << "Waiting for kernel to be ready ...";
  while (this->isReady() == false && rclcpp::ok())
  {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  HEEX_LOG(info) << "Connected to kernel.";
}
