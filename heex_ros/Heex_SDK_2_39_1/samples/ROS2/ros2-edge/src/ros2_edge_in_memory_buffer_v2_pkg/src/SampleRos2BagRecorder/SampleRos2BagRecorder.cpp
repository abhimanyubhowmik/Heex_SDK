/// Copyright (c) 2024 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "SampleRos2BagRecorder.h"

#include <boost/bind/bind.hpp>

#include "HeexUtilsLog.h"
#include "RecorderTools.h"
#include "ros2SampleUtils.hpp"

// Make sure to add all the specific sensor_msg headers you shall be needing:
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// CONSTANTS
static constexpr const char* BAG_RECORDER_NODE_NAME = "ros2_edge_in_memory_buffer_v2_bagRecorderNode"; ///< name of the bagRecorder node
static constexpr int ADDITIONAL_BUFFER_TIME         = 5; ///< [seconds] fixed time added to the buffer size allowing processing time to finish

SampleRos2BagRecorder::SampleRos2BagRecorder(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort) : Recorder(uuid, serverIp, serverPort)
{
  /// Optionally, you can specify a QoS profile for each topic. If none specified, a default 'QoS{10}' shall be used
  rclcpp::QoS specific_qos_profile{100};
  specific_qos_profile.keep_all();

  /// Add all topics that need to be subscribed to in here. TopicDetails_t is filled with (topic_name, topic_type, [OPTIONAL: topic QoS])
  /// NOTE: if you set _topicDetails to be empty, we shall subscribe to all available topics that will be published
  _topicDetails = {
      Heex::TopicDetails_t("/gps/fix", "sensor_msgs/msg/NavSatFix"),
      Heex::TopicDetails_t("/demo/bool", "std_msgs/Bool", specific_qos_profile),
      Heex::TopicDetails_t("/imu", "sensor_msgs/msg/Imu")};

  /// options to set the rclcpp Node
  rclcpp::NodeOptions nodeOptions;
  _bagRecorderNode = std::make_shared<Heex::BagRecorder>(BAG_RECORDER_NODE_NAME, nodeOptions);
  _bagRecorderNode->subscribeToAllTopics(_topicDetails);

  _positionTopicName = "/gps/fix"; ///< make sure it's part of the _topicDetails (or leave _topicDetails empty so it subscribes to all topics)
}

///! Callback function that shall be called during each configuration change (including first initialization)
void SampleRos2BagRecorder::onConfigurationChangedCallback()
{
  /// fetch the recorder ranges
  const std::vector<std::string> dynamicConfigValues = this->retrieveDynamicConfigValues(Heex::HEEX_RECORDING_RANGE_KEY);
  double minrecordIntervalStart                      = 0.0;
  double maxrecordIntervalEnd                        = 0.0;

  /// retrieve the min start interval, and the max
  for (const std::string& value : dynamicConfigValues)
  {
    const Heex::RecorderArgs::RecorderRangesValues res = Heex::RecorderTools::parseRecorderRangesMsg(value);
    if (res.valid == true)
    {
      if (res.recordIntervalStart < minrecordIntervalStart)
      {
        minrecordIntervalStart = res.recordIntervalStart;
      }
      if (res.recordIntervalEnd > maxrecordIntervalEnd)
      {
        maxrecordIntervalEnd = res.recordIntervalEnd;
      }
    }
  }
  if ((maxrecordIntervalEnd - minrecordIntervalStart) > 0.0)
  {
    if (_bagRecorderNode != nullptr)
    {
      /// setting the bagRecorder's beffer to (max - min) + 5 seconds
      _bagRecorderNode->setOldestBufferMsgDate(static_cast<int>(maxrecordIntervalEnd - minrecordIntervalStart) + 5);
    }
  }
}

bool SampleRos2BagRecorder::generateRequestedValues(const Heex::RecorderArgs::RecorderContextValueArgs& query, std::vector<Heex::RecorderArgs::ContextValue>& contextValues)
{
  rclcpp::Time eventTime = ros2_sample_utils::iso_extended_str_to_rclcpptime(query.timestamp);
  rclcpp::SerializedMessage closestMsg;
  rclcpp::Time closestTime;
  bool ret = _bagRecorderNode->extractClosestMsgToEvent(_positionTopicName, eventTime, closestMsg, closestTime);
  if (ret)
  {
    std::shared_ptr<sensor_msgs::msg::NavSatFix> posMsgPtr = std::make_shared<sensor_msgs::msg::NavSatFix>();
    rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serializer;
    // Deserialize the message from the serialized message

    serializer.deserialize_message(&closestMsg, posMsgPtr.get());

    std::vector<std::string> keys = this->getContextValueKeys(query);
    HEEX_LOG(info) << "SampleRos2BagRecorder::generateRequestedValues | Number of requested ContextValues for the SampleRecorder: " << keys.size() << std::endl;

    for (std::string& key : keys)
    {
      HEEX_LOG(info) << "key : " << key << std::endl;
      if (key == "position")
      {
        // Add to the value to the specified "position" key
        HEEX_LOG(info) << "SampleRos2BagRecorder::generateRequestedValues | Position extracted from RosBag : " << posMsgPtr->latitude << ", " << posMsgPtr->longitude << std::endl;
        bool success = this->addGNSSContextValue(contextValues, key, posMsgPtr->latitude, posMsgPtr->longitude);
        if (!success)
        {
          HEEX_LOG(error) << "SampleRos2BagRecorder::generateRequestedValues | Failed to add addGNSSContextValue for position" << std::endl;
          return false;
        }
      }
      if (key == "bag_timestamp")
      {
        // Add to the value to the specified "bag_timestamp" key
        std::string ts = ros2_sample_utils::rclcpptime_to_iso_extended_str(closestTime);
        HEEX_LOG(info) << "SampleRos2BagRecorder::generateRequestedValues | Timestamp extracted from RosBag : " << ts << std::endl;
        bool success = this->addContextValue(contextValues, key, ts);
        if (!success)
        {
          HEEX_LOG(error) << "SampleRos2BagRecorder::generateRequestedValues | Failed to addContextValue for bag_timestamp" << std::endl;
          return false;
        }
      }
    }

    return true;
  }
  return false;
}

bool SampleRos2BagRecorder::generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath)
{
  std::vector<std::string> ignoredTopics = {}; // optionnally, some topic names can be added in this vector so that these specific topics are not recorded in the rosbag
  const bool ret = _bagRecorderNode->recordToBag(query.recordIntervalStart, query.recordIntervalEnd, query.timestamp, query.eventUuid, filepath, ignoredTopics);
  if (!ret)
  {
    HEEX_LOG(error) << "SampleRos2BagRecorder::generateRequestedFilePaths | Error while recording bag" << std::endl;
  }
  return ret;
}

void SampleRos2BagRecorder::awaitReady()
{
  HEEX_LOG(info) << "SampleRos2BagRecorder::awaitReady | Waiting to connect to kernel" << std::endl;
  while (!this->isReady() || !rclcpp::ok())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  HEEX_LOG(info) << "SampleRos2BagRecorder::awaitReady | Connected to kernel." << std::endl;
}
