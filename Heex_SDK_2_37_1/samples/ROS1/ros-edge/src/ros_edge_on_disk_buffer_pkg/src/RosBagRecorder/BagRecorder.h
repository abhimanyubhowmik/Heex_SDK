/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @brief
/// Class that records messages of any type to a bag file.
////
#pragma once

#include <ros/ros.h>
#include <ros/time.h>
#include <topic_tools/shape_shifter.h>

#include <mutex>
#include <unordered_set>

#include "rosbag/bag.h"

namespace Heex
{
/// @brief store a message for a topic
/// with the received time and connection header
struct BagMessage
{
  BagMessage(const std::string& topic, topic_tools::ShapeShifter::ConstPtr msg, boost::shared_ptr<ros::M_string> connectionHeader, ros::Time _time)
      : topic(topic),
        message(msg),
        connectionHeader(connectionHeader),
        time(_time)
  {
  }

  std::string topic;
  topic_tools::ShapeShifter::ConstPtr message;
  boost::shared_ptr<ros::M_string> connectionHeader;
  ros::Time time;
};

/// @brief BagRecorder class.
/// BagRecorder offers an API to record messages from either listed topics or all
/// topics seen by roscore to a bag file. messages from callback are stored in a
/// queue for a duration specified by the record function. The data are then
/// written to a bag file and return to the client.
class BagRecorder
{
public:
  BagRecorder();
  ~BagRecorder();

  /// blocking call returns with the data
  ///
  /// @brief start recording messages received from callback. Messages are
  /// written into a bag file. This is a blocking function and returns when the
  /// bag file is closed.
  /// @param recordIntervalEnd
  /// @param topics
  /// @param bagPath the absolute path to the bag file
  /// @param recordAllTopics
  /// @return true
  /// @return false
  /// start recording messages received from callback. Messages are written into
  /// a bag file.
  bool record(std::string recordIntervalEnd, const std::string& queryTimestamp, std::string& bagPath, std::vector<std::string> topics = {});

  void stopRecording();

  // Subscribe queue size for each topic
  static const int QUEUE_SIZE;

private:
  /// Add a topic to the list of topics to be recorded
  void addTopic(const std::string& topic);
  /// subscribe to all topics seen by roscore
  void subcribeToAllTopics();
  /// subscribe to a single topic
  void subscribeToTopic(const std::string& topic);
  /// callback for all topics. the function acquires a lock to update the queue
  void subcriberCallback(
      const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event,
      std::string const& topic,
      boost::shared_ptr<ros::Subscriber> subscriber,
      boost::shared_ptr<int> count);

  /// check that there is enough space on disk to create the bag file
  bool isDiskSpaceEnough(const boost::filesystem::path& bagPath);

  std::string _bagFolder;
  std::vector<boost::shared_ptr<ros::Subscriber>> _subscribers;
  std::unordered_set<std::string> _subscribedTopics;
  std::mutex _bagMessageQueueMutex;

  // Bag data
  std::queue<Heex::BagMessage> _bagMessagesQueue;
  ros::NodeHandle _nh;
};
} // namespace Heex
