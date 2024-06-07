/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#pragma once

#include <boost/date_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <rclcpp/rclcpp.hpp>

#include "HeexUtilsLog.h"
#include "Snapshotter.hpp"

namespace ros2_sample_utils
{
/// @brief Converts the time from a rclcpp time format to a string iso extended one
///
/// @param rclcpp_time time in rclcpp::Time format
///
/// @return time in iso extended format, std::string
inline std::string rclcpptime_to_iso_extended_str(const rclcpp::Time& rclcpp_time)
{
  // Convert ROS time (which is in nanoseconds since epoch) to a ptime object
  boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
  boost::posix_time::time_duration since_epoch(0, 0, rclcpp_time.seconds(), (rclcpp_time.nanoseconds() - rclcpp_time.seconds() * 1e9));

  // Adding microsecond precision
  since_epoch += boost::posix_time::microseconds((rclcpp_time.nanoseconds() / 1000) % 1000000);

  boost::posix_time::ptime ptime = epoch + since_epoch;

  // Convert ptime object to ISO extended string
  return boost::posix_time::to_iso_extended_string(ptime);
}

/// @brief Converts the time from a string iso extended format to a rclcpp time one
///
/// @param posix_iso_extended_str time in iso extended format, std::string
///
/// @return time in rclcpp::Time format
inline rclcpp::Time iso_extended_str_to_rclcpptime(const std::string& posix_iso_extended_str)
{
  boost::posix_time::ptime boost_ptime      = boost::posix_time::from_iso_extended_string(posix_iso_extended_str);
  // Calculate the duration from epoch
  boost::posix_time::time_duration duration = boost_ptime - boost::posix_time::ptime(boost::gregorian::date(1970, 1, 1));

  // Convert duration to rclcpp::Time
  return rclcpp::Time(duration.total_seconds(), duration.total_nanoseconds() - duration.total_seconds() * 1e9);
}

/// @brief Extracts from the snapshotter the topicDetail based on the given topicName
///
/// @param snapshotter - std::shared_ptr<heex_rosbag2_snapshot::Snapshotter>
/// @param topicName - std::string
///
/// @return expectedTopic - heex_rosbag2_snapshot::TopicDetails
inline heex_rosbag2_snapshot::TopicDetails getSnapshottersTopicByName(const std::shared_ptr<heex_rosbag2_snapshot::Snapshotter> snapshotter, const std::string& topicName)
{
  // since there can be a delay between the subscription of topic in the snapshotter and the request of its topicDetail, we shall
  // set a loop that requests for the topic name during max_retries times, with a 100ms sleep timer
  std::vector<heex_rosbag2_snapshot::TopicDetails> allTopics;
  heex_rosbag2_snapshot::TopicDetails expectedTopic;
  bool topicFound       = false;
  int counter           = 0;
  const int max_retries = 20; // max 2 seconds
  while (topicFound == false)
  {
    allTopics = snapshotter->getTopics();
    for (auto topic : allTopics)
    {
      if (topic.name == topicName)
      {
        expectedTopic = topic;
        topicFound    = true;
        HEEX_LOG(debug) << "Topic " << topicName << " found in snapshotter's topic list." << std::endl;
        break;
      }
    }
    if ((counter == max_retries) && (topicFound == false))
    {
      HEEX_LOG(warning) << "getSnapshottersTopicByName was not able to find " << topicName << " in the Snapshotter's topic list." << std::endl;
    }
    counter++;
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  return expectedTopic;
}
} // namespace ros2_sample_utils
