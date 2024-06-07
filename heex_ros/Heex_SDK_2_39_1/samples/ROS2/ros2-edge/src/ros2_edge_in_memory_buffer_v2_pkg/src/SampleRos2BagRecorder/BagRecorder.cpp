/// Copyright (c) 2024 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#include "BagRecorder.h"

#include <algorithm>
#include <boost/filesystem.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <thread>

#include "HeexUtilsFileOperations.h"
#include "HeexUtilsLog.h"
#include "ros2SampleUtils.hpp"

static constexpr int MAX_BUFFER_SIZE                 = 10000; // Max buffer size
static constexpr int DEFAULT_MAX_OLDEST_MESSAGE_DATE = 15;    ///< [seconds] Default oldest message kept in buffer is 15s.

namespace Heex
{
BagRecorder::BagRecorder(const std::string& node_name, const rclcpp::NodeOptions& options) : rclcpp::Node(node_name, options), _oldestBufferMsgDate{DEFAULT_MAX_OLDEST_MESSAGE_DATE}
{
}

void BagRecorder::subscribeToAllTopics(const std::vector<Heex::TopicDetails_t>& topicDetails)
{
  if (topicDetails.empty())
  {
    // subscribe to all topics
    HEEX_LOG(info) << "Subscribe to all publishing topics..." << std::endl;

    // here we create a wall timere that shall look for topics every second. This timer is destroyed after the first record
    // since we can consider that when the first event has been triggered, no new publisher shall be added into the message flow
    _poll_topic_timer = create_wall_timer(std::chrono::seconds(1), [this]() -> void { pollAllPublishingTopics(); });
  }
  else
  {
    for (const Heex::TopicDetails_t& topicDetail : topicDetails)
    {
      BagRecorder::addTopicAndSubscribe(topicDetail);
    }
  }
}

// ignoredTopics is optional: list of topic names that can be ignored and not included in the rosbag
bool BagRecorder::recordToBag(
    const std::string& recordIntervalStart,
    const std::string& recordIntervalEnd,
    const std::string& queryTimestamp,
    const std::string& queryEventUuid,
    std::string& bagPath,
    const std::vector<std::string>& ignoredTopics /* = {} */)
{
  auto tic                  = rclcpp::Clock().now();
  const float intervalStart = std::stof(recordIntervalStart);
  const float intervalEnd   = std::stof(recordIntervalEnd);
  if ((intervalStart >= 0.F) && (intervalEnd <= 0.F))
  {
    HEEX_LOG(warning) << "BagRecorder::recordToBag | bag will be empty as recordLength is <= 0s" << std::endl;
    return false;
  }

  const rclcpp::Time eventTime = ros2_sample_utils::iso_extended_str_to_rclcpptime(queryTimestamp);
  if (_subscribedTopicDetails.empty())
  {
    HEEX_LOG(warning) << "BagRecorder::recordToBag | _subscribedTopicDetails list is empty" << std::endl;
    return false;
  }

  const auto startTime = eventTime + rclcpp::Duration::from_seconds(intervalStart);
  const auto endTime   = eventTime + rclcpp::Duration::from_seconds(intervalEnd);
  HEEX_LOG(info) << "BagRecorder::recordToBag | Creation of a rosBag of " << intervalEnd - intervalStart << "s around " << queryTimestamp << " has started." << std::endl;

  if (endTime > rclcpp::Clock().now())
  {
    HEEX_LOG(info) << "BagRecorder::recordToBag | Waiting for the buffer to have all wanted data..." << std::endl;
    while (endTime > rclcpp::Clock().now())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  // If we were polling for topics to subscribe to, we can consider that after the first record, all topics are polled. So now we can kill the wall timer
  if (_poll_topic_timer != nullptr)
  {
    _poll_topic_timer->cancel();
    _poll_topic_timer = nullptr;
  }

  // Construct bag folder path
  const std::string bagFolderName           = "rosbag_" + queryTimestamp + "_" + queryEventUuid;
  const boost::filesystem::path bagFullPath = boost::filesystem::temp_directory_path() / "RosBags" / bagFolderName;

  // Fetch a copy of current queue
  const Heex::bagMessagesDeque_t copyBagMessageDeque = BagRecorder::getCopyCurrentBuffer();
  if (copyBagMessageDeque.empty())
  {
    HEEX_LOG(warning) << "BagRecorder::recordToBag | queue is empty" << std::endl;
    return false;
  }

  // Check if we have enough space on disk before creating the recording
  unsigned long long totalBuffSize = 0;
  for (const auto& pair : copyBagMessageDeque)
  {
    bool isIgnored = std::find(ignoredTopics.begin(), ignoredTopics.end(), pair.first) != ignoredTopics.end();
    if (isIgnored)
    {
      continue;
    }
    for (const auto& message : pair.second)
    {
      // Add the size of each BagMessage_t object in the deque
      totalBuffSize += static_cast<unsigned long long>(sizeof(Heex::BagMessage_t));
    }
  }
  if (!HeexUtils::FileOperations::isDiskSpaceEnough(bagFullPath, totalBuffSize))
  {
    HEEX_LOG(error) << "BagRecorder::recordToBag | There is not enough space on disk to save the recording : " << totalBuffSize << std::endl;
    return false;
  }

  // create the bag writer
  rosbag2_cpp::Writer bag_writer;
  try
  {
    bag_writer.open(bagFullPath.string());
  }
  catch (const std::exception& ex)
  {
    HEEX_LOG(error) << "BagRecorder::recordToBag | unable to open bag for writing : " << bagFullPath.string() << std::endl;
    return false;
  }
  // Write all data in buffer that is surrounding the event within the given interval.
  for (const auto& topicDetail : _subscribedTopicDetails)
  {
    auto found     = copyBagMessageDeque.find(topicDetail.name);
    bool isIgnored = std::find(ignoredTopics.begin(), ignoredTopics.end(), topicDetail.name) != ignoredTopics.end();
    if ((found == copyBagMessageDeque.end()) || isIgnored)
    {
      HEEX_LOG(info) << "BagRecorder::recordToBag | Requested topic " << topicDetail.name << " is not subscribed to or is ignored, skipping." << std::endl;
      continue;
    }
    rosbag2_storage::TopicMetadata tm;
    tm.name                 = topicDetail.name;
    tm.type                 = topicDetail.type;
    tm.serialization_format = "cdr";
    bag_writer.create_topic(tm);

    // and now we parse all the messages of given topic, and keep only the ones within the interval
    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    for (const auto& bagMsg : found->second)
    {
      auto ret = rcutils_system_time_now(&bag_message->time_stamp);
      if (ret != RCL_RET_OK)
      {
        HEEX_LOG(error) << "BagRecorder::recordToBag | Failed to assign time to rosbag message." << std::endl;
        return false;
      }
      // only write data that is inside the wanted interval
      if ((bagMsg._time.nanoseconds() >= startTime.nanoseconds()) && (bagMsg._time.nanoseconds() <= endTime.nanoseconds()))
      {
        bag_message->topic_name      = tm.name;
        bag_message->time_stamp      = bagMsg._time.nanoseconds();
        bag_message->serialized_data = std::make_shared<rcutils_uint8_array_t>(bagMsg._message->get_rcl_serialized_message());
        bag_writer.write(bag_message);
      }
    }
  }
  bagPath = bagFullPath.string();
  return true;
}

bool BagRecorder::extractClosestMsgToEvent(const std::string& msgTopicName, const rclcpp::Time& eventTimestamp, rclcpp::SerializedMessage& closestMsg, rclcpp::Time& closestTime)
{
  Heex::bagMessagesDeque_t topicMessageDeque = BagRecorder::getCopyCurrentBuffer(msgTopicName);
  if ((topicMessageDeque.empty()) || (topicMessageDeque[msgTopicName].empty()))
  {
    HEEX_LOG(warning) << "BagRecorder::extractClosestMsgToEvent | " << msgTopicName << "'s buffer is empty, message can be extracted" << std::endl;
    return false;
  }

  // Find the message with the closest timestamp to the query timestamp
  closestMsg  = *topicMessageDeque[msgTopicName].front()._message;
  closestTime = topicMessageDeque[msgTopicName].front()._time;
  while (!topicMessageDeque[msgTopicName].empty() && topicMessageDeque[msgTopicName].front()._time > eventTimestamp)
  {
    closestMsg  = *topicMessageDeque[msgTopicName].front()._message;
    closestTime = topicMessageDeque[msgTopicName].front()._time;
    topicMessageDeque[msgTopicName].pop_front();
  }
  return true;
}

void BagRecorder::addTopicAndSubscribe(const Heex::TopicDetails_t& topicDetail)
{
  const auto it = std::find(_subscribedTopicDetails.begin(), _subscribedTopicDetails.end(), topicDetail);
  if (it == _subscribedTopicDetails.end())
  {
    Heex::TopicDetails_t cleanTopic;
    try
    {
      cleanTopic.name       = rclcpp::expand_topic_or_service_name(topicDetail.name, this->get_name(), this->get_namespace());
      cleanTopic.type       = topicDetail.type;
      cleanTopic.qosProfile = topicDetail.qosProfile;
      subscribeToTopic(cleanTopic);
    }
    catch (const rclcpp::exceptions::InvalidTopicNameError& err)
    {
      HEEX_LOG(warning) << "BagRecorder::addTopicAndSubscribe | Requested topic " << topicDetail.name << " is invalid, skipping." << std::endl;
    }
  }
}

void BagRecorder::subscribeToTopic(const Heex::TopicDetails_t& topicDetail)
{
  auto opts                              = rclcpp::SubscriptionOptions{};
  opts.topic_stats_options.state         = rclcpp::TopicStatisticsState::Enable;
  opts.topic_stats_options.publish_topic = topicDetail.name + "/statistics";

  HEEX_LOG(info) << "BagRecorder::subscribeToTopic | Subscribing to " << topicDetail.name << std::endl;
  auto callback     = std::bind(&BagRecorder::subscriberCallback, this, std::placeholders::_1, topicDetail);
  auto subscription = this->create_generic_subscription(topicDetail.name, topicDetail.type, topicDetail.qosProfile, callback, opts);
  _subscribedTopicDetails.push_back(topicDetail);
  _subscribers.push_back(subscription);
}

void BagRecorder::pollAllPublishingTopics()
{
  // fetch all topics in the ROS2 message flow
  const auto topicNamesAndTypes = this->get_topic_names_and_types();

  for (const auto& nameAndType : topicNamesAndTypes)
  {
    if (nameAndType.second.size() < 1)
    {
      HEEX_LOG(error) << "BagRecorder::pollAllPublishingTopics | Subscribed topic has no associated type." << std::endl;
      return;
    }

    if (nameAndType.second.size() > 1)
    {
      HEEX_LOG(error) << "BagRecorder::pollAllPublishingTopics | Subscribed topic has more than one associated type." << std::endl;
      return;
    }

    Heex::TopicDetails_t topicDetail{};
    topicDetail.name = nameAndType.first;
    topicDetail.type = nameAndType.second[0];
    const auto it    = std::find(_subscribedTopicDetails.begin(), _subscribedTopicDetails.end(), topicDetail);
    if (it == _subscribedTopicDetails.end()) // only subscribe to topics that haven't been subscribed to yet
    {
      // We subscribe only to the publisher nodes
      const std::vector<rclcpp::TopicEndpointInfo> publisherTopicsEndPointVec = this->get_publishers_info_by_topic(nameAndType.first, false);
      if (publisherTopicsEndPointVec.size() > 0)
      {
        if (publisherTopicsEndPointVec.size() > 1)
        {
          HEEX_LOG(warning) << "BagRecorder::pollAllPublishingTopics | " << nameAndType.first
                            << " has more than one endpoint (ie: several publishers on same topic). The QoS profile "
                               "applied for the subscription shall be selected on the first endpoint found."
                            << std::endl;
        }
        // We retrieve the QoS profile and add it to the topic details
        topicDetail.qosProfile                  = publisherTopicsEndPointVec[0].qos_profile();
        // Sometimes the history is not available (depth set to 0). In that case, we set the depth to 10 to avoid failures
        const rmw_qos_profile_t rmw_qos_profile = topicDetail.qosProfile.get_rmw_qos_profile();
        if ((rmw_qos_profile.depth == 0) && (rmw_qos_profile.history != rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_ALL))
        {
          topicDetail.qosProfile.keep_last(Heex::DEFAULT_QOS_HISTORY_DEPTH);
        }
        BagRecorder::addTopicAndSubscribe(topicDetail);
      }
    }
  }
}

void BagRecorder::setOldestBufferMsgDate(const int oldestMsgDate)
{
  HEEX_LOG(info) << "BagRecorder::setOldestBufferMsgDate | Updating the oldest allowed message in our buffers to " << oldestMsgDate << "s." << std::endl;
  const std::lock_guard<std::mutex> lock(_oldestBufferMsgDateMutex);
  _oldestBufferMsgDate = oldestMsgDate;
}

void BagRecorder::subscriberCallback(const std::shared_ptr<const rclcpp::SerializedMessage> msg, const Heex::TopicDetails_t& topicDetail)
{
  rclcpp::Time receivedTime = rclcpp::Clock().now();
  int maxOldestMessageDate; // Fetch the oldest allowed date in the buffer so that we can delete all that are older
  {
    const std::lock_guard<std::mutex> lock(_oldestBufferMsgDateMutex);
    maxOldestMessageDate = _oldestBufferMsgDate; ///< [seconds]

  } // release _oldestBufferMsgDateMutex lock
  BagMessage_t message(msg, receivedTime);
  {
    const std::lock_guard<std::mutex> lock(_bagMessageDequeMutex);
    auto pos = _bagMessageDeque.find(topicDetail.name);
    if (pos != _bagMessageDeque.end()) // Only subscribe to message not yet subscribed to
    {
      rclcpp::Time oldestMsgDate = pos->second.front()._time;
      while ((rclcpp::Duration(receivedTime - oldestMsgDate).seconds() > maxOldestMessageDate) || (pos->second.size() >= MAX_BUFFER_SIZE))
      {
        pos->second.pop_front();
        oldestMsgDate = pos->second.front()._time;
      }
      pos->second.push_back(message);
    }
    else
    {
      // first time we receive a message for this topic.
      _bagMessageDeque.emplace(topicDetail.name, std::deque<Heex::BagMessage_t>{message});
    }
  } // release _bagMessageDequeMutex lock
}

/// topicName optional, if left empty, returns entire buffer. Default empty.
Heex::bagMessagesDeque_t BagRecorder::getCopyCurrentBuffer(const std::string& topicName /* = "" */)
{
  const std::lock_guard<std::mutex> lock(_bagMessageDequeMutex);
  if (topicName.empty())
  {
    return _bagMessageDeque;
  }
  else
  {
    auto found = _bagMessageDeque.find(topicName);
    if (found == _bagMessageDeque.end())
    {
      HEEX_LOG(error) << "BagRecorder::getCopyCurrentBuffer | requesting the buffer for topic " << topicName << ". But it is not part of the buffer. Returning entire buffer..."
                      << std::endl;
      return _bagMessageDeque;
    }
    else
    {
      return Heex::bagMessagesDeque_t{{topicName, found->second}};
    }
  }
}
} // namespace Heex
