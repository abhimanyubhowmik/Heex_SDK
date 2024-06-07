/// Copyright (c) 2024 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include <boost/filesystem.hpp>
#include <mutex>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <unordered_set>

namespace Heex
{

static constexpr int DEFAULT_QOS_HISTORY_DEPTH = 10; ///< up to the last 10 messages published on the topic will be delivered to any new subscribers when they first connect.

///@brief structure containing necessary details of a topic
///
struct TopicDetails_t
{
  std::string name;
  std::string type;
  rclcpp::QoS qosProfile{DEFAULT_QOS_HISTORY_DEPTH};

  TopicDetails_t() : name(""), type("") {}
  TopicDetails_t(const std::string& name, const std::string& type) : name(name), type(type) {}
  TopicDetails_t(const std::string& name, const std::string& type, const rclcpp::QoS& qosProfile) : name(name), type(type), qosProfile(qosProfile) {}
  TopicDetails_t(const TopicDetails_t& other) : name(other.name), type(other.type), qosProfile(other.qosProfile) {}

  bool operator==(const TopicDetails_t& t) const { return name == t.name && type == t.type; }

  bool operator<(const TopicDetails_t& t) const { return t.name < name || (t.name == name && t.type < type); }

  bool operator>(const TopicDetails_t& t) const { return t.name > name || (t.name == name && t.type > type); }

  bool empty() const { return name.empty() && type.empty(); }
};

/// @brief store a message for a topic with the received time and connection header
///
struct BagMessage_t
{
  BagMessage_t(std::shared_ptr<const rclcpp::SerializedMessage> msg, const rclcpp::Time time) : _message(msg), _time(time) {}
  std::shared_ptr<const rclcpp::SerializedMessage> _message;
  rclcpp::Time _time;
};

/// @brief <topic_name, queue>
///
typedef std::unordered_map<std::string, std::deque<Heex::BagMessage_t>> bagMessagesDeque_t;

/// @brief BagRecorder offers an API to record messages from either listed topics or all published topics on the ROS message flow.
///        Messages from callback are stored in a queue for a duration adapted to the recorder.
class BagRecorder : public rclcpp::Node
{
public:
  BagRecorder(const std::string& node_name, const rclcpp::NodeOptions& options);
  ~BagRecorder() = default;

  /// @brief records subscribed topic messages into a rosbag
  /// @details If recordIntervalEnd seconds have not yet passed since query, it'll wait intil it has.
  ///          Then, it creates a copy of the current buffer. The data that is within
  ///          the recordInterval is then written into a bag, which path is then written into bagPath
  ///
  /// @param [in] recordIntervalStart
  /// @param [in] recordIntervalEnd
  /// @param [in] queryTimestamp
  /// @param [in] queryEventUuid used for bag filename
  /// @param [out] bagPath the absolute path to the bag file
  /// @param [in] ignoredTopics [OPTIONAL] list of topic names that can be ignored and not included in the rosbag
  /// @return success (bool)
  bool recordToBag(
      const std::string& recordIntervalStart,
      const std::string& recordIntervalEnd,
      const std::string& queryTimestamp,
      const std::string& queryEventUuid,
      std::string& bagPath,
      const std::vector<std::string>& ignoredTopics = {});

  /// @brief extracts the message of given topicName that is closest to the eventTimestamp from the main buffer
  ///
  /// @param [in] msgTopicName
  /// @param [in] eventTimestamp
  /// @param [out] closestMsg
  /// @param [out] closestTime
  /// @return success (bool)
  bool extractClosestMsgToEvent(const std::string& msgTopicName, const rclcpp::Time& eventTimestamp, rclcpp::SerializedMessage& closestMsg, rclcpp::Time& closestTime);

  /// @brief gets a copy of the bagMessagesDeque_t
  /// @details Returns a copy of the deque buffer. Locks appropriate buffer before doing so.
  ///          an optional topicName vector can be given to only return a copy of the specified topic buffer, else if left
  ///          empty, it'll return the entire buffer
  ///
  /// @return bufferCopy (Heex::bagMessagesDeque_t)
  bagMessagesDeque_t getCopyCurrentBuffer(const std::string& topicName = "");

  ///@brief subscribes to all given topics (or all available topics if given input is empty)
  ///@details Will either subscribe to all topics listed in topicDetails, or if topicDetails is empty, it'll start a wall timer that
  ///         shall subscribe to any topic that is published. The timer is set to 1s, meaning every second it looks for new topics that
  ///         have not yet been subscribed to.
  ///@param topicDetails
  ///
  void subscribeToAllTopics(const std::vector<Heex::TopicDetails_t>& topicDetails);

  ///@brief Set the allowed oldest date for a message in our buffer. A proper lock is set
  ///
  ///@param oldestMsgDate
  void setOldestBufferMsgDate(const int oldestMsgDate);

private:
  ///@brief Adds a topic to the list of topics to which we are subscribed to (_subscribedTopicDetails) and subscribes to it.
  ///
  ///@param topicDetail
  void addTopicAndSubscribe(const TopicDetails_t& topicDetail);

  ///@brief Subscribes to all topics in the message flow that come from publishers. Only subscribes to topics that have not yet been
  ///       subscribed to. Manages any QoS policy issue.
  ///
  void pollAllPublishingTopics();

  ///@brief subscribes to given topic. Subscription is a generic one, serializing the messages.
  ///
  ///@param topicDetail
  void subscribeToTopic(const TopicDetails_t& topicDetail);

  ///@brief callback for all topics. the function acquires a lock to updates the queue
  ///
  ///@param msg
  ///@param topicDetail
  void subscriberCallback(const std::shared_ptr<const rclcpp::SerializedMessage> msg, const TopicDetails_t& topicDetail);

  std::vector<std::shared_ptr<rclcpp::GenericSubscription>> _subscribers; ///< list of all subscribers
  std::vector<TopicDetails_t> _subscribedTopicDetails;                    ///< list of each subscribed topic detail

  std::mutex _bagMessageDequeMutex;    ///< mutex for _bagMessageDeque
  bagMessagesDeque_t _bagMessageDeque; ///< map containig the buffer for each of the topic name: <topic_name, message_buffer>

  std::mutex _oldestBufferMsgDateMutex; ///< mutex for _oldestBufferMsgDate
  int _oldestBufferMsgDate;             ///< [in seconds] oldest date we allow for a message to be kept in given buffer

  std::shared_ptr<rclcpp::TimerBase> _poll_topic_timer; ///< wall timer in case we want to poll all topics. Is killed at first record
};
} // namespace Heex
