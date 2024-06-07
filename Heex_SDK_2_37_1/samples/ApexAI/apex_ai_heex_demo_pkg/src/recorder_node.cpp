///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "apex_ai_heex_demo_pkg/recorder_node.hpp"

#include <chrono> // for std::chrono::seconds
#include <string>

#include "logging/logging_macros.hpp" // for APEX_INFO
#include "rclcpp/rclcpp.hpp"

// Type(s) of message(s) the Recorder shall use
#include "sensor_msgs/msg/imu.hpp"         // for sensor_msgs::msg::Imu
#include "sensor_msgs/msg/nav_sat_fix.hpp" // for sensor_msgs::msg::NavSatFix
#include "std_msgs/msg/bool.hpp"           // for std_msgs::msg::Bool

namespace apex
{
namespace apex_ai_heex_demo_pkg
{
RecorderNode::RecorderNode(
    const apex::string_strict256_t& node_name,
    const apex::string_strict256_t& node_namespace,
    const apex::string_strict256_t& uuid,
    const apex::string_strict256_t& serverIp,
    const uint& serverPort,
    std::vector<apex::string_strict256_t>& topics,
    std::unordered_map<apex::string_strict256_t, apex::string_strict256_t>& topics_once_at_bagstart)
    : Recorder(uuid, serverIp, serverPort, "1.0.0"),
      _node(node_name),
      _node_state(&_node, std::chrono::seconds::max()),
      _logger{&_node, node_name},
      _topics{topics},
      _topics_once_at_bagstart(topics_once_at_bagstart)
{
  for (const auto& topic : topics_once_at_bagstart)
  {
    if (!topic.second.empty())
    {
      _subscribersForTopicThatAppearsOnce[topic.first] = _node.create_generic_polling_subscription(topic.first, topic.second, rclcpp::DefaultQoS().keep_last(1));
    }
    else
    {
      HEEX_LOG(error) << "RecorderNode::RecorderNode | Could not subscribed to " << topic.first;
    }
  }
}

RecorderNode::~RecorderNode() {}

template <typename MsgType>
bool RecorderNode::get_subscriber_recording(
    const std::vector<apex::string_strict256_t>& topics,
    const std::string& timestamp,
    const std::string& eventUuid,
    const std::string& recordIntervalStart,
    const std::string& recordIntervalEnd,
    std::string& filepath,
    unsigned int bagNum)
{
  bool res = true;
  for (auto& subscriber : _subscribers)
  {
    auto* sub = dynamic_cast<GenericSubscriberRecorder<MsgType>*>(subscriber.second.get());
    if (sub)
    {
      res = sub->get_recording(topics, timestamp, eventUuid, recordIntervalStart, recordIntervalEnd, filepath, bagNum);
    }
  }
  return res;
}

bool RecorderNode::writeToBagAllCachedMessagesAtTime(std::string& filepath, const std::string& eventUuid)
{
  if (_topics_once_at_bagstart.empty())
  {
    HEEX_LOG(info) << "looking to add single message queue";
    filepath = std::string();
    return true;
  }

  std::string filename = "recording_" + eventUuid + "_record_once";

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri        = filename;
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format  = rmw_get_serialization_format();
  converter_options.output_serialization_format = rmw_get_serialization_format();

  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

  sequential_writer->open(storage_options, converter_options);
  // create topics. Will not create already existing topics
  for (const auto& topic : _topics_once_at_bagstart)
  {
    sequential_writer->create_topic({topic.first, topic.second, rmw_get_serialization_format(), ""});

    auto subscriberIt = _subscribersForTopicThatAppearsOnce.find(topic.first);
    if (subscriberIt == _subscribersForTopicThatAppearsOnce.end())
    {
      continue;
    }

    // Write Data in the rosbag
    // take_serialized consume(retreive) all stored messages. as these topics appear only once, no more data will be availaible later
    // So we cache the data once.
    auto loaned_msgs{(subscriberIt->second)->take_serialized()};
    if (loaned_msgs.size() > 0)
    {
      auto msg                                    = loaned_msgs[loaned_msgs.size() - 1];
      _bufferForTopicThatAppearsOnce[topic.first] = msg;
    }

    auto bag_message                   = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_message->topic_name            = topic.first;
    apex::system_clock::time_point now = apex::system_clock::now();
    auto ns_count                      = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    bag_message->time_stamp            = ns_count;
    bag_message->serialized_data       = _bufferForTopicThatAppearsOnce[topic.first];
    try
    {
      sequential_writer->write(bag_message);
    }
    catch (const std::exception& e)
    {
      HEEX_LOG(error) << "Recorder:writeToBagAllCachedMessagesAtTime:writer exception : " << e.what();
      return false;
    }
  }
  sequential_writer->close();
  filepath = boost::filesystem::canonical(filename).string();
  return true;
}

template <typename MsgType>
bool RecorderNode::get_subscriber_context_values(const apex::string_strict256_t& topic, const std::string& timestamp, MsgType& msg, std::string& msg_reception_time)
{
  bool res = true;
  for (auto& subscriber : _subscribers)
  {
    auto* sub = dynamic_cast<GenericSubscriberRecorder<MsgType>*>(subscriber.second.get());
    if (sub)
    {
      res = sub->get_context_values(topic, timestamp, msg, msg_reception_time);
    }
  }
  return res;
}

bool RecorderNode::generateRequestedValues(const Heex::RecorderArgs::RecorderContextValueArgs& query, std::vector<Heex::RecorderArgs::ContextValue>& contextValues)
{
  ///
  /// Add a line for each message that is needed for context values
  /// sensor_msgs::msg::NavSatFix msg;
  /// i.e :
  /// this->get_subscriber_context_values<sensor_msgs::msg::NavSatFix>(topic,
  /// query.timestamp, msg);
  sensor_msgs::msg::NavSatFix msg;
  std::string msg_reception_time;
  bool res = this->get_subscriber_context_values<sensor_msgs::msg::NavSatFix>(_topics[0], query.timestamp, msg, msg_reception_time);
  if (res == false)
  {
    HEEX_LOG(info) << "There is no context values that matches the query ";
    return res;
  }

  std::vector<std::string> keys = this->getContextValueKeys(query);
  for (std::string& key : keys)
  {
    std::stringstream posContextValue;
    if (key == "position")
    {
      posContextValue << msg.latitude << "," << msg.longitude;
      bool success = this->addContextValue(contextValues, key, posContextValue.str());
      if (success == false)
      {
        return false;
      }
    }

    if (key == "bag_timestamp")
    {
      bool success = this->addContextValue(contextValues, "bag_timestamp", msg_reception_time);
      if (success == false)
      {
        return false;
      }
    }
  }
  return true;
}

bool RecorderNode::generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath)
{
  std::string bag_filepath = std::string();
  int bag_index            = 0;
  std::vector<std::string> bags;

  if (this->writeToBagAllCachedMessagesAtTime(bag_filepath, query.eventUuid) == false)
  {
    HEEX_LOG(error) << "RecorderNode::generateRequestedFilePaths | "
                       "writeToBagAllCachedMessagesAtTime return false";
    return false;
  }
  if (!bag_filepath.empty())
  {
    bags.push_back(bag_filepath);
  }

  std::vector<rosbag2_storage::StorageOptions> input_bags;
  std::vector<std::pair<rosbag2_storage::StorageOptions, rosbag2_transport::RecordOptions>> output_bags;
  ///
  /// Add a line for each message type that is needed for recordings
  /// sensor_msgs::msg::NavSatFix msg;
  /// i.e : this->get_subscriber_recording<sensor_msgs::msg::NavSatFix>(_topics,
  ///  query.timestamp, query.eventUuid, query.recordIntervalStart,
  /// query.recordIntervalEnd, temp_filepath);

  bool res = this->get_subscriber_recording<sensor_msgs::msg::NavSatFix>(
      {_topics[0]}, query.timestamp, query.eventUuid, query.recordIntervalStart, query.recordIntervalEnd, bag_filepath, ++bag_index);
  if (res == false)
  {
    HEEX_LOG(error) << "RecorderNode::generateRequestedFilePaths | recording for %s failed" << _topics[0];
    return res;
  }
  bags.push_back(bag_filepath);

  res = this->get_subscriber_recording<std_msgs::msg::Bool>(
      {_topics[1]}, query.timestamp, query.eventUuid, query.recordIntervalStart, query.recordIntervalEnd, bag_filepath, ++bag_index);
  if (res == false)
  {
    HEEX_LOG(error) << "RecorderNode::generateRequestedFilePaths | recording for %s failed" << _topics[1];
    return res;
  }
  bags.push_back(bag_filepath);

  res = this->get_subscriber_recording<sensor_msgs::msg::Imu>(
      {_topics[2]}, query.timestamp, query.eventUuid, query.recordIntervalStart, query.recordIntervalEnd, bag_filepath, ++bag_index);
  if (res == false)
  {
    HEEX_LOG(error) << "RecorderNode::generateRequestedFilePaths | recording for %s failed" << _topics[2];
    return res;
  }
  bags.push_back(bag_filepath);

  for (auto& bag : bags)
  {
    rosbag2_storage::StorageOptions storage;
    storage.uri = bag;
    input_bags.push_back(storage);
  }

  std::string filename = "recording_" + query.eventUuid;

  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri        = filename;
  output_storage.storage_id = "sqlite3";
  rosbag2_transport::RecordOptions output_record;
  output_record.all = true;
  output_bags.push_back({output_storage, output_record});
  rosbag2_transport::bag_rewrite(input_bags, output_bags);

  // remove temporary bags
  for (auto& bag : bags)
  {
    (void)apex::apex_ai_heex_demo_pkg::Utils::removeTemporaryFile(bag);
  }
  filepath = boost::filesystem::canonical(filename).string();
  return true;
}
} // namespace apex_ai_heex_demo_pkg
} // namespace apex
