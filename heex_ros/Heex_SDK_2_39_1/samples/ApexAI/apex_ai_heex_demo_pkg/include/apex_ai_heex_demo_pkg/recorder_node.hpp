/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#ifndef RECORDER_NODE_HPP_
#define RECORDER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp> // for rosbag
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_transport/bag_rewrite.hpp>
#include <unordered_map>

#include "apex_ai_heex_demo_pkg/visibility_control.hpp" // for APEX_AI_HEEX_DEMO_PKG_PUBLIC
#include "executor/apex_node_base.hpp"                  // for apex::executor::apex_node_base
#include "logging/logging.hpp"                          // for apex::logging::Logger
#include "rclcpp/generic_polling_subscription.hpp"
#include "rclcpp/polling_subscription.hpp" // for rclcpp::PollingSubscription

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wold-style-cast"
  #pragma GCC diagnostic ignored "-Wconversion"
  #pragma GCC diagnostic ignored "-Wsign-conversion"
  #pragma GCC diagnostic ignored "-Wuseless-cast"
  #include <boost/date_time/posix_time/posix_time.hpp>
  #include <boost/filesystem.hpp>
  #pragma GCC diagnostic pop
#endif

#include "Recorder.h"

namespace apex
{
namespace apex_ai_heex_demo_pkg
{
namespace Utils
{

boost::posix_time::ptime convert_ros_time_to_boost(apex::system_clock::time_point time)
{
  boost::posix_time::ptime pt = boost::posix_time::from_time_t(apex::system_clock::to_time_t(time));
  pt += boost::posix_time::microseconds(std::chrono::duration_cast<std::chrono::microseconds>(time.time_since_epoch()).count() % 1000000);
  return pt;
}

unsigned int removeTemporaryFile(const std::string& pathStr)
{
  boost::system::error_code ec;
  boost::filesystem::path path{pathStr};
  uintmax_t res = boost::filesystem::remove_all(path, ec);
  if (ec == boost::system::errc::success && res > 0)
  {
    return 0;
  }
  else
  {
    return int(ec.value());
  }
  return 0;
}
} // namespace Utils

class GenericSubscriberRecorderBase
{
public:
  virtual ~GenericSubscriberRecorderBase() {}
};

template <class MsgType> class APEX_AI_HEEX_DEMO_PKG_PUBLIC GenericSubscriberRecorder : public GenericSubscriberRecorderBase, apex::executor::executable_item
{
public:
  /// @brief Construct a new GenericSubscriberRecorder object
  /// GenericSubscriberRecorder is a class that holds polling subscriptions to
  /// topics of the same type. The class also performs extraction of context
  /// values and bag recording
  ///
  /// @param node_name ROS node  name
  /// @param node_namespace ROS node namespace
  GenericSubscriberRecorder(
      const apex::string_strict256_t& node_name,
      const apex::string_strict256_t& node_namespace,
      const std::vector<apex::string_strict256_t>& topics,
      rclcpp::Node& node,
      apex::NodeState& node_state)
      : apex::executor::executable_item{node, node_state},
        _logger{&get_rclcpp_node(), "GenericSubscriberRecorder"}
  {
    // Setup the QoS, using default QoS
    rclcpp::QoS qos = rclcpp::DefaultQoS().keep_last(QUEUE_SIZE);
    for (const auto& topic : topics)
    {
      _subscribers[topic] = get_rclcpp_node().template create_polling_subscription<MsgType>(topic, qos);
    }

    std::map<std::string, std::vector<std::string>> topic_names_and_types = get_rclcpp_node().get_topic_names_and_types();
    for (const auto& pair : topic_names_and_types)
    {
      if (topic_names_and_types.count(pair.first) > 0 && pair.first == topics[0])
      {
        const auto& msg_types = pair.second;
        if (!msg_types.empty())
        {
          _topic_type = msg_types.front();
        }
      }
    }
  }

  /// Iterate of messages in buffer and find the closest message with the
  /// requested timestamp
  bool get_context_values(const apex::string_strict256_t& topic, const std::string& timestamp, MsgType& msg, std::string& msg_reception_time)
  {
    auto subscriberIt = _subscribers.find(topic);
    if (subscriberIt == _subscribers.end())
    {
      return false;
    }
    boost::posix_time::ptime query_timestamp = boost::posix_time::from_iso_extended_string(timestamp);
    HEEX_LOG(info) << "Recorder::get_context_values | Extracting data at " << boost::posix_time::to_iso_extended_string(query_timestamp);
    auto loaned_msgs{(subscriberIt->second)->read()};

    // Cache the ContextValue of the best candidate
    auto last_msg_that_satisfy_req_it = std::find_if(
        std::make_reverse_iterator(loaned_msgs.end()),
        std::make_reverse_iterator(loaned_msgs.begin()),
        [this, query_timestamp](const auto& msg)
        {
          if (msg.info().valid())
          {
            apex::system_clock::time_point msg_timestamp = msg.info().reception_timestamp();
            boost::posix_time::ptime msg_pt              = apex::apex_ai_heex_demo_pkg::Utils::convert_ros_time_to_boost(msg_timestamp);
            return msg_pt <= query_timestamp;
          }
          else
          {
            return false;
          }
        });

    if (last_msg_that_satisfy_req_it == loaned_msgs.rend())
    {
      return false;
    }
    unsigned int data_idx = loaned_msgs.size() - 1u - static_cast<unsigned int>(std::distance(loaned_msgs.rbegin(),
                                                                                              last_msg_that_satisfy_req_it)); // index in the DDS queue
    msg                   = loaned_msgs[data_idx].data();

    // create a Boost.Date_Time ptime from time_point.
    apex::system_clock::time_point msg_timestamp = loaned_msgs[data_idx].info().reception_timestamp();
    boost::posix_time::ptime msg_pt              = apex::apex_ai_heex_demo_pkg::Utils::convert_ros_time_to_boost(msg_timestamp);

    msg_reception_time = boost::posix_time::to_iso_extended_string(msg_pt);
    return true;
  }

  /// Record data into a bag for all the topics listed in topics from
  /// recordIntervalStart to recordIntervalEnd
  bool get_recording(
      const std::vector<apex::string_strict256_t>& topics,
      const std::string& timestamp,
      const std::string& eventUuid,
      const std::string& recordIntervalStart,
      const std::string& recordIntervalEnd,
      std::string& filepath,
      unsigned int bagNum)
  {
    std::string filename;
    filename = "recording_" + eventUuid + "_" + std::to_string(bagNum);

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri        = filename;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format  = rmw_get_serialization_format();
    converter_options.output_serialization_format = rmw_get_serialization_format();

    auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

    sequential_writer->open(storage_options, converter_options);
    // create topics. Will not create already existing topics
    for (const auto& topic : topics)
    {
      sequential_writer->create_topic({topic, _topic_type, rmw_get_serialization_format(), ""});
    }

    int milliseconds = static_cast<int>(atof(recordIntervalStart.c_str()) * 1000);

    boost::posix_time::ptime query_start = boost::posix_time::from_iso_extended_string(timestamp) + boost::posix_time::milliseconds(milliseconds);

    milliseconds = static_cast<int>(atof(recordIntervalEnd.c_str()) * 1000);

    boost::posix_time::ptime query_end = boost::posix_time::from_iso_extended_string(timestamp) + boost::posix_time::milliseconds(milliseconds);

    HEEX_LOG(info) << "Recorder::get_recording | Extracting data between [" << boost::posix_time::to_iso_extended_string(query_start) << "; "
                   << boost::posix_time::to_iso_extended_string(query_end) << "]";
    // waiting untill all data is in the range of the query
    while (rclcpp::ok())
    {
      boost::posix_time::ptime current_time = apex::apex_ai_heex_demo_pkg::Utils::convert_ros_time_to_boost(apex::system_clock::now());
      if (current_time >= query_end)
      {
        break;
      }
      double sec_to_sleep = static_cast<double>((query_end - current_time).total_milliseconds() / 1000);
      rclcpp::Rate rate(1.0 / sec_to_sleep);
      rate.sleep();
    }

    for (auto& topic : topics)
    {
      auto subscriberIt = _subscribers.find(topic);
      if (subscriberIt == _subscribers.end())
      {
        continue;
      }

      // Write Data in the rosbag
      {
        auto loaned_msgs{(subscriberIt->second)->read()};
        for (const auto& msg : loaned_msgs)
        {
          if (msg.info().valid())
          {
            apex::system_clock::time_point msg_timestamp = msg.info().reception_timestamp();
            boost::posix_time::ptime msg_pt              = apex::apex_ai_heex_demo_pkg::Utils::convert_ros_time_to_boost(msg_timestamp);

            if (msg_pt >= query_start && msg_pt <= query_end)
            {
              rclcpp::SerializedMessage serialized_msg;

              rclcpp::Serialization<MsgType> serialization;
              serialization.serialize_message(&msg, &serialized_msg);

              auto bag_message             = std::make_shared<rosbag2_storage::SerializedBagMessage>();
              bag_message->topic_name      = topic;
              auto ns_count                = std::chrono::duration_cast<std::chrono::nanoseconds>(msg_timestamp.time_since_epoch()).count();
              bag_message->time_stamp      = ns_count;
              bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(&serialized_msg.get_rcl_serialized_message(), [](rcutils_uint8_array_t* /* data */) {});
              try
              {
                sequential_writer->write(bag_message);
              }
              catch (const std::exception& e)
              {
                HEEX_LOG(error) << "Recorder:get_recording:writer exception : " << e.what();
                return false;
              }
            }
          }
        }
      }
    }
    sequential_writer->close();
    filepath = boost::filesystem::canonical(filename).string();
    return true;
  }

private:
  void execute_impl() override {}

  apex::executor::subscription_list get_triggering_subscriptions_impl() const override { return {}; }
  apex::logging::Logger<> _logger;

  /// Map of polling subscriptions on the same msg type. Topics are keys.
  std::unordered_map<apex::string_strict256_t, typename rclcpp::PollingSubscription<MsgType>::SharedPtr> _subscribers;
  apex::string_strict256_t _topic_type;
  const size_t QUEUE_SIZE = 100;
};

class APEX_AI_HEEX_DEMO_PKG_PUBLIC RecorderNode : public Recorder
{
  /// @brief Header file for the Recorder for the rosbag online (edge) detection
  /// on APEX AI Env on ROS 2.
public:
  /// @brief Construct a new RecorderNode object
  ///
  /// @param node_name ROS node  name
  /// @param node_namespace ROS node namespace
  /// @param uuid
  /// @param serverIp
  /// @param serverPort
  RecorderNode(
      const apex::string_strict256_t& node_name,
      const apex::string_strict256_t& node_namespace,
      const apex::string_strict256_t& uuid,
      const apex::string_strict256_t& serverIp,
      const uint& serverPort,
      std::vector<apex::string_strict256_t>& topics,
      std::unordered_map<apex::string_strict256_t, apex::string_strict256_t>& topics_once_at_bagstart);

  virtual ~RecorderNode();

  /**
   * NOTE templated function is instantiated here so
   * that clients will not need to instantiate the function for
   * each type
   */

  template <typename MsgType> void add_subscriber(const std::vector<apex::string_strict256_t>& topics)
  {
    for (const apex::string_strict256_t& topic : topics)
    {
      // Create a new GenericSubscriberRecorder for the given topic type MsgType
      auto recorder = std::make_shared<GenericSubscriberRecorder<MsgType>>(_node.get_name(), _node.get_namespace(), topics, _node, _node_state);

      // Add the recorder to the map of subscribers
      _subscribers.emplace(topic, std::move(recorder));
    }
  }

  template <typename MsgType>
  bool get_subscriber_recording(
      const std::vector<apex::string_strict256_t>& topics,
      const std::string& timestamp,
      const std::string& eventUuid,
      const std::string& recordIntervalStart,
      const std::string& recordIntervalEnd,
      std::string& filepath,
      unsigned int bagNum);

  template <typename MsgType>
  bool get_subscriber_context_values(const apex::string_strict256_t& topic, const std::string& timestamp, MsgType& msg, std::string& msg_reception_time);

protected:
  /// @brief Returns the value as Heex::RecorderArgs::ContextValue for the
  /// advertized context by the Recorder (e.g. position). This method is virtual
  /// and by default return an empty string standing for error. It requires to be
  /// defined within any subclasses to the address the logic of getting data
  /// value advertized by your Recorder. Any value with an empty string will be
  /// recognized as an error.
  ///
  /// @param query Current RecorderContextValueArgs-structured query that holds
  /// the arguments of the request of ContextValuelue extraction.
  /// @param contextValues The pass-by-value contextValues variable that contains
  /// the key and values of ContextValues advertized by the Recorder. Edit it
  /// using the addContextValue(contextValues, "key", "value"). E.g. A context
  /// value named "position" would return the value "48.8582651,2.2938142".
  ///
  /// @return true Notify that the generation has completed with success.
  /// @return false Notify that the generation has failed and ignore the result.
  virtual bool generateRequestedValues(const Heex::RecorderArgs::RecorderContextValueArgs& query, std::vector<Heex::RecorderArgs::ContextValue>& contextValues) override;

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
  /// data on the machine. E.g. A filepath would return the value
  /// "/tmp/my_file.txt". We encourage to limit the use of special character like
  /// spaces within the filepath. Special characters ";" and ":" are stricly
  /// forbidden.
  ///
  /// @return true Notify that the generation has completed with success.
  /// @return false Notify that the generation has failed and ignore the result.
  virtual bool generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath) override;

  bool writeToBagAllCachedMessagesAtTime(std::string& filepath, const std::string& eventUuid);

private:
  rclcpp::Node _node;
  apex::NodeState _node_state;
  apex::logging::Logger<> _logger;
  std::vector<apex::string_strict256_t> _topics;

  /// Map of topics and GenericSubscriberRecorderBase. topics can be of
  /// different messages types
  std::unordered_map<apex::string_strict256_t, std::shared_ptr<GenericSubscriberRecorderBase>> _subscribers;
  // create a generic polling subscription topics that only need latest value
  // for each topic map of topic and generic polling subscription. These
  // subscriptions will only keep the last value
  std::unordered_map<apex::string_strict256_t, std::shared_ptr<rclcpp::GenericPollingSubscription>> _subscribersForTopicThatAppearsOnce;
  std::unordered_map<apex::string_strict256_t, std::shared_ptr<rcutils_uint8_array_t>> _bufferForTopicThatAppearsOnce;
  std::unordered_map<apex::string_strict256_t, apex::string_strict256_t> _topics_once_at_bagstart;
};
} // namespace apex_ai_heex_demo_pkg
} // namespace apex

#endif // RECORDER_NODE_HPP_
