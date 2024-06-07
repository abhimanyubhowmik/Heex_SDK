// Copyright (c) 2018-2021, Open Source Robotics Foundation, Inc., GAIA Platform, Inc., All rights reserved.  // NOLINT
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef ROSBAG2_SNAPSHOT__SNAPSHOTTER_HPP_
#define ROSBAG2_SNAPSHOT__SNAPSHOTTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
// Remove includes to rosbag2_snapshot_msgs dependencies
// #include <rosbag2_snapshot_msgs/msg/topic_details.hpp>
// #include <rosbag2_snapshot_msgs/srv/trigger_snapshot.hpp>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <rosbag2_cpp/writer.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace heex_rosbag2_snapshot
{
using namespace std::chrono_literals; // NOLINT
// MODIFIED: removed rosbag2_snapshot_msgs dependency
// using DetailsMsg = rosbag2_snapshot_msgs::msg::TopicDetails;

struct TopicDetails
{
  std::string name;
  std::string type;

  TopicDetails() {}
  TopicDetails(std::string name, std::string type) : name(name), type(type) {}
  // MODIFIED: adding constructor with QoS profile
  TopicDetails(std::string name, std::string type, rclcpp::QoS qosProfile) : name(name), type(type), qosProfile(qosProfile) {}

  bool operator==(const TopicDetails& t) const { return name == t.name && type == t.type; }

  bool operator<(const TopicDetails& t) const { return t.name < name || (t.name == name && t.type < type); }

  bool operator>(const TopicDetails& t) const { return t.name > name || (t.name == name && t.type > type); }

  // MODIFIED: remove asMessage() as not needed in our case, since we removed whole rosbag2_snapshot_msgs dependency
  // DetailsMsg asMessage() const
  // {
  //   DetailsMsg msg{};
  //   msg.name = name;
  //   msg.type = type;
  //   return msg;
  // }

  // MODIFIED: added empty() function
  bool empty() const { return name.empty() && type.empty(); }

  // MODIFIED: adding QoS profile with a default history depth of 10
  rclcpp::QoS qosProfile{10};
};

// Remapping of the rosbag_snapshot_msgs::TriggerSnapshot::Request (req) structure from rosbag_snapshot to Heex query structure
struct SnapshotRequest
{
  rclcpp::Time start_time;
  rclcpp::Time stop_time;
  std::string filename;
  std::vector<TopicDetails> topics;
  rclcpp::Time last_write_time;

  SnapshotRequest(const rclcpp::Time& start, const rclcpp::Time& stop, const std::string& fn, const std::vector<TopicDetails>& tpcs, const rclcpp::Time& last)
      : start_time(start),
        stop_time(stop),
        filename(fn),
        topics(tpcs),
        last_write_time(last)
  {
  }
};

struct SnapshotInstantRequest
{
  rclcpp::SerializedMessage msg;
  rclcpp::Time query_time;
  rclcpp::Time msg_time;
  rclcpp::Duration search_offset; // how far back shall we look
  TopicDetails topic;

  SnapshotInstantRequest(const rclcpp::SerializedMessage& msg, const rclcpp::Time& qt, const rclcpp::Time& mt, const rclcpp::Duration& so, const TopicDetails& tpc)
      : msg(msg),
        query_time(qt),
        msg_time(mt),
        search_offset(so),
        topic(tpc)
  {
  }
};

// Remapping structure from rosbag_snapshot_msgs::TriggerSnapshot::Response (res) rosbag_snapshot to Heex query structure
struct SnapshotResponse
{
  bool success;
  std::string message;
  const std::string NO_DATA_MESSAGE = "no messages buffered on selected topics";

  SnapshotResponse(bool suc, const std::string& m) : success(suc), message(m) {}
};

class Snapshotter;

/* Configuration for a single topic in the Snapshotter node. Holds
 * the buffer limits for a topic by duration (time difference between newest and oldest message)
 * and memory usage, in bytes.
 */
struct SnapshotterTopicOptions
{
  // When the value of duration_limit_, do not truncate the buffer
  // no matter how large the duration is
  static const rclcpp::Duration NO_DURATION_LIMIT;
  // When the value of memory_limit_, do not trunctate the buffer
  // no matter how much memory it consumes (DANGROUS)
  static const int32_t NO_MEMORY_LIMIT;
  // When the value of duration_limit_, inherit the limit from
  // the node's configured default
  static const rclcpp::Duration INHERIT_DURATION_LIMIT;
  // When the value of memory_limit_, inherit the limit from
  // the node's configured default
  static const int32_t INHERIT_MEMORY_LIMIT;

  // Maximum difference in time from newest and oldest message in
  // buffer before older messages are removed
  rclcpp::Duration duration_limit_;
  // Maximum memory usage of the buffer before older messages are removed
  int32_t memory_limit_;

  SnapshotterTopicOptions(rclcpp::Duration duration_limit = INHERIT_DURATION_LIMIT, int32_t memory_limit = INHERIT_MEMORY_LIMIT);
};

/* Configuration for the Snapshotter node. Contains default limits for memory and duration
 * and a map of topics to their limits which may override the defaults.
 */
struct SnapshotterOptions
{
  // Duration limit to use for a topic's buffer if one is not specified
  rclcpp::Duration default_duration_limit_;
  // Memory limit to use for a topic's buffer if one is not specified
  int32_t default_memory_limit_;
  // Flag if all topics should be recorded
  bool all_topics_;

  // MODIFIED: Provides list of topics to snapshot and snapshot the latest message only
  std::vector<TopicDetails> topics_once_at_bagstart_;

  typedef std::map<TopicDetails, SnapshotterTopicOptions> topics_t;
  // Provides list of topics to snapshot and their limit configurations
  topics_t topics_;

  SnapshotterOptions(rclcpp::Duration default_duration_limit = rclcpp::Duration(30s), int32_t default_memory_limit = -1);

  // Add a new topic to the configuration, returns false if the topic was already present
  bool addTopic(
      const TopicDetails& topic_details,
      rclcpp::Duration duration_limit = SnapshotterTopicOptions::INHERIT_DURATION_LIMIT,
      int32_t memory_limit            = SnapshotterTopicOptions::INHERIT_MEMORY_LIMIT);
};

/* Stores a buffered message of an ambiguous type and it's associated metadata (time of arrival),
 * for later writing to disk
 */
struct SnapshotMessage
{
  SnapshotMessage(std::shared_ptr<const rclcpp::SerializedMessage> _msg, rclcpp::Time _time);
  std::shared_ptr<const rclcpp::SerializedMessage> msg;
  // ROS time when messaged arrived (does not use header stamp)
  rclcpp::Time time;
};

/* Stores a queue of buffered messages for a single topic ensuring
 * that the duration and memory limits are respected by truncating
 * as needed on push() operations.
 */
class MessageQueue
{
  friend Snapshotter;

private:
  // Logger for outputting ROS logging messages
  rclcpp::Logger logger_;
  // Locks access to size_ and queue_
  std::mutex lock;
  // Stores limits on buffer size and duration
  SnapshotterTopicOptions options_;
  // Current total size of the queue, in bytes
  int64_t size_;
  typedef std::deque<SnapshotMessage> queue_t;
  queue_t queue_;
  // Subscriber to the callback which uses this queue
  std::shared_ptr<rclcpp::GenericSubscription> sub_;

public:
  explicit MessageQueue(const SnapshotterTopicOptions& options, const rclcpp::Logger& logger);
  // Add a new message to the internal queue if possible, truncating the front
  // of the queue as needed to enforce limits
  void push(const SnapshotMessage& msg);
  // Removes the message at the front of the queue (oldest) and returns it
  SnapshotMessage pop();
  // Returns the time difference between back and front of queue, or 0 if size <= 1
  rclcpp::Duration duration() const;
  // Clear internal buffer
  void clear();
  // Store the subscriber for this topic's queue internaly so it is not deleted
  void setSubscriber(std::shared_ptr<rclcpp::GenericSubscription> sub);
  typedef std::pair<queue_t::const_iterator, queue_t::const_iterator> range_t;
  // Get a begin and end iterator into the buffer respecting the start and
  // end timestamp constraints
  range_t rangeFromTimes(const rclcpp::Time& start, const rclcpp::Time& end);

  // Return the total message size including the meta-information
  int64_t getMessageSize(SnapshotMessage const& msg) const;

private:
  // Internal push whitch does not obtain lock
  void _push(SnapshotMessage const& msg);
  // Internal pop which does not obtain lock
  SnapshotMessage _pop();
  // Internal clear which does not obtain lock
  void _clear();
  // Truncate front of queue as needed to fit a new message of specified size and time.
  // Returns False if this is impossible.
  bool preparePush(int32_t size, rclcpp::Time const& time);
};

// MODIFIED : Add logic to cache any latest message from distinct topics without logic of tile and write them at the start of the given bag.
class SingleMessageCache
{
private:
  // std::unordered_map<std::string, SnapshotMessage> cache_;
  std::vector<SnapshotMessage> cache_; // Can't be stored in STL container like map.
  std::unordered_map<std::string, u_int> topic_lookup_index_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> subscribers_;
  std::unordered_map<std::string, std::string> topic_name_to_type_;
  std::mutex cache_mutex_;
  std::shared_ptr<rclcpp::Node> _node;
  void cacheLatestMessageOnly(std::shared_ptr<const rclcpp::SerializedMessage> msg, const heex_rosbag2_snapshot::TopicDetails& topic);

public:
  SingleMessageCache() = default;
  void set_node(std::shared_ptr<rclcpp::Node> node) { _node = node; };
  std::shared_ptr<rclcpp::Node> get_node() { return _node; };
  void subscribeToTopic(const heex_rosbag2_snapshot::TopicDetails& topic);
  SnapshotMessage getLatestMessage(const heex_rosbag2_snapshot::TopicDetails& topic);
  int getCachedTopicIndex(const heex_rosbag2_snapshot::TopicDetails& topic) const;
  void unsubscribeFromTopic(const heex_rosbag2_snapshot::TopicDetails& topic);
  bool writeToBagAllCachedMessagesAtTime(rosbag2_cpp::Writer& bag_writer);
};

// Snapshotter node. Maintains a circular buffer of the most recent messages
// from configured topics while enforcing limits on memory and duration.
// The node can be triggered to write some or all of these buffers to a bag
// file via a service call. Useful in live testing scenerios where interesting
// data may be produced before a user has the oppurtunity to "rosbag record" the data.
class Snapshotter : public rclcpp::Node
{
public:
  explicit Snapshotter(const rclcpp::NodeOptions& options);
  //MODIFIED: overloaded constructor by adding snap_options and optional node_name arguments for our purposes
  explicit Snapshotter(const rclcpp::NodeOptions& options, SnapshotterOptions& snap_options, std::string node_name);
  ~Snapshotter();
  // MODIFIED added the singleMessageCaches node getter
  std::shared_ptr<rclcpp::Node> get_single_msg_caches_node() { return singleMessageCaches_.get_node(); }

private:
  // Subscribe queue size for each topic
  static const int QUEUE_SIZE;
  SnapshotterOptions options_;
  typedef std::map<TopicDetails, std::shared_ptr<MessageQueue>> buffers_t;
  buffers_t buffers_;
  // Locks recording_ and writing_ states.
  std::shared_mutex state_lock_;
  // True if new messages are being written to the internal buffer
  bool recording_;
  // True if currently writing buffers to a bag file
  bool writing_;

  // MODIFIED: added singleMessageCaches_ and commented out server code
  SingleMessageCache singleMessageCaches_;
  // rclcpp::Service<rosbag2_snapshot_msgs::srv::TriggerSnapshot>::SharedPtr
  //   trigger_snapshot_server_;
  // rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_server_;
  rclcpp::TimerBase::SharedPtr poll_topic_timer_;

  // Convert parameter values into a SnapshotterOptions object
  void parseOptionsFromParams();
  // Replace individual topic limits with node defaults if they are
  // flagged for it (see SnapshotterTopicOptions)
  void fixTopicOptions(SnapshotterTopicOptions& options);
  // If file is "prefix" mode (doesn't end in .bag), append current datetime and .bag to end
  bool postfixFilename(std::string& file);
  /// Return current local datetime as a string such as 2018-05-22-14-28-51.
  // Used to generate bag filenames
  std::string timeAsStr();
  // Clear the internal buffers of all topics. Used when resuming after a pause to avoid time gaps
  void clear();
  // Subscribe to one of the topics, setting up the callback to add to the respective queue
  void subscribe(const TopicDetails& topic_details, std::shared_ptr<MessageQueue> queue);
  // Called on new message from any configured topic. Adds to queue for that topic
  void topicCb(std::shared_ptr<const rclcpp::SerializedMessage> msg, std::shared_ptr<MessageQueue> queue);
  // Service callback, write all of part of the internal buffers to a bag file
  // according to request parameters
public:
  // MODIFIED : ::SharedPtr to std::shared_ptr<>
  void triggerSnapshotCb(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<SnapshotRequest> req, std::shared_ptr<SnapshotResponse> res);

  // MODIFIED : added instantmessage callback function
  void InstantMessageCb(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<heex_rosbag2_snapshot::SnapshotInstantRequest> req,
      std::shared_ptr<heex_rosbag2_snapshot::SnapshotResponse> res);

  // MODIFIED: added function that returns a vector of all the snaphotters TopicDetails
  std::vector<heex_rosbag2_snapshot::TopicDetails> getTopics();

private:
  // Service callback, enable or disable recording (storing new messages into queue).
  // Used to pause before writing
  void enableCb(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool_Response::SharedPtr res);
  // Set recording_ to false and do nessesary cleaning, CALLER MUST OBTAIN LOCK
  void pause();
  // Set recording_ to true and do nesessary cleaning, CALLER MUST OBTAIN LOCK
  void resume();
  // Poll master for new topics
  void pollTopics();
  // Write the parts of message_queue within the time constraints of req to the queue
  // If returns false, there was an error opening/writing the bag and an error message
  // was written to res.message
  // MODIFIED: changed ::SharedPtr so std::shared_ptr<>, and TriggerSnapshot::Request to SnapshotRequest
  bool writeTopic(
      rosbag2_cpp::Writer& bag_writer,
      MessageQueue& message_queue,
      const TopicDetails& topic_details,
      const std::shared_ptr<heex_rosbag2_snapshot::SnapshotRequest> req,
      const std::shared_ptr<heex_rosbag2_snapshot::SnapshotResponse> res);
};

// Configuration for SnapshotterClient
// struct SnapshotterClientOptions
// {
//   SnapshotterClientOptions();
//   enum Action
//   {
//     TRIGGER_WRITE,
//     PAUSE,
//     RESUME
//   };
//   // What to do when SnapshotterClient.run is called
//   Action action_;
//   // List of topics to write when action_ == TRIGGER_WRITE.
//   // If empty, write all buffered topics.
//   std::vector<TopicDetails> topics_;
//   // Name of file to write to when action_ == TRIGGER_WRITE, relative to snapshot node.
//   // If empty, use prefix
//   std::string filename_;
//   // Prefix of the name of file written to when action_ == TRIGGER_WRITE.
//   std::string prefix_;
// };

// Node used to call services which interface with the snapshotter node to trigger
// write, pause, and resume
// class SnapshotterClient : public rclcpp::Node
// {
// public:
//   explicit SnapshotterClient(const rclcpp::NodeOptions & options);

// private:
//   void setSnapshotterClientOptions(SnapshotterClientOptions const & opts);
// };

} // namespace heex_rosbag2_snapshot

#endif // ROSBAG2_SNAPSHOT__SNAPSHOTTER_HPP_
