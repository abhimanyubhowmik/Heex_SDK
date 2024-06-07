/**
 * @file Snapshotter.cpp
 * @author Matthieu Carré (matthieu@heex.io)
 * @brief Header file defining the adapted ROS Snapshotter main tool class and related structures.
 * @version 0.1
 * @date 2022-06-07
 *
 * @copyright See license below. Modified and adapted by Heex Technologies for demonstration purposes of the Heex SDK.
 *
 */
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Open Source Robotics Foundation, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Open Source Robotics Foundation, Inc. nor the
*     names of its contributors may be used to endorse or promote products
*     derived from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/
#include "Snapshotter.h"

#include <ros/assert.h>
#include <ros/ros.h>
#include <time.h>
#include <topic_tools/shape_shifter.h>

#include <boost/date_time/local_time/local_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/scope_exit.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/xtime.hpp>
#include <queue>
#include <string>
#include <vector>

// #include <rosbag_snapshot_msgs/SnapshotStatus.h>
// #include <rosbag_snapshot/snapshotter.h>
#include <memory>
#include <utility>

using boost::shared_ptr;
using ros::Time;
using std::string;

namespace heex_rosbag_snapshot
{
const ros::Duration SnapshotterTopicOptions::NO_DURATION_LIMIT      = ros::Duration(-1);
const int32_t SnapshotterTopicOptions::NO_MEMORY_LIMIT              = -1;
const int32_t SnapshotterTopicOptions::NO_COUNT_LIMIT               = -1;
const ros::Duration SnapshotterTopicOptions::INHERIT_DURATION_LIMIT = ros::Duration(0);
const int32_t SnapshotterTopicOptions::INHERIT_MEMORY_LIMIT         = 0;
const int32_t SnapshotterTopicOptions::INHERIT_COUNT_LIMIT          = 0;

SnapshotterTopicOptions::SnapshotterTopicOptions(ros::Duration duration_limit, int32_t memory_limit, int32_t count_limit)
    : duration_limit_(duration_limit),
      memory_limit_(memory_limit),
      count_limit_(count_limit)
{
}

SnapshotterOptions::SnapshotterOptions(ros::Duration default_duration_limit, int32_t default_memory_limit, int32_t default_count_limit, ros::Duration status_period)
    : default_duration_limit_(default_duration_limit),
      default_memory_limit_(default_memory_limit),
      default_count_limit_(default_count_limit),
      status_period_(status_period),
      topics_()
{
}

bool SnapshotterOptions::addTopic(const std::string& topic, ros::Duration duration, int32_t memory, int32_t count)
{
  SnapshotterTopicOptions ops(duration, memory, count);
  std::pair<topics_t::iterator, bool> ret;
  ret = topics_.insert(topics_t::value_type(topic, ops));
  return ret.second;
}

// SnapshotterClientOptions::SnapshotterClientOptions() : action_(SnapshotterClientOptions::TRIGGER_WRITE)
// {
// }

SnapshotMessage::SnapshotMessage(topic_tools::ShapeShifter::ConstPtr _msg, boost::shared_ptr<ros::M_string> _connection_header, Time _time)
    : msg(_msg),
      connection_header(_connection_header),
      time(_time)
{
}

MessageQueue::MessageQueue(const SnapshotterTopicOptions& options) : options_(options), size_(0) {}

void MessageQueue::setSubscriber(shared_ptr<ros::Subscriber> sub)
{
  sub_ = sub;
}

void MessageQueue::fillStatus(heex_rosbag_snapshot::MessageQueueStatistic& status)
{
  //void MessageQueue::fillStatus(rosgraph_msgs::TopicStatistics& status)
  boost::mutex::scoped_lock l(lock);
  if (!queue_.size())
  {
    return;
  }
  status.traffic        = size_;
  status.delivered_msgs = queue_.size();
  status.window_start   = queue_.front().time;
  status.window_stop    = queue_.back().time;
}

void MessageQueue::clear()
{
  boost::mutex::scoped_lock l(lock);
  _clear();
}

void MessageQueue::_clear()
{
  queue_.clear();
  size_ = 0;
}

ros::Duration MessageQueue::duration() const
{
  // No duration if 0 or 1 messages
  if (queue_.size() <= 1)
  {
    return ros::Duration();
  }
  return queue_.back().time - queue_.front().time;
}

bool MessageQueue::preparePush(int32_t size, const ros::Time& time)
{
  // If new message is older than back of queue, time has gone backwards and buffer must be cleared
  if (!queue_.empty() && time < queue_.back().time)
  {
    ROS_WARN("Time has gone backwards. Clearing buffer for this topic.");
    _clear();
  }

  // The only case where message cannot be addded is if size is greater than limit
  if (options_.memory_limit_ > SnapshotterTopicOptions::NO_MEMORY_LIMIT && size > options_.memory_limit_)
  {
    return false;
  }

  // If memory limit is enforced, remove elements from front of queue until limit would be met once message is added
  if (options_.memory_limit_ > SnapshotterTopicOptions::NO_MEMORY_LIMIT)
  {
    while (queue_.size() != 0 && size_ + size > options_.memory_limit_)
    {
      _pop();
    }
  }

  // If duration limit is encforced, remove elements from front of queue until duration limit would be met once message
  // is added
  if (options_.duration_limit_ > SnapshotterTopicOptions::NO_DURATION_LIMIT && queue_.size() != 0)
  {
    ros::Duration dt = time - queue_.front().time;
    while (dt > options_.duration_limit_)
    {
      _pop();
      if (queue_.empty())
      {
        break;
      }
      dt = time - queue_.front().time;
    }
  }

  // If count limit is enforced, remove elements from front of queue until the the count is below the limit
  if (options_.count_limit_ > SnapshotterTopicOptions::NO_COUNT_LIMIT && queue_.size() != 0)
  {
    while (queue_.size() != 0 && queue_.size() >= options_.count_limit_)
    {
      _pop();
    }
  }

  return true;
}

void MessageQueue::push(const SnapshotMessage& _out)
{
  boost::mutex::scoped_try_lock l(lock);
  if (!l.owns_lock())
  {
    ROS_ERROR("Failed to lock. Time %f", _out.time.toSec());
    return;
  }
  _push(_out);
}

SnapshotMessage MessageQueue::pop()
{
  boost::mutex::scoped_lock l(lock);
  return _pop();
}

int64_t MessageQueue::getMessageSize(const SnapshotMessage& snapshot_msg) const
{
  return snapshot_msg.msg->size() + snapshot_msg.connection_header->size() + snapshot_msg.msg->getDataType().size() + snapshot_msg.msg->getMD5Sum().size() +
         snapshot_msg.msg->getMessageDefinition().size() + sizeof(SnapshotMessage);
}

void MessageQueue::_push(const SnapshotMessage& _out)
{
  int32_t size = _out.msg->size();
  // If message cannot be added without violating limits, it must be dropped
  if (!preparePush(size, _out.time))
  {
    return;
  }
  queue_.push_back(_out);
  // Add size of new message to running count to maintain correctness
  size_ += getMessageSize(_out);
}

SnapshotMessage MessageQueue::_pop()
{
  SnapshotMessage tmp = queue_.front();
  queue_.pop_front();
  //  Remove size of popped message to maintain correctness of size_
  size_ -= getMessageSize(tmp);
  return tmp;
}

MessageQueue::range_t MessageQueue::rangeFromTimes(const Time& start, const Time& stop)
{
  range_t::first_type begin = queue_.begin();
  range_t::second_type end  = queue_.end();

  // Increment / Decrement iterators until time contraints are met
  if (!start.isZero())
  {
    while (begin != end && (*begin).time < start)
    {
      ++begin;
    }
  }
  if (!stop.isZero())
  {
    while (end != begin && (*(end - 1)).time > stop)
    {
      --end;
    }
  }
  return range_t(begin, end);
}

const int Snapshotter::QUEUE_SIZE = 10;

Snapshotter::Snapshotter(const SnapshotterOptions& options) : options_(options), recording_(true), writing_(false)
{
  // status_pub_ = nh_.advertise<rosbag_snapshot_msgs::SnapshotStatus>("snapshot_status", 10);

  // MODIFIED: Initialize the subscriptions to all topic requiring a single message cache
  for (const std::string& topic : options.topics_once_at_bagstart_)
  {
    singleMessageCaches_.subscribeToTopic(topic);
  }
}

Snapshotter::~Snapshotter()
{
  // Each buffer contains a pointer to the subscriber and vice versa, so we need to
  // shutdown the subscriber to allow garbage collection to happen
  for (std::pair<const std::string, boost::shared_ptr<MessageQueue>>& buffer : buffers_)
  {
    buffer.second->sub_->shutdown();
  }
}

void Snapshotter::fixTopicOptions(SnapshotterTopicOptions& options)
{
  if (options.duration_limit_ == SnapshotterTopicOptions::INHERIT_DURATION_LIMIT)
  {
    options.duration_limit_ = options_.default_duration_limit_;
  }
  if (options.memory_limit_ == SnapshotterTopicOptions::INHERIT_MEMORY_LIMIT)
  {
    options.memory_limit_ = options_.default_memory_limit_;
  }
  if (options.count_limit_ == SnapshotterTopicOptions::INHERIT_COUNT_LIMIT)
  {
    options.count_limit_ = options_.default_memory_limit_;
  }
}

bool Snapshotter::postfixFilename(string& file)
{
  size_t ind = file.rfind(".bag");
  // If requested ends in .bag, this is literal name do not append date
  if (ind != string::npos && ind == file.size() - 4)
  {
    return true;
  }
  // Otherwise treat as prefix and append datetime and extension
  file += timeAsStr() + ".bag";
  return true;
}

string Snapshotter::timeAsStr()
{
  std::stringstream msg;
  const boost::posix_time::ptime now     = boost::posix_time::second_clock::local_time();
  boost::posix_time::time_facet* const f = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
  msg.imbue(std::locale(msg.getloc(), f));
  msg << now;
  return msg.str();
}

void Snapshotter::topicCB(const ros::MessageEvent<const topic_tools::ShapeShifter>& msg_event, boost::shared_ptr<MessageQueue> queue)
{
  // If recording is paused (or writing), exit
  {
    boost::shared_lock<boost::upgrade_mutex> lock(state_lock_);
    if (!recording_)
    {
      return;
    }
  }

  // Pack message and metadata into SnapshotMessage holder
  SnapshotMessage out(msg_event.getMessage(), msg_event.getConnectionHeaderPtr(), msg_event.getReceiptTime());
  queue->push(out);
}

void Snapshotter::subscribe(const string& topic, boost::shared_ptr<MessageQueue> queue)
{
  ROS_INFO("Subscribing to %s", topic.c_str());

  shared_ptr<ros::Subscriber> sub(boost::make_shared<ros::Subscriber>());
  ros::SubscribeOptions ops;
  ops.topic      = topic;
  ops.queue_size = QUEUE_SIZE;
  ops.md5sum     = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
  ops.datatype   = ros::message_traits::datatype<topic_tools::ShapeShifter>();
  ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<const ros::MessageEvent<const topic_tools::ShapeShifter>&>>(boost::bind(&Snapshotter::topicCB, this, _1, queue));
  *sub       = nh_.subscribe(ops);
  queue->setSubscriber(sub);
}

bool Snapshotter::writeTopic(rosbag::Bag& bag, MessageQueue& message_queue, const string& topic, SnapshotRequest& req, SnapshotResponse& res)
{
  // bool Snapshotter::writeTopic(rosbag::Bag& bag, MessageQueue& message_queue, string const& topic,
  //                         rosbag_snapshot_msgs::TriggerSnapshot::Request& req,
  //                         rosbag_snapshot_msgs::TriggerSnapshot::Response& res)
  // acquire lock for this queue
  boost::mutex::scoped_lock l(message_queue.lock);

  MessageQueue::range_t range = message_queue.rangeFromTimes(req.start_time, req.stop_time);

  // open bag if this the first valid topic and there is data
  if (!bag.isOpen() && range.second > range.first)
  {
    try
    {
      bag.open(req.filename, rosbag::bagmode::Write);
    }
    catch (const rosbag::BagException& err)
    {
      res.success = false;
      res.message = string("failed to open bag: ") + err.what();
      return false;
    }
    ROS_INFO("Writing snapshot to %s", req.filename.c_str());
  }

  // write queue
  try
  {
    for (MessageQueue::range_t::first_type msg_it = range.first; msg_it != range.second; ++msg_it)
    {
      const SnapshotMessage& msg = *msg_it;
      bag.write(topic, msg.time, msg.msg, msg.connection_header);
    }
  }
  catch (const rosbag::BagException& err)
  {
    res.success = false;
    res.message = string("failed to write bag: ") + err.what();
  }

  ROS_INFO_STREAM(
      "Snapshotter | Performed " << topic << " messages extraction for " << std::fixed << std::setprecision(10) << "[" << req.start_time << "; " << req.stop_time << "]");
  return true;
}
void Snapshotter::InstantMessageCb(SnapshotInstantRequest& req, SnapshotResponse& res)
{
  res.success = false;

  if (!req.topic.empty())
  {
    // Resolve and clean topic
    std::string topic = req.topic;
    try
    {
      topic = ros::names::resolve(nh_.getNamespace(), topic);
    }
    catch (const ros::InvalidNameException& err)
    {
      ROS_WARN("Requested topic %s is invalid, skipping.", topic.c_str());
      res.success = false;
      return;
    }

    // Find the message queue for this topic if it exsists
    buffers_t::iterator found = buffers_.find(topic);
    if (found == buffers_.end())
    {
      ROS_WARN("Requested topic %s is not subscribed, skipping.", topic.c_str());
      res.success = false;
      return;
    }
    MessageQueue& message_queue = *(*found).second;
    boost::mutex::scoped_lock l(message_queue.lock);

    // Time interval to look for the message
    Time start_time(req.query_time + req.search_offset);
    MessageQueue::range_t range = message_queue.rangeFromTimes(start_time, req.query_time); // Get last message, closest to timestamp

    if (range.first != range.second)
    {
      const SnapshotMessage& msg = *(prev(range.second));
      req.msg                    = msg.msg;
      req.msg_time               = msg.time;
      res.success                = true;
    }
    else
    {
      ROS_INFO_STREAM("Snapshot buffer is empty or does not contain data in the range timestamp requested");
      res.success = false;
    }
  }
}
bool Snapshotter::triggerSnapshotCb(SnapshotRequest& req, SnapshotResponse& res)
{
  // Wait until all data is within buffer range
  ros::Rate rate(2);              // ROS Rate at 2Hz
  ros::Duration two_seconds(2.0); // delay for all callback to receive data
  while ((req.stop_time + two_seconds) > ros::Time::now())
  {
    ROS_INFO_STREAM("Snapshotter | Waiting for extraction after " << std::fixed << std::setprecision(10) << req.stop_time);
    rate.sleep();
  }
  ROS_INFO_STREAM(
      "Snapshotter | All data are buffered. Extraction now will starts at " << std::fixed << std::setprecision(10) << ros::Time::now() << " for "
                                                                            << "[" << req.start_time << "; " << req.stop_time << "]");

  // bool Snapshotter::triggerSnapshotCb(rosbag_snapshot_msgs::TriggerSnapshot::Request& req,
  //                                   rosbag_snapshot_msgs::TriggerSnapshot::Response& res)
  if (!postfixFilename(req.filename))
  {
    res.success = false;
    res.message = "invalid";
    return true;
  }
  bool recording_prior; // Store if we were recording prior to write to restore this state after write
  {
    boost::upgrade_lock<boost::upgrade_mutex> read_lock(state_lock_);
    recording_prior = recording_;
    if (writing_)
    {
      res.success = false;
      res.message = "Already writing";
      return true;
    }
    boost::upgrade_to_unique_lock<boost::upgrade_mutex> write_lock(read_lock);
    if (recording_prior)
    {
      pause();
    }
    writing_ = true;
  }

  // Ensure that state is updated when function exits, regardlesss of branch path / exception events
  BOOST_SCOPE_EXIT(&state_lock_, &writing_, recording_prior, this_)
  {
    // Clear buffers beacuase time gaps (skipped messages) may have occured while paused
    boost::unique_lock<boost::upgrade_mutex> write_lock(state_lock_);
    // Turn off writing flag and return recording to its state before writing
    writing_ = false;
    if (recording_prior)
    {
      this_->resume();
    }
  }
  BOOST_SCOPE_EXIT_END

  // Create bag
  rosbag::Bag bag;

  // MODIFIED: Write first all messages from the singleMessageCaches to the start of the bag. Exit early if failed.
  if (!singleMessageCaches_.writeToBagAllCachedMessagesAtTime(bag, req, res))
  {
    return true;
  }

  // Write each selected topic's queue to bag file
  if (req.topics.size() && req.topics.at(0).size())
  {
    for (std::string& topic : req.topics)
    {
      // Resolve and clean topic
      try
      {
        topic = ros::names::resolve(nh_.getNamespace(), topic);
      }
      catch (const ros::InvalidNameException& err)
      {
        ROS_WARN("Requested topic %s is invalid, skipping.", topic.c_str());
        continue;
      }

      // Find the message queue for this topic if it exsists
      buffers_t::iterator found = buffers_.find(topic);
      // If topic not found, error and exit
      if (found == buffers_.end())
      {
        ROS_WARN("Requested topic %s is not subscribed, skipping.", topic.c_str());
        continue;
      }
      MessageQueue& message_queue = *(*found).second;
      if (!writeTopic(bag, message_queue, topic, req, res))
      {
        return true;
      }
    }
  }
  // If topic list empty, record all buffered topics
  else
  {
    for (const buffers_t::value_type& pair : buffers_)
    {
      MessageQueue& message_queue = *(pair.second);
      const std::string& topic    = pair.first;
      if (!writeTopic(bag, message_queue, topic, req, res))
      {
        return true;
      }
    }
  }

  // If no topics were subscribed/valid/contained data, this is considered a non-success
  if (!bag.isOpen())
  {
    res.success = false;
    res.message = res.NO_DATA_MESSAGE;
    return true;
  }

  res.success = true;
  return true;
}

void Snapshotter::clear()
{
  for (const buffers_t::value_type& pair : buffers_)
  {
    pair.second->clear();
  }
}

void Snapshotter::pause()
{
  ROS_INFO("Buffering paused");
  recording_ = false;
}

void Snapshotter::resume()
{
  clear();
  recording_ = true;
  ROS_INFO("Buffering resumed and old data cleared.");
}

// bool Snapshotter::enableCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
// {
//   boost::upgrade_lock<boost::upgrade_mutex> read_lock(state_lock_);
//   if (req.data && writing_)  // Cannot enable while writing
//   {
//     res.success = false;
//     res.message = "cannot enable recording while writing.";
//     return true;
//   }
//   // Obtain write lock and update state if requested state is different from current
//   if (req.data && !recording_)
//   {
//     boost::upgrade_to_unique_lock<boost::upgrade_mutex> write_lock(read_lock);
//     resume();
//   }
//   else if (!req.data && recording_)
//   {
//     boost::upgrade_to_unique_lock<boost::upgrade_mutex> write_lock(read_lock);
//     pause();
//   }
//   res.success = true;
//   return true;
// }

// void Snapshotter::publishStatus(ros::TimerEvent const& e)
// {
//   (void)e;  // Make your "unused variable" warnings a thing of the past with cast to void!
//   // Don't bother doing computing if no one is subscribed
//   if (!status_pub_.getNumSubscribers())
//     return;

//   // TODO(any): consider options to make this faster
//   // (caching and updating last status, having queues track their own status)
//   rosbag_snapshot_msgs::SnapshotStatus msg;
//   {
//     boost::shared_lock<boost::upgrade_mutex> lock(state_lock_);
//     msg.enabled = recording_;
//   }
//   std::string node_id = ros::this_node::getName();
//   for (const buffers_t::value_type& pair : buffers_)
//   {
//     rosgraph_msgs::TopicStatistics status;
//     status.node_sub = node_id;
//     status.topic = pair.first;
//     pair.second->fillStatus(status);
//     msg.topics.push_back(status);
//   }
//
//   status_pub_.publish(msg);
// }

void Snapshotter::pollTopics(const ros::TimerEvent& e, heex_rosbag_snapshot::SnapshotterOptions* options)
{
  (void)e;
  ros::master::V_TopicInfo topics;
  if (ros::master::getTopics(topics))
  {
    for (const ros::master::TopicInfo& t : topics)
    {
      std::string topic = t.name;
      if (options->addTopic(topic))
      {
        SnapshotterTopicOptions topic_options;
        fixTopicOptions(topic_options);
        shared_ptr<MessageQueue> queue;
        queue.reset(new MessageQueue(topic_options));
        std::pair<buffers_t::iterator, bool> res = buffers_.insert(buffers_t::value_type(topic, queue));
        ROS_ASSERT_MSG(res.second, "failed to add %s to topics. Perhaps it is a duplicate?", topic.c_str());
        subscribe(topic, queue);
      }
    }
  }
  else
  {
    ROS_WARN_THROTTLE(5, "Failed to get topics from the ROS master");
  }
}

int Snapshotter::run()
{
  if (!nh_.ok())
  {
    return 0;
  }

  // Create the queue for each topic and set up the subscriber to add to it on new messages
  for (SnapshotterOptions::topics_t::value_type& pair : options_.topics_)
  {
    string topic = ros::names::resolve(nh_.getNamespace(), pair.first);
    fixTopicOptions(pair.second);
    shared_ptr<MessageQueue> queue;
    queue.reset(new MessageQueue(pair.second));
    std::pair<buffers_t::iterator, bool> res = buffers_.insert(buffers_t::value_type(topic, queue));
    ROS_ASSERT_MSG(res.second, "failed to add %s to topics. Perhaps it is a duplicate?", topic.c_str());
    subscribe(topic, queue);
  }

  // // Now that subscriptions are setup, setup service servers for writing and pausing
  // trigger_snapshot_server_ = nh_.advertiseService("trigger_snapshot", &Snapshotter::triggerSnapshotCb, this);
  // enable_server_ = nh_.advertiseService("enable_snapshot", &Snapshotter::enableCB, this);

  // // Start timer to publish status regularly
  // if (options_.status_period_ > ros::Duration(0))
  //   status_timer_ = nh_.createTimer(options_.status_period_, &Snapshotter::publishStatus, this);

  // Start timer to poll ROS master for topics
  if (options_.all_topics_)
  {
    poll_topic_timer_ = nh_.createTimer(ros::Duration(1.0), boost::bind(&Snapshotter::pollTopics, this, _1, &options_));
  }

  // NOTE: Spin is performed by the caller. Thus, this section gets commented out by Heex
  // // Use multiple callback threads
  // ros::MultiThreadedSpinner spinner(4);  // Use 4 threads
  // spinner.spin();                        // spin() will not return until the node has been shutdown
  return 0;
}

// SnapshotterClient::SnapshotterClient()
// {
// }

// int SnapshotterClient::run(SnapshotterClientOptions const& opts)
// {
//   if (opts.action_ == SnapshotterClientOptions::TRIGGER_WRITE)
//   {
//     ros::ServiceClient client = nh_.serviceClient<rosbag_snapshot_msgs::TriggerSnapshot>("trigger_snapshot");
//     if (!client.exists())
//     {
//       ROS_ERROR("Service %s does not exist. Is snapshot running in this namespace?", "trigger_snapshot");
//       return 1;
//     }
//     rosbag_snapshot_msgs::TriggerSnapshotRequest req;
//     req.topics = opts.topics_;
//     // Prefix mode
//     if (opts.filename_.empty())
//     {
//       req.filename = opts.prefix_;
//       size_t ind = req.filename.rfind(".bag");
//       if (ind != string::npos && ind == req.filename.size() - 4)
//         req.filename.erase(ind);
//     }
//     else
//     {
//       req.filename = opts.filename_;
//       size_t ind = req.filename.rfind(".bag");
//       if (ind == string::npos || ind != req.filename.size() - 4)
//         req.filename += ".bag";
//     }
//
//     // Resolve filename relative to clients working directory to avoid confusion
//     if (req.filename.empty())  // Special case of no specified file, ensure still in working directory of client
//       req.filename = "./";
//     boost::filesystem::path p(boost::filesystem::system_complete(req.filename));
//     req.filename = p.string();
//
//     rosbag_snapshot_msgs::TriggerSnapshotResponse res;
//     if (!client.call(req, res))
//     {
//       ROS_ERROR("Failed to call service");
//       return 1;
//     }
//     if (!res.success)
//     {
//       ROS_ERROR("%s", res.message.c_str());
//       return 1;
//     }
//     return 0;
//   }
//   else if (opts.action_ == SnapshotterClientOptions::PAUSE || opts.action_ == SnapshotterClientOptions::RESUME)
//   {
//     ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("enable_snapshot");
//     if (!client.exists())
//     {
//       ROS_ERROR("Service %s does not exist. Is snapshot running in this namespace?", "enable_snapshot");
//       return 1;
//     }
//     std_srvs::SetBoolRequest req;
//     req.data = (opts.action_ == SnapshotterClientOptions::RESUME);
//     std_srvs::SetBoolResponse res;
//     if (!client.call(req, res))
//     {
//       ROS_ERROR("Failed to call service.");
//       return 1;
//     }
//     if (!res.success)
//     {
//       ROS_ERROR("%s", res.message.c_str());
//       return 1;
//     }
//     return 0;
//   }
//   else
//   {
//     ROS_ASSERT_MSG(false, "Invalid value of enum.");
//     return 1;
//   }
// }

void SingleMessageCache::cacheLatestMessageOnly(const ros::MessageEvent<const topic_tools::ShapeShifter>& msg_event, const std::string& topic)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  int index = this->getCachedTopicIndex(topic);
  if (index == -1)
  {
    // Create new entry in the cache and reference the new index
    cache_.push_back(SnapshotMessage(msg_event.getMessage(), msg_event.getConnectionHeaderPtr(), msg_event.getReceiptTime()));
    topic_lookup_index_[topic] = cache_.size() - 1;
    return;
  }

  // Replace existing message. Only latest message is kept.
  cache_[index] = SnapshotMessage(msg_event.getMessage(), msg_event.getConnectionHeaderPtr(), msg_event.getReceiptTime());
}

void SingleMessageCache::subscribeToTopic(const std::string& topic_name)
{
  if (subscribers_.find(topic_name) != subscribers_.end())
  {
    return; // Guard condition
  }

  ros::NodeHandle nh;

  shared_ptr<ros::Subscriber> sub(boost::make_shared<ros::Subscriber>());
  ros::SubscribeOptions ops;
  ops.topic      = topic_name;
  ops.queue_size = 3; // Set to a low value as want to only keep the latest. We can and want to throw away the oldest ones.
  ops.md5sum     = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
  ops.datatype   = ros::message_traits::datatype<topic_tools::ShapeShifter>();
  ops.helper     = boost::make_shared<ros::SubscriptionCallbackHelperT<const ros::MessageEvent<const topic_tools::ShapeShifter>&>>(
      boost::bind(&SingleMessageCache::cacheLatestMessageOnly, this, _1, topic_name));
  *sub                     = nh.subscribe(ops);
  subscribers_[topic_name] = sub;
  // We do not create cache entry here. Cache and its lookup index only gets created through the callback.
}

int SingleMessageCache::getCachedTopicIndex(const std::string& topic_name) const
{
  auto it = topic_lookup_index_.find(topic_name);
  if (it != topic_lookup_index_.end())
  {
    return it->second;
  }
  return -1;
}

void SingleMessageCache::unsubscribeFromTopic(const std::string& topic_name)
{
  // Clean subscriber first
  if (subscribers_.find(topic_name) == subscribers_.end())
  {
    return; // Guard condition
  }

  subscribers_[topic_name]->shutdown();
  subscribers_.erase(topic_name);

  // Clean cache and topic index
  int indexToDelete = this->getCachedTopicIndex(topic_name);
  if (indexToDelete == -1)
  {
    return; // Guard condition
  }

  topic_lookup_index_.erase(topic_name);
  if (indexToDelete >= 0 && indexToDelete < cache_.size())
  {
    cache_.erase(cache_.begin() + indexToDelete);
  }
}

bool SingleMessageCache::writeToBagAllCachedMessagesAtTime(rosbag::Bag& bag, SnapshotRequest& req, SnapshotResponse& res)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);

  // Check that bag is open:
  // open bag if this the first valid topic and there is data
  if (!bag.isOpen() && cache_.size() > 0)
  {
    try
    {
      bag.open(req.filename, rosbag::bagmode::Write);
    }
    catch (const rosbag::BagException& err)
    {
      res.success = false;
      res.message = string("failed to open bag: ") + err.what();
      return false;
    }
    ROS_INFO("Writing snapshot to %s", req.filename.c_str());
  }

  for (const auto& topic_pair : topic_lookup_index_)
  {
    const std::string& topic   = topic_pair.first;
    const SnapshotMessage& msg = cache_[topic_pair.second];
    // We are using ShapeShifter, which provides a convenient way
    // to write messages of unknown type to a bag.
    bag.write(topic, req.start_time, msg.msg, msg.connection_header);
  }
  return true;
}

} // namespace heex_rosbag_snapshot
