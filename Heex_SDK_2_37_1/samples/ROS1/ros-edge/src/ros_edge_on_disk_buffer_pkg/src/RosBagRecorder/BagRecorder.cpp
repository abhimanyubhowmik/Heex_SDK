/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

/// @brief source code of class bag recorder.
/// Heavily inspired from
/// http://docs.ros.org/en/diamondback/api/rosbag/html/c++/classrosbag_1_1Recorder.html
/// Simplified and adapted for this use case

#include <boost/bind/bind.hpp>
using namespace boost::placeholders;
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <thread>

#include "BagRecorder.h"
#include "HeexUtilsLog.h"

Heex::BagRecorder::BagRecorder()
{
  // ensure that the system is in a valid state
  if (!_nh.ok())
  {
    HEEX_LOG(error) << "BagRecorder::BagRecorder| ros handle is not valid";
    return;
  }
  if (!ros::Time::waitForValid(ros::WallDuration(2.0)))
  {
    HEEX_LOG(error) << "BagRecorder::BagRecorder| did not get a valid time";
    return;
  }
}
const int Heex::BagRecorder::QUEUE_SIZE = 10000; // Can be adjust depending on the ram available

Heex::BagRecorder::~BagRecorder() {}

bool Heex::BagRecorder::record(std::string recordIntervalEnd, const std::string& queryTimestamp, std::string& bagPath, std::vector<std::string> topics)
{
  if (!_nh.ok())
  {
    HEEX_LOG(error) << "BagRecorder::BagRecorder| ros handle is not valid";
    return false;
  }

  float intervalEnd = atof(recordIntervalEnd.c_str());
  if (intervalEnd <= 0)
  {
    HEEX_LOG(warning) << "BagRecorder::record| bag will be emtpy as recordLength is empty";
    return false;
  }

  // use time here instead of query.timestamp as it is the real time where
  // recording is started
  ros::Time eventTime = ros::Time::fromBoost(boost::posix_time::from_iso_extended_string(queryTimestamp));

  // start recording data
  if (topics.empty())
  {
    subcribeToAllTopics();
  }
  else
  {
    for (const std::string& topic : topics)
    {
      addTopic(topic);
    }
  }

  if (_subscribedTopics.empty())
  {
    HEEX_LOG(warning) << "BagRecorder::record| topics list is empty";
    return false;
  }
  // Wait for recordLength seconds
  // Wait until all data is within buffer range
  ros::Duration te(intervalEnd);

  HEEX_LOG(info) << "BagRecorder::BagRecorder | Extraction now will starts now " << std::fixed << std::setprecision(10) << ros::Time::now().toSec() << " for recordIntervalEnd "
                 << recordIntervalEnd << "s";
  ros::Rate rate(2.0); // ROS Rate at 2Hz
  while ((eventTime + te) > ros::Time::now())
  {
    HEEX_LOG(info) << "BagRecorder::record| Waiting for extraction after";
    rate.sleep();
  }

  // stop all callbacks
  this->stopRecording();

  std::string bagFileName             = "rosbag_" + queryTimestamp + ".bag";
  boost::filesystem::path bagFullPath = boost::filesystem::temp_directory_path() / "RosBags" / bagFileName;

  if (!isDiskSpaceEnough(bagFullPath))
  {
    return false;
  }

  if (_bagMessagesQueue.empty())
  {
    HEEX_LOG(info) << "BagRecorder::record| queue is empty";
    return false;
  }

  rosbag::Bag bag;
  try
  {
    bag.open(bagFullPath.string(), rosbag::bagmode::Write);
  }
  catch (const std::exception& e)
  {
    HEEX_LOG(error) << "BagRecorder::record| Error when opening the bag caused by" << e.what();
    return false;
  }

  // write all data in queue in the bag
  // There is not callbacks updating the queue, so not lock is required
  while (!_bagMessagesQueue.empty())
  {
    BagMessage message = _bagMessagesQueue.front();
    _bagMessagesQueue.pop();
    bag.write(message.topic, message.time, message.message, message.connectionHeader);
  }
  bag.close();
  bagPath = bagFullPath.string();
  return true;
}

void Heex::BagRecorder::addTopic(const std::string& topic)
{
  if (_subscribedTopics.find(topic) == _subscribedTopics.end())
  {
    std::string cleanTopic;
    // Resolve and clean topic
    try
    {
      cleanTopic = ros::names::resolve(_nh.getNamespace(), topic);
      subscribeToTopic(cleanTopic);
    }
    catch (ros::InvalidNameException const& err)
    {
      HEEX_LOG(warning) << "Requested topic %s is invalid, skipping." << topic.c_str();
    }
  }
}

void Heex::BagRecorder::subscribeToTopic(const std::string& topic)
{
  boost::shared_ptr<int> count(boost::make_shared<int>(0));
  boost::shared_ptr<ros::Subscriber> sub(boost::make_shared<ros::Subscriber>());
  ros::SubscribeOptions ops;
  ops.topic      = topic;
  ops.queue_size = 100; // Internal message queue before older messages are
                        // dropped if not collected
  ops.md5sum     = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
  ops.datatype   = ros::message_traits::datatype<topic_tools::ShapeShifter>();
  ops.helper     = boost::make_shared<ros::SubscriptionCallbackHelperT<const ros::MessageEvent<topic_tools::ShapeShifter const>&>>(
      boost::bind(&Heex::BagRecorder::subcriberCallback, this, _1, topic, sub, count));

  *sub = _nh.subscribe(ops);
  if (sub)
  {
    _subscribedTopics.insert(topic);
    _subscribers.push_back(sub);
  }
}

void Heex::BagRecorder::subcribeToAllTopics()
{
  ros::master::V_TopicInfo topics;
  if (ros::master::getTopics(topics))
  {
    for (const ros::master::TopicInfo& t : topics)
    {
      addTopic(t.name);
    }
  }
}

void Heex::BagRecorder::subcriberCallback(
    const ros::MessageEvent<topic_tools::ShapeShifter const>& event,
    const std::string& topic,
    boost::shared_ptr<ros::Subscriber> sub,
    boost::shared_ptr<int> count)
{
  // These do nothing, but fill the template requirements
  (void)sub;
  (void)count;
  ros::Time receivedTime = ros::Time::now();
  BagMessage message(topic, event.getMessage(), event.getConnectionHeaderPtr(), receivedTime);
  {
    const std::lock_guard<std::mutex> lock(_bagMessageQueueMutex);
    while (_bagMessagesQueue.size() >= QUEUE_SIZE)
    {
      _bagMessagesQueue.pop();
    }
    _bagMessagesQueue.push(message);
  }
}

void Heex::BagRecorder::stopRecording()
{
  for (const boost::shared_ptr<ros::Subscriber>& subscriber : _subscribers)
  {
    subscriber->shutdown();
  }
}

bool Heex::BagRecorder::isDiskSpaceEnough(const boost::filesystem::path& bagFullPath)
{
  // minimun space is 1G
  unsigned long long minRecordingSpace = 1024 * 1024 * 1024;
  boost::filesystem::path parentPath   = bagFullPath.parent_path();
  boost::filesystem::create_directory(parentPath);

  boost::system::error_code ec;
  boost::filesystem::create_directory(parentPath, ec);
  if (ec.value() != boost::system::errc::success && ec.value() != boost::system::errc::file_exists)
  {
    HEEX_LOG(error) << "BagRecorder::isDiskSpaceEnough | Can't create folder " << parentPath.string() + ". Filesystem error caused by: " + ec.message();
    return false;
  }
  boost::filesystem::space_info spaceInfo;
  try
  {
    spaceInfo = boost::filesystem::space(parentPath);
  }
  catch (const std::exception& e)
  {
    HEEX_LOG(error) << "BagRecorder::isDiskSpaceEnough| Error when checking "
                       "disk space caused by"
                    << e.what();
    return false;
  }

  if (spaceInfo.available < minRecordingSpace)
  {
    HEEX_LOG(error) << "BagRecorder::isDiskSpaceEnough| Not enough disk space available";
    return false;
  }
  return true;
}
