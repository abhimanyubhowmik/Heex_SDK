/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include <boost/bind/bind.hpp>
using namespace boost::placeholders;
// Type(s) of message(s) the Recorder shall use (required when deserialized to
// read data)
#include <sensor_msgs/NavSatFix.h>

#include "RosBagRecorder.h"

RosBagRecorder::RosBagRecorder(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort) : RecorderInThePast(uuid, serverIp, serverPort)
{
  // Read options
  _topics       = {"/gps/fix", "/demo/bool", "/imu"}; // Hard-coded topic names available in the example bag.
  _cvSubscriber = _nh.subscribe("/gps/fix", 100, &RosBagRecorder::contextValueCallback, this);
}

RosBagRecorder::~RosBagRecorder() {}

// Queue size for context values. Shall be adjusted depending on the rate of
// incoming messages and ram available
const int RosBagRecorder::QUEUE_SIZE = 100;

void RosBagRecorder::contextValueCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(_cvMutex);
  ros::Time receivedTime = ros::Time::now();
  while (_contextValuesQueue.size() >= QUEUE_SIZE)
  {
    (void)_contextValuesQueue.pop();
  }
  PositionMessage message(msg, receivedTime);
  _contextValuesQueue.push(message);
}

bool RosBagRecorder::extractContextValue(const Heex::RecorderArgs::RecorderContextValueArgs& query, sensor_msgs::NavSatFix::ConstPtr& msg, ros::Time& time)
{
  std::queue<PositionMessage> contextValuesQueue;
  {
    // Get a copy of the current queue
    const std::lock_guard<std::mutex> lock(_cvMutex);
    contextValuesQueue = _contextValuesQueue;
  }
  if (contextValuesQueue.empty())
  {
    return false;
  }

  // Find the message with the closest timestamp to the query timestamp
  sensor_msgs::NavSatFix::ConstPtr closestMsg;
  ros::Time closestTime;
  ros::Time event_time = ros::Time::fromBoost(boost::posix_time::from_iso_extended_string(query.timestamp));

  while (!contextValuesQueue.empty() && contextValuesQueue.front().time <= event_time)
  {
    closestMsg  = contextValuesQueue.front().msg;
    closestTime = contextValuesQueue.front().time;
    contextValuesQueue.pop();
  }
  msg  = closestMsg;
  time = closestTime;
  return true;
}

bool RosBagRecorder::generateRequestedValues(const Heex::RecorderArgs::RecorderContextValueArgs& query, std::vector<Heex::RecorderArgs::ContextValue>& contextValues)
{
  sensor_msgs::NavSatFix::ConstPtr posMsgPtr;
  ros::Time msgTime;
  bool ret = extractContextValue(query, posMsgPtr, msgTime);
  if (ret)
  {
    std::vector<std::string> keys = this->getContextValueKeys(query);
    HEEX_LOG(info) << "RosBagRecorder | Number of requested ContextValues for "
                      "the SampleRecorder: "
                   << keys.size() << std::endl;

    for (std::string& key : keys)
    {
      if (key == "position")
      {
        HEEX_LOG(info) << "Position extracted from RosBag : " << posMsgPtr->latitude << ", " << posMsgPtr->longitude << '\n';
        // Add to the value to the specified "position" key
        bool success = this->addGNSSContextValue(contextValues, key, posMsgPtr->latitude, posMsgPtr->longitude);
        if (success == false)
        {
          return false;
        }
      }
      if (key == "bag_timestamp")
      {
        // Add to the value to the specified "bag_timestamp" key
        std::string ts = boost::posix_time::to_iso_extended_string(msgTime.toBoost());
        HEEX_LOG(info) << "Timestamp extracted from RosBag : " << ts << '\n';
        bool success = this->addContextValue(contextValues, key, ts);
        if (success == false)
        {
          return false;
        }
      }
    }

    return true;
  }
  return false;
}

bool RosBagRecorder::generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath)
{
  Heex::BagRecorder recorder;
  bool ret = recorder.record(query.recordIntervalEnd, query.timestamp, filepath, _topics);
  if (!ret)
  {
    HEEX_LOG(error) << "RosBagRecorder::generateRequestedFilePaths | Error "
                       "while recording bag";
  }
  return ret;
}

void RosBagRecorder::awaitReady()
{
  while (this->isReady() == false && ros::ok())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
