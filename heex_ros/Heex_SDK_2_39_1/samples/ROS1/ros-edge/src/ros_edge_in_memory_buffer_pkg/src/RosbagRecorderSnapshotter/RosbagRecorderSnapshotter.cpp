///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include <boost/bind/bind.hpp>
using namespace boost::placeholders;
#include <boost/filesystem.hpp>
#include <iostream>

// Rosbag and time related function calls
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#define BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <chrono>
#include <thread>
// ROS1 Api to enable manipulation of any type message without deserializing
#include <topic_tools/shape_shifter.h>

// Type(s) of message(s) the Recorder shall use (required when deserialized to read data)
#include <sensor_msgs/NavSatFix.h>

#include "RosbagRecorderSnapshotter.h"

RosbagRecorderSnapshotter::RosbagRecorderSnapshotter(ros::NodeHandle* nh, const std::string& serverIp, const unsigned int& serverPort)
    : Recorder("R-a5d42923-d4e8-4eb3-8f58-a19ee0b0370b(1.0.0)", serverIp, serverPort)
{
  // Set the different topics to record
  _topics                  = {"/gps/fix", "/demo/bool", "/imu"}; // Hard-coded topic names available in the example bag.
  _topicNameGps            = "/gps/fix";                         // Hard-code of the topic name containing GPS data (expecting msg of type sensor_msgs/NavSatFix.h)
  /// Hard-coded topic names whose latest message will be written at start of the extracted bag.
  _topics_once_at_bagstart = {"/tf_static"};

  // Configure Snapshotter with the topics to snapshot
  heex_rosbag_snapshot::SnapshotterOptions options(ros::Duration(15));
  /// 1 : Select the list of topics to snapshot over the period of time.
  /// Choose one:
  /// 1.1 Set the _all_topics to `false` if you want to record only topics with in _topics list.
  /// 1.2 Set the _all_topics to `true` if you want to record all available topics. Topics in _topics won't be considered.
  _all_topics = false;
  if (!_all_topics)
  {
    // Configure and add all topics in _topics list with the default limit topic option
    options.all_topics_ = _all_topics;
    heex_rosbag_snapshot::SnapshotterTopicOptions topicsOptions;
    for (std::string topic : _topics)
    {
      options.topics_[topic] = topicsOptions;
    }
  }
  else
  {
    // Since we use the _all_topics and subscribe to all available topics, _topics is not used so we can clear it
    options.all_topics_ = _all_topics;
    _topics.clear();
  }
  /// 2 : Assign the list of topics to snapshot once and write only once at the start of any extracted bag.
  options.topics_once_at_bagstart_ = _topics_once_at_bagstart;

  _snap       = new heex_rosbag_snapshot::Snapshotter(options);
  _snapThread = new boost::thread(boost::bind(&RosbagRecorderSnapshotter::runCyclingBuffer, this));
}

RosbagRecorderSnapshotter::~RosbagRecorderSnapshotter()
{
  delete _snap;
  _snap = NULL;

  _snapThread->join();
}

void RosbagRecorderSnapshotter::runCyclingBuffer()
{
  _snap->run();
}

bool RosbagRecorderSnapshotter::generateRequestedValues(const Heex::RecorderArgs::RecorderContextValueArgs& query, std::vector<Heex::RecorderArgs::ContextValue>& contextValues)
{
  // Prepare SnapshotRequest content
  ros::Time event_time = ros::Time::fromBoost(boost::posix_time::from_iso_extended_string(query.timestamp));
  ros::Time query_time(event_time);
  ros::Time msg_time;
  ros::Duration search_offset(-2); // This paramater shall vary according to the frequency of message publication

  heex_rosbag_snapshot::SnapshotInstantRequest req{{}, query_time, msg_time, search_offset, _topicNameGps};
  heex_rosbag_snapshot::SnapshotResponse res{false, std::string()};

  HEEX_LOG(info) << "RosbagRecorderSnapshotter::generateRequestedValues | Extracting values at timestamp " << std::fixed << std::setprecision(10) << query_time;
  // Call to Snapshotter with the request element to extract smart data from the cycling buffer
  _snap->InstantMessageCb(req, res); // Return when all past data have been written

  if (res.success == true)
  {
    try
    {
      sensor_msgs::NavSatFix::Ptr posMsgPtr = req.msg.get()->instantiate<sensor_msgs::NavSatFix>();

      if (posMsgPtr != nullptr)
      {
        std::vector<std::string> keys = this->getContextValueKeys(query);
        HEEX_LOG(info) << "SampleRecorder | Number of requested ContextValues for the SampleRecorder: " << keys.size() << std::endl;

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
            std::string ts = boost::posix_time::to_iso_extended_string(req.msg_time.toBoost());
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
      else
      {
        return false;
      }
    }
    catch (topic_tools::ShapeShifterException& err)
    {
      std::string log{err.what()};
      HEEX_LOG(info) << "Log exception : " << log << '\n';
      ROS_WARN("Empty msg return from Snapshotter");
      return false;
    }
  }
  else
  {
    return false;
  }

  return false;
}

bool RosbagRecorderSnapshotter::generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath)
{
  // Pre-compute variables
  ros::Duration ts(atof(query.recordIntervalStart.c_str()));
  ros::Duration te(atof(query.recordIntervalEnd.c_str()));

  // Prepare SnapshotRequest content
  ros::Time event_time = ros::Time::fromBoost(boost::posix_time::from_iso_extended_string(query.timestamp));
  ros::Time start_time(event_time + ts);
  ros::Time stop_time(event_time + te);
  std::string filename = "/tmp/recording_" + query.eventUuid + "_" + query.uuid + ".bag";

  heex_rosbag_snapshot::SnapshotRequest req{start_time, stop_time, filename, _all_topics ? std::vector<std::string>{} : _topics, start_time};
  heex_rosbag_snapshot::SnapshotResponse res{false, std::string()};
  ROS_INFO_STREAM(
      "RosbagRecorderSnapshotter | Requesting messages extraction for " << std::fixed << std::setprecision(10) << "[" << req.start_time << "; " << req.stop_time << "]");

  // Call to Snapshotter with the request element to extract smart data from the cycling buffer
  _snap->triggerSnapshotCb(req, res); // Return when all past data have been written

  // Temporary sleeping time to handle and wait for "future" messages to be written in the bag
  // TODO: Replace this sleep by a call to _snap->isRecordingBagReady(filename) (blocks, has timeout) in future updates
  // std::this_thread::sleep_for(std::chrono::seconds(2 * atoi(query.recordIntervalEnd.c_str()))); // Force to wait enough time for the bag to be written

  // Manage request result with Heex recorder
  if (res.success)
  {
    filepath = filename;
    return true;
  }
  // Error state
  std::cerr << "[ERR] RosbagRecorderSnapshotter::generateRequestedFilePaths | An error has occurred: " << res.message << std::endl;
  return false;
}

void RosbagRecorderSnapshotter::awaitReady()
{
  while (this->isReady() == false && ros::ok())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
