///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file RecorderRosbagDataLakeExtractor.cpp
/// @author Matthieu Carré (matthieu@heex.io)
/// @brief Source file for the Recorder that implements the extraction services for the HeexSDK. It implements the rosbag offline extraction vith position value as ContextValues and path to the extracted bagfile as EventRecordingPart.
/// @version 0.1
/// @date 2022-04-21
#include "RecorderRosbagDataLakeExtractor.h"

#include <boost/filesystem.hpp>
#include <iostream>

// Rosbag and time related function calls
#define BOOST_BIND_GLOBAL_PLACEHOLDERS // remove warning on the Bind placeholders for ROS
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#define BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// Message headers (standard)
#include <sensor_msgs/NavSatFix.h>

RecorderRosbagDataLakeExtractor::RecorderRosbagDataLakeExtractor(
    const std::string& uuid,
    const std::string& serverIp,
    const unsigned int& serverPort,
    const std::string& recorderInputBag,
    std::string& recorderOutputDir,
    std::string& recorderPositionTopic,
    std::vector<std::string> topics_tf)
    : Recorder(uuid, serverIp, serverPort),
      _recorderInputBag(recorderInputBag),
      _recorderOutputDir(recorderOutputDir),
      _recorderPositionTopic(recorderPositionTopic),
      _topics_tf(topics_tf)
{
  // INFO
  std::cout << "[INFO] RecorderRosbagDataLakeExtractor | Cutting from " << _recorderInputBag << std::endl;
  std::cout << "[INFO] RecorderRosbagDataLakeExtractor | Writing recordings in " << _recorderOutputDir << std::endl;
}

bool RecorderRosbagDataLakeExtractor::generateRequestedValues(
    const Heex::RecorderArgs::RecorderContextValueArgs& query,
    std::vector<Heex::RecorderArgs::ContextValue>& contextValues)
{
  // TODO: Uncomment and fill the variable `contextValues` with all the values for each of the ContextValue keys in the Recorder query. We recommend to use the addContextValue method. The keys for a Recorder are listed in the Recorder implementation panel.
  // Use this->getContextValueKeys(contextValues) to obtain all required keys from the query (optional check).
  // Use this->addContextValue(contextValues, key, value) to assign the value to the key. Values are defaulted to an empty string.
  // Use this->addGNSSContextValue(contextValues, "position", latitude, longitude) to assign separate latitude and longitude to the key "position".
  // Access query info for Recorder uuid (query.uuid), Event uuid (query.eventUuid) and requested timestamp (query.timestamp). See RecorderContextValueArgs structure.

  std::string positionKey   = "position";
  std::string positionValue = this->getPositionValueFromFile(query);
  bool res                  = this->addContextValue(contextValues, positionKey, positionValue);

  // Return true when all the context keys have their values added in `contextValues`.
  return res;
}

std::string RecorderRosbagDataLakeExtractor::getPositionValueFromFile(const Heex::RecorderArgs::RecorderContextValueArgs& query)
{
  // Open the bag and set where to find
  rosbag::Bag datalakeBag(this->_recorderInputBag, rosbag::bagmode::Read);
  rosbag::View viewQuery(true);
  std::vector<std::string> topics; // Topics (Select what we want to keep)
  topics.push_back(_recorderPositionTopic);

  // Select part of bag where best candidates can be
  ros::Time event_time = ros::Time::fromBoost(boost::posix_time::from_iso_extended_string(query.timestamp));
  ros::Duration ti(3.0); // Windows range to find best candidate to leverage message frequency and latency.
  /// We want the score at the nearest timestamp. We a bit ahead of the timestamp
  ros::Time query_begin_time(event_time - ti);
  ros::Time query_end_time(event_time); // We don't look after as it would not reflect the value at this time

  viewQuery.addQuery(datalakeBag, rosbag::TopicQuery(topics), query_begin_time, query_end_time);

  // Select best candidate
  ros::Time computeTime;           // Cache value to store computeTime for the current message
  ros::Duration deltaTimeResult;   // Cache value to store delta time value for the current message
  ros::Duration deltaTimeAbsolute; // Cache value to store absolute value of delta time
  /// Initialize minimum value for candidate with a max that aimed to be replaced by first iteration
  ros::Duration deltaTimeAbsoluteMin(10);
  std::stringstream candidateContextValue; // Cache the ContextValue of the best candidate
  candidateContextValue << std::fixed;     // Prevent scientific notation for the time values

  BOOST_FOREACH (rosbag::MessageInstance const m, viewQuery)
  {
    sensor_msgs::NavSatFix::ConstPtr p = m.instantiate<sensor_msgs::NavSatFix>();
    if (p != NULL)
    {
      // DEBUG
      std::cout << m.getTime() << " / " << p->header.stamp << " / " << p->header.seq << " [" << p->latitude << "," << p->longitude << "]" << std::endl;

      // Get time from NavSatFix header rather than message timestamp
      computeTime     = p->header.stamp; // Alternative would be to use: // m.getTime();
      deltaTimeResult = event_time - computeTime;

      // Get absolute value between reference and candidate. Handle case when Delta is negative.
      if (deltaTimeResult.toSec() < 0)
      {
        deltaTimeAbsolute = -deltaTimeResult;
      }
      else
      {
        deltaTimeAbsolute = deltaTimeResult;
      }

      // Determine min
      if (deltaTimeAbsolute < deltaTimeAbsoluteMin)
      {
        candidateContextValue.str(std::string());
        candidateContextValue << p->latitude << "," << p->longitude;
        deltaTimeAbsoluteMin = deltaTimeAbsolute;
      }
    }
  }
  datalakeBag.close();

  // Set best candidate
  if (candidateContextValue.rdbuf()->in_avail() == 0) // Check if empty
  {
    std::cerr << "RecorderRosbagDataLakeExtractor::getPositionValueFromFile | No candidate found for timestamp " << event_time << std::endl;
    return std::string(); // Returns an empty string
  }

  return candidateContextValue.str();
}

bool RecorderRosbagDataLakeExtractor::generateRequestedFilePaths(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query, std::string& filepath)
{
  // TODO: Uncomment and fill the `filepath` variable with your filepath value pointing to your generated event recording part.
  // Access query info for Recorder uuid (query.uuid), Event uuid (query.eventUuid) and requested timestamp (query.timestamp). See RecorderContextValueArgs structure.
  // You can provided a filepath with the desired option. Each of the options has a specific syntax to match a use case:
  //  - 1: Send the file only. Return the direct path to file (no modification required).
  //  - 2: Send the folder and all its content. Return the direct path to folder (no modification required, all folder content will be included).
  //  - 3: Send only the content of the folder. Return the path to the folder with "/*" as an extra suffix (wildcard at the end to include only files within the parent directory).
  // Access query info for Recorder uuid (query.uuid), Event uuid (query.eventUuid), requested timestamp (query.timestamp), and the intervals of time before and after the timestamp (query.recordIntervalStart and query.recordIntervalEnd). See RecorderEventRecordingPartArgs structure.

  filepath = this->executeLakeExtractorRosbagAPI(query);

  // Make path absolute (base is /) and canonical (remove any .. or .)
  boost::filesystem::path bpath(filepath);
  bpath    = boost::filesystem::system_complete(bpath);
  bpath    = boost::filesystem::canonical(bpath);
  filepath = bpath.string();

  // Return true when the path to the event recording part have been replaced in `filepath`. File(s) shall have been completely generated.
  return true;
}

std::string RecorderRosbagDataLakeExtractor::executeLakeExtractorRosbagAPI(const Heex::RecorderArgs::RecorderEventRecordingPartArgs& query)
{
  // Step 1 : Init
  std::string dataFilePath(_recorderOutputDir + "/bag_" + query.uuid + "_" + query.eventUuid + ".bag");

  // Time format conversions
  boost::posix_time::ptime bpTime = boost::posix_time::from_iso_extended_string(query.timestamp);
  ros::Time event_time            = ros::Time::fromBoost(bpTime);
  long double tsf                 = atof(query.recordIntervalStart.c_str());
  long double tef                 = atof(query.recordIntervalEnd.c_str());
  ros::Duration ts(tsf);
  ros::Duration te(tef);
  ros::Time query_begin_time(event_time + ts);
  ros::Time query_end_time(event_time + te);
  std::cout << "[INFO] RecorderRosbagDataLakeExtractor::executeLakeExtractorRosbagAPI Recording Event at " << event_time << " with extraction interval [" << ts << ";" << te << "]."
            << "Extracting window is [" << query_begin_time << ";" << query_end_time << "]." << std::endl;

  // Open the original bag (datalakeBag) for Read and output bag (extractedBag) for Write
  rosbag::Bag datalakeBag(this->_recorderInputBag, rosbag::bagmode::Read);
  rosbag::Bag extractedBag;
  extractedBag.open(dataFilePath, rosbag::bagmode::Write);

  // Create a first view with a query for all messages from all topics. Check other parameters of addQuery for topic selection.
  rosbag::View viewQuery(true);
  viewQuery.addQuery(datalakeBag, query_begin_time, query_end_time);

  // Step 2 : Copy all messages from the _topics_tf list at the start of the bag. Suitable for tf_static and other configuration messages only.
  if (_topics_tf.size() > 0)
  {
    rosbag::View view_tf(datalakeBag, rosbag::TopicQuery(_topics_tf));
    BOOST_FOREACH (rosbag::MessageInstance const m, view_tf)
    {
      extractedBag.write(m.getTopic(), viewQuery.getBeginTime(), m); // FIXME: requires the same connection header ?
    }
  }

  // Step 3 : Copy all messages from all topics.
  // To select only specific topics, add a fourth parameter to addQuery() call. It should a std::vector<std::string> that contains all the topics to extract.
  BOOST_FOREACH (rosbag::MessageInstance const m, viewQuery)
  {
    extractedBag.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
  }

  datalakeBag.close();
  extractedBag.close();

  return dataFilePath;
}
