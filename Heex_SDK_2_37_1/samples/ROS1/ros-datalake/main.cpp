///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file main.cpp
/// @author Matthieu Carré (matthieu@heex.io)
/// @brief Main process that separate the logic of detections and generation of Smart Data. It uses the Monitor to notify the SDE Core of any state event or transition. It runs the Recorder to expose the services to generate ContextValue and EventRecordingPart on Core demand (i.e. each raised event).
/// @version 0.1
/// @date 2022-04-21
#include <boost/program_options.hpp>
#include <iostream>
#include <sstream>

// Rosbag and time related function calls
#define BOOST_BIND_GLOBAL_PLACEHOLDERS // remove warning on the Bind placeholders for ROS
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

// Date conversion (ROS - Posix - ISO-Extended)
#define BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// Include the headers of the topics of interest.
#include <std_msgs/Bool.h>

// Monitors and Recorders using the HeexSDK
#include "MonitorADTransitionDisengagement/MonitorADTransitionDisengagement.h"
#include "RecorderRosbagDataLakeExtractor/RecorderRosbagDataLakeExtractor.h"

int main(int argc, char** argv)
{
  //
  // Setup values for example bag
  //
  boost::program_options::options_description desc{"Options"};
  // clang-format off
  desc.add_options()
  ("help,h", "Help screen")
  (
      "monitor_uuid,m",
      boost::program_options::value<std::string>()->default_value("M-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)"),
      "Monitor UUID and Implementation version"
  )
  (
      "recorder_uuid,r",
      boost::program_options::value<std::string>()->default_value("R-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)"),
      "Recorder UUID and Implementation version"
  )
  (
      "input_bag,i",
      boost::program_options::value<std::string>()->default_value("./example_offline_extraction.bag"),
      "Input bag path"
  )
  (
      "tf",
      boost::program_options::value<std::vector<std::string>>()->multitoken(),
      "List topics that need their messages to be extracted and copied at the start of the bag (eg. tf_static)."
  );
  // clang-format on

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << '\n';
    return EXIT_FAILURE;
  }

  const std::string monitor_uuid  = vm["monitor_uuid"].as<std::string>();
  const std::string recorder_uuid = vm["recorder_uuid"].as<std::string>();
  const std::string inputBag      = vm["input_bag"].as<std::string>();
  std::vector<std::string> topics_tf;
  if (vm.count("tf"))
  {
    topics_tf = vm["tf"].as<std::vector<std::string>>();
  }
  else
  {
    topics_tf.push_back(std::string("/tf_static"));
  }
  std::string inputDetectionTopic   = "/demo/bool";
  std::string recorderPositionTopic = "/gps/fix";
  std::string recorderOuputPath     = "."; // Store bag in the current directory

  //
  // Instantiate Monitors and Recorders
  //
  MonitorADTransitionDisengagement monitorBagDisengagement(monitor_uuid, "127.0.0.1", 4242);
  RecorderRosbagDataLakeExtractor recorderBagExtractor(recorder_uuid, "127.0.0.1", 4243, inputBag, recorderOuputPath, recorderPositionTopic, topics_tf);

  // Make sure all connected
  bool isAllAgentsConnected = false;
  while (!isAllAgentsConnected)
  {
    std::cout << "monitorBagDisengagement " << (monitorBagDisengagement.isConnected() ? "is connected to Core" : "is not connected to Core") << std::endl;
    std::cout << "recorderBagExtractor " << (recorderBagExtractor.isConnected() ? "is connected to Core" : "is not connected to Core") << std::endl;
    isAllAgentsConnected = monitorBagDisengagement.isConnected() && recorderBagExtractor.isConnected();
    sleep(1);
  }
  // Make sure all configured
  monitorBagDisengagement.awaitReady();

  //
  // Run Detection on bag
  //

  // Select topics (Select what we want to keep)
  std::vector<std::string> topics;
  topics.push_back(inputDetectionTopic);

  // Bag works to select part of bag where best candidates can be
  rosbag::Bag datalakeBag(inputBag, rosbag::bagmode::Read);
  rosbag::View viewQuery(true);
  viewQuery.addQuery(datalakeBag, rosbag::TopicQuery(topics));

  BOOST_FOREACH (rosbag::MessageInstance const m, viewQuery)
  {
    // Check if message correspond to the type to perform detection on with monitorBagDisengagement
    std_msgs::Bool::ConstPtr p = m.instantiate<std_msgs::Bool>();
    if (p != NULL)
    {
      // Get the content of the std_msgs::Bool message.
      // As the std_msgs::Bool::ConstPtr doesn't contain any timestamp, we will use the time from the message header. However, it forces us to add it as a second parameter to the computeNewBooleanMsg() methods.
      // Transform ROS timestamp to iso_extended format and use updateValue to notify monitor of a new value.
      // If your message already contains a header, consider removing this 2nd parameter completely from definition and declaration making the call even simpler.
      std::string ts = to_iso_extended_string(m.getTime().toBoost());
      monitorBagDisengagement.computeNewBooleanMsg(p, ts);
    }
  }

  // Close the bag
  datalakeBag.close();

  // Wait for Recorders to unstack and finish their jobs.
  // Either wait a finite number of time (eg. sleep(20);) or wait until no job remains in Recorder query stacks
  unsigned int waitingCount = 0;
  while (waitingCount < 5) // Wait at least 5s to be sure all jobs have been performed.
  {
    sleep(1);
    std::cout << "Waiting time " << waitingCount << "/5 before stopping." << std::endl;
    if (recorderBagExtractor.isPerformingCallback() == true)
    {
      waitingCount = 0;
    }
    else
    {
      ++waitingCount;
    }
  }

  return EXIT_SUCCESS;
}
