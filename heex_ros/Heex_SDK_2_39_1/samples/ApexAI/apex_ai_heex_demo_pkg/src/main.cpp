///
/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include <chrono>  // for std::chrono::milliseconds
#include <memory>  // for std::make_shared<>
#include <utility> // for std::move

#include "apex_ai_heex_demo_pkg/monitor_node.hpp"  // for MonitorNode
#include "apex_ai_heex_demo_pkg/recorder_node.hpp" // for RecorderNode
#include "apex_init/apex_init.hpp"                 // for apex::pre_init and apex::post_init
#include "cpputils/common_exceptions.hpp"          // for apex::runtime_error
#include "executor/executor_factory.hpp"           // for apex::executor::executor_factory
#include "executor/executor_runner.hpp"            // for apex::executor::executor_runner
#include "interrupt/interrupt_handler.hpp"         // for apex::interrupt_handler

// Type(s) of message(s) the Recorder shall use. Include header for needed
// messages types
#include "sensor_msgs/msg/imu.hpp"         // for sensor_msgs::msg::Imu
#include "sensor_msgs/msg/nav_sat_fix.hpp" // for sensor_msgs::msg::NavSatFix
#include "std_msgs/msg/bool.hpp"           // for std_msgs::msg::Bool

int32_t main(const int32_t argc, char** const argv)
{
  int32_t result{};

  try
  {
    if (apex::pre_init(argc, argv, false) != APEX_RET_OK)
    {
      throw apex::runtime_error("Can't pre-init Apex");
    }

    const apex::interrupt_handler::installer interrupt_handler_installer{};

    // Instanciate Detector with the UUID from the web plateform
    auto apex_ai_heex_demo_pkg_monitor = std::make_shared<apex::apex_ai_heex_demo_pkg::MonitorNode>(
        "apex_ai_heex_demo_pkg_monitor", "apex_ai_heex_demo_pkg_namespace", "/demo/bool", "M-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX", "127.0.0.1", 4242);

    // Instanciate Recorder with the UUID from the web platform
    std::vector<apex::string_strict256_t> topics = {"/gps/fix", "/demo/bool", "/imu"};
    std::unordered_map<apex::string_strict256_t, apex::string_strict256_t> topics_once_at_bagstart{{"/tf_static", "tf2_msgs/msg/TFMessage"}};
    auto apex_ai_heex_demo_pkg_recorder = std::make_shared<apex::apex_ai_heex_demo_pkg::RecorderNode>(
        "apex_ai_heex_demo_pkg_recorder", "apex_ai_heex_demo_pkg_namespace", "R-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX", "127.0.0.1", 4243, topics, topics_once_at_bagstart);

    apex_ai_heex_demo_pkg_recorder->add_subscriber<sensor_msgs::msg::NavSatFix>({topics[0]});
    apex_ai_heex_demo_pkg_recorder->add_subscriber<std_msgs::msg::Bool>({topics[1]});
    apex_ai_heex_demo_pkg_recorder->add_subscriber<sensor_msgs::msg::Imu>({topics[2]});
    const auto executor = apex::executor::executor_factory::create();

    apex_ai_heex_demo_pkg_monitor->wait_for_matched();

    executor->add(std::move(apex_ai_heex_demo_pkg_monitor));

    const apex::executor::executor_runner runner{apex::executor::executor_runner::deferred, *executor};

    if (apex::post_init() != APEX_RET_OK)
    {
      throw std::runtime_error("Can't post-init Apex");
    }

    runner.issue();
    apex::interrupt_handler::wait();
  }
  catch (const std::exception& e)
  {
    if (rclcpp::ok())
    {
      APEX_FATAL_R(e.what());
    }
    else
    {
      std::cerr << e.what() << "\n";
    }
    result = 2;
  }
  catch (...)
  {
    if (rclcpp::ok())
    {
      APEX_FATAL_R("Unknown error occurred");
    }
    else
    {
      std::cerr << "Unknown error occurred"
                << "\n";
    }
    result = -1;
  }
  return result;
}
