/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#ifndef APEX_AI_HEEX_DEMO_PKG_MONITOR_NODE_HPP_
#define APEX_AI_HEEX_DEMO_PKG_MONITOR_NODE_HPP_

#include "apex_ai_heex_demo_pkg/visibility_control.hpp" // for APEX_AI_HEEX_DEMO_PKG_PUBLIC
#include "executor/apex_node_base.hpp"                  // for apex::executor::apex_node_base
#include "logging/logging.hpp"                          // for apex::logging::Logger
#include "rclcpp/polling_subscription.hpp"              // for rclcpp::PollingSubscription

// Type(s) of message(s) the Monitor shall use
#include "std_msgs/msg/bool.hpp" // for std_msgs::msg::Bool

// Type of Monitor choosen from the Heex SDK library. Can be custom as well.
#include "BooleanMonitor.h"

namespace apex
{
namespace apex_ai_heex_demo_pkg
{

class APEX_AI_HEEX_DEMO_PKG_PUBLIC MonitorNode : public apex::executor::apex_node_base, public BooleanMonitor
{
  /// @brief Header file for the monitor on disengagement for the rosbag online
  /// (edge) detection on APEX AI Env on ROS 2.
public:
  /// @brief Construct a new MonitorNode object
  ///
  /// @param node_name ROS node  name
  /// @param node_namespace ROS node namespace
  /// @param topic ROS topic to listen to
  /// @param uuid
  /// @param serverIp
  /// @param serverPort
  MonitorNode(
      const apex::string_strict256_t& node_name,
      const apex::string_strict256_t& node_namespace,
      const apex::string_strict256_t& topic,
      const apex::string_strict256_t& uuid,
      const apex::string_strict256_t& serverIp,
      const uint& serverPort);

  void wait_for_matched() const;

private:
  void execute_impl() override;

  apex::executor::subscription_list get_triggering_subscriptions_impl() const override;

  const rclcpp::PollingSubscription<std_msgs::msg::Bool>::SharedPtr _subscription;
  apex::logging::Logger<> _logger;
};

} // namespace apex_ai_heex_demo_pkg
} // namespace apex

#endif // APEX_AI_HEEX_DEMO_PKG_MONITOR_NODE_HPP_
