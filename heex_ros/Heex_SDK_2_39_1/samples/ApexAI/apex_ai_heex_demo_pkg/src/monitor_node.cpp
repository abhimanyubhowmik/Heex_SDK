///
/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "apex_ai_heex_demo_pkg/monitor_node.hpp"

#include <boost/date_time/posix_time/posix_time.hpp> // for time operations, an alternative could be to std/apex
#include <chrono>                                    // for std::chrono::seconds

#include "logging/logging_macros.hpp" // for APEX_INFO
#include "rclcpp/qos.hpp"             // for rclcpp::DefaultQoS
#include "rclcpp/time.hpp"            // for rclcpp::Time

namespace apex
{
namespace apex_ai_heex_demo_pkg
{

MonitorNode::MonitorNode(
    const apex::string_strict256_t& node_name,
    const apex::string_strict256_t& node_namespace,
    const apex::string_strict256_t& topic,
    const apex::string_strict256_t& uuid,
    const apex::string_strict256_t& serverIp,
    const uint& serverPort)
    : apex::executor::apex_node_base{node_name.c_str(), node_namespace.c_str()},
      BooleanMonitor(uuid, serverIp, serverPort, "1.0.0"),
      _subscription{get_rclcpp_node().create_polling_subscription<std_msgs::msg::Bool>(topic.c_str(), rclcpp::DefaultQoS().keep_last(1))},
      _logger{&get_rclcpp_node(), "MonitorNode"}
{
}

void MonitorNode::wait_for_matched() const
{
  HEEX_LOG(info) << "waiting for matched publisher to topic " << _subscription->get_topic_name();
  _subscription->wait_for_matched(1U, std::chrono::seconds{5});
}

void MonitorNode::execute_impl()
{
  apex::system_clock::time_point now = apex::system_clock::now();
  // Qos is set as keep the last one, so there is only message in the queue
  auto loaned_msgs{_subscription->read()};
  if (loaned_msgs.empty())
  {
    return;
  }
  // Detection logic
  if (loaned_msgs.back().info().valid())
  {
    boost::posix_time::ptime event_time = boost::posix_time::from_time_t(apex::system_clock::to_time_t(now));
    event_time += boost::posix_time::microseconds(std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000);
    std::string ts = boost::posix_time::to_iso_extended_string(event_time);
    this->updateValue(loaned_msgs.back().data().data, ts);
  }
}

apex::executor::subscription_list MonitorNode::get_triggering_subscriptions_impl() const
{
  return {_subscription};
}

} // namespace apex_ai_heex_demo_pkg
} // namespace apex
