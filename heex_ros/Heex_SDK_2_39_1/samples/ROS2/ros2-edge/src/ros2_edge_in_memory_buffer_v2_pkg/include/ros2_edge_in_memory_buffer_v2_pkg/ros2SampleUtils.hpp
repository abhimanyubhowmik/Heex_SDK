/// Copyright (c) 2024 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#pragma once

#include <boost/date_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <rclcpp/rclcpp.hpp>

#include "HeexUtilsLog.h"

namespace ros2_sample_utils
{
/// @brief Converts the time from a rclcpp time format to a string iso extended one
///
/// @param rclcpp_time time in rclcpp::Time format
///
/// @return time in iso extended format, std::string
inline std::string rclcpptime_to_iso_extended_str(const rclcpp::Time& rclcpp_time)
{
  // Convert ROS time (which is in nanoseconds since epoch) to a ptime object
  const boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
  boost::posix_time::time_duration since_epoch(0, 0, rclcpp_time.seconds(), (rclcpp_time.nanoseconds() - rclcpp_time.seconds() * 1e9));

  // Adding microsecond precision
  since_epoch += boost::posix_time::microseconds((rclcpp_time.nanoseconds() / 1000) % 1000000);

  const boost::posix_time::ptime ptime = epoch + since_epoch;

  // Convert ptime object to ISO extended string
  return boost::posix_time::to_iso_extended_string(ptime);
}

/// @brief Converts the time from a string iso extended format to a rclcpp time one
///
/// @param posix_iso_extended_str time in iso extended format, std::string
///
/// @return time in rclcpp::Time format
inline rclcpp::Time iso_extended_str_to_rclcpptime(const std::string& posix_iso_extended_str)
{
  const boost::posix_time::ptime boost_ptime      = boost::posix_time::from_iso_extended_string(posix_iso_extended_str);
  // Calculate the duration from epoch
  const boost::posix_time::time_duration duration = boost_ptime - boost::posix_time::ptime(boost::gregorian::date(1970, 1, 1));

  // Convert duration to rclcpp::Time
  return rclcpp::Time(duration.total_seconds(), duration.total_nanoseconds() - duration.total_seconds() * 1e9);
}

} // namespace ros2_sample_utils
