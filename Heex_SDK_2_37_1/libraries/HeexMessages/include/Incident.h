///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#pragma once

#include <boost/algorithm/string/regex.hpp>
#include <boost/regex.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "HeexUtils.h"
#include "HeexUtilsLog.h"

namespace Heex
{
/// @brief Structure to store information regarding incidents.
struct Incident
{
  Incident(std::string timestamp, std::string uuid, std::string component, std::string componentVersion, std::string message)
      : timestamp(timestamp),
        uuid(uuid),
        component(component),
        componentVersion(componentVersion),
        message(message)
  {
  }

  Incident() = default;

  bool operator==(const Incident& other)
  {
    if (timestamp != other.timestamp)
    {
      return false;
    }

    if (uuid != other.uuid)
    {
      return false;
    }

    if (component != other.component)
    {
      return false;
    }

    if (componentVersion != other.componentVersion)
    {
      return false;
    }

    if (message != other.message)
    {
      return false;
    }

    return true;
  }

  std::string timestamp;        ///<Timestamp of the incident
  std::string uuid;             ///<UUID of the component that raised the incident
  std::string component;        ///<Name of the component
  std::string componentVersion; ///<Version of the component
  std::string message;          ///<Message that describes the incident

  /// Mimic JSON serialization without boost.
  std::string serialize()
  {
    std::stringstream s;

    // {"timestamp":"2022-11-18T11:01:29.608229","uuid":"","component":"heexDataCollector","componentVersion":"2.4.0","message":"DataCollector test incident."}
    s << "{\"timestamp\":\"" << timestamp << "\"";
    s << ",\"uuid\":\"" << uuid << "\"";
    s << ",\"component\":\"" << component << "\"";
    s << ",\"componentVersion\":\"" << componentVersion << "\"";
    s << ",\"message\":\"" << message << "\"}";

    return s.str();
  }

  static std::string serializeIncidents(std::vector<Incident> incidents)
  {
    std::stringstream ret;
    ret << "[";
    for (std::vector<Incident>::iterator it = incidents.begin(); it != incidents.end(); ++it)
    {
      ret << (*it).serialize();
      if (incidents.size() > 1 && it != (incidents.end() - 1))
      {
        ret << ",";
      }
    }
    ret << "]";
    return ret.str();
  }

  static std::vector<Incident> deserializeIncidents(std::string serializedIncidents)
  {
    std::vector<Incident> res;
    std::vector<std::string> lines;
    boost::algorithm::split_regex(lines, serializedIncidents, boost::regex("},{"));

    for (std::vector<std::string>::iterator it = lines.begin(); it != lines.end(); ++it)
    {
      std::string line = (*it);

      const std::string timestamp(HeexUtils::parseValue(line, "\"timestamp\":\"", "\""));
      const std::string uuid(HeexUtils::parseValue(line, "\"uuid\":\"", "\""));
      const std::string component(HeexUtils::parseValue(line, "\"component\":\"", "\""));
      const std::string componentVersion(HeexUtils::parseValue(line, "\"componentVersion\":\"", "\""));
      const std::string message(HeexUtils::parseValue(line, "\"message\":\"", "\""));

      if (timestamp != "" && message != "")
      {
        const Incident inc(timestamp, uuid, component, componentVersion, message);
        res.push_back(inc);
      }
    }
    return res;
  }
}; // struct Incident

} // namespace Heex
