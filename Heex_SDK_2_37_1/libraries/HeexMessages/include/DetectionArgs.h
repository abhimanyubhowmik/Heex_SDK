///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include <boost/date_time/posix_time/posix_time.hpp>

#include "Incident.h"

enum DetectorActivityType
{
  SIGNAL_ON,
  SIGNAL_OFF,
  SIGNAL_ON_OFF,
  END_DETECTOR_ACTIVITY_TYPE
}; // DetectorActivityType

/// Args structure for DetectionArgs used in Detector interprocess communication.
struct DetectionArgs
{
  DetectionArgs() : uuid(""), type(END_DETECTOR_ACTIVITY_TYPE), timestamp(""), unparsedArgs("") {}

  bool valid{false};
  std::string uuid;
  DetectorActivityType type;
  std::string timestamp;
  std::string unparsedArgs;
  boost::posix_time::ptime receptionTime;
  boost::posix_time::ptime utcTimestampPt;
  std::vector<Heex::Incident> incidents;
}; // DetectionArgs
