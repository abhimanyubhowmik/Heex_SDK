///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "HeexUtilsGnss.h"

#include <math.h>

#include "HeexUtils.h"
#include "HeexUtilsLog.h"

namespace HeexUtils
{
double degreesMinSecToDecimalDegees(double dms)
{
  double minutes{0};
  double degrees{0};
  double seconds{0};
  double milliseconds{0};

  degrees      = trunc(dms / 100);
  minutes      = dms - (degrees * 100);
  seconds      = (minutes - trunc(minutes)) * 60;
  milliseconds = (seconds - trunc(seconds)) * 1000;

  minutes = trunc(minutes);
  seconds = trunc(seconds);

  const double decimalDegree = degrees + minutes / 60 + seconds / 3600 + milliseconds / 3600000;
  return decimalDegree;
}

double distanceBetween(double lat1, double lon1, double lat2, double lon2)
{
  const double rlat1    = radians(lat1);
  const double rlat2    = radians(lat2);
  const double ralfalat = radians(lat2 - lat1);
  const double ralfalon = radians(lon2 - lon1);

  const double a = sin(ralfalat / 2.0) * sin(ralfalat / 2.0) + cos(rlat1) * cos(rlat2) * sin(ralfalon / 2.0) * sin(ralfalon / 2.0);
  const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  const double d = EARTHR * c;

  return d;
}

double courseTo(double lat1, double long1, double lat2, double long2)
{
  const double dlon = radians(long2 - long1);
  lat1              = radians(lat1);
  lat2              = radians(lat2);
  const double a1   = sin(dlon) * cos(lat2);
  double a2         = sin(lat1) * cos(lat2) * cos(dlon);
  a2                = cos(lat1) * sin(lat2) - a2;
  a2                = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += 6.28318530718; // 2 * Pi
  }
  return degrees(a2);
}

void computeTarget(double lat1, double lon1, double dist, double course, double& lat2, double& lon2)
{
  course               = radians(course);
  lat1                 = radians(lat1);
  lat2                 = radians(lat2);
  const double sinLat1 = sin(lat1);
  const double cosLat1 = cos(lat1);
  const double sinDR   = sin(dist / EARTHR);
  const double cosDR   = cos(dist / EARTHR);

  lat2 = asin(sinLat1 * cosDR + cosLat1 * sinDR * cos(course));
  lon2 = radians(lon1) + atan2(sin(course) * sinDR * cosLat1, cosDR - sinLat1 * sin(lat2));

  lat2 = degrees(lat2);
  lon2 = degrees(lon2);
}

bool isContainedIn(double latOrLon1, double value, double latOrLon2)
{
  return (latOrLon1 < value && value < latOrLon2) || (latOrLon2 < value && value < latOrLon1);
}

bool isContainedIn(double lat1, double lon1, double latValue, double lonValue, double lat2, double lon2)
{
  return (((lat1 < latValue && latValue < lat2) || (lat2 < latValue && latValue < lat1)) && ((lon1 < lonValue && lonValue < lon2) || (lon2 < lonValue && lonValue < lon1)));
}

bool isInsideCircle(double latValue, double lonValue, double latCenter, double lonCenter, double radius)
{
  const double distance = distanceBetween(latValue, lonValue, latCenter, lonCenter);
  HEEX_LOG(debug) << "distance = " << distance << std::endl;
  return distance <= radius;
}

bool isPointInPoly(double pLat, double pLon, std::vector<std::pair<double, double>> poly)
{
  if (poly.size() < 3)
  {
    return false;
  }

  bool c                                               = false;
  std::vector<std::pair<double, double>>::iterator it  = poly.begin();
  std::vector<std::pair<double, double>>::iterator pit = poly.end();
  pit--;

  while (it != poly.end())
  {
    if ((((*it).second >= pLon) != ((*pit).second >= pLon)) && (pLat <= ((*pit).first - (*it).first) * (pLon - (*it).second) / ((*pit).second - (*it).second) + (*it).first))
    {
      c = !c;
    }

    pit = it;
    it++;
  }
  return c;
}
} // namespace HeexUtils
