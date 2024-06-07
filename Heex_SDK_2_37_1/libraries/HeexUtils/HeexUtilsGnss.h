///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file HeexUtilsGnss.h
///
/// @brief Creation of a HeexUtilsGnns collection
///
/// @author Romain Grave
/// Contact: romain@heex.io
///
/// @date 2021-12-17
///
#pragma once

#include <vector>

///
/// @class HeexUtils
/// @brief Collection of commonly used functions.

namespace HeexUtils
{

#define EARTHR 6372795.477598 // in meters

/// @brief Translate gnss DDMMSS to DD.DDDD
///
/// @param  : dms Degree minute second string to be converted.
double degreesMinSecToDecimalDegees(double dms);

/// @brief Returns distance in meters between two gnss coordonates.
///
/// @param lat1 : Latitude of first point.
/// @param lng1 : Longitude of first point.
/// @param lat2 : Latitude of second point.
/// @param lng2 : Longitude of second point.
double distanceBetween(double lat1, double lon1, double lat2, double lon2);

/// @brief Compute course from a gnss location to an other.
///
/// @param lat1 : Latitude of first point.
/// @param lng1 : Longitude of first point.
/// @param lat2 : Latitude of second point.
/// @param lng2 : Longitude of second point.
double courseTo(double lat1, double long1, double lat2, double long2);

/// @brief
/// @param lat1 : Latitude of first point.
/// @param lng1 : Longitude of first point.
/// @param dist : Distance in meters from start to target.
/// @param course : Course from start to target.
/// @param lat2 : Latitude of second point.
/// @param lng2 : Longitude of second point.
void computeTarget(double lat1, double lon1, double dist, double course, double& lat2, double& lon2);

/// @brief Determine if a position (in one dimention) is included between two positions

/// @brief value : Latitude or Longitude of the value to check.
/// @param latOrLon2 : Latitude or Longitude of second point.
bool isContainedIn(double latOrLon1, double value, double latOrLon2);

/// @brief Determine if a position is included between two positions
///

/// @brief latValue : Latitude of the value to check.
/// @param lonValue : Longitude of the value to check.
/// @param lat2 : Latitude of second point.
/// @param lng2 : Longitude of second point.
bool isContainedIn(double lat1, double lon1, double latValue, double lonValue, double lat2, double lon2);

/// @brief
///
/// @param latValue Latitude of the value to check.
/// @param lonValue Longitude of the value to check.
/// @param latCenter Latitude of center of the circle.
/// @param lonCenter Longitude of center of the circle.
/// @param radius Radius of the circle
/// @return true
/// @return false
bool isInsideCircle(double latValue, double lonValue, double latCenter, double lonCenter, double radius);

/// @brief Determine if a position inside a polygon
///
/// @param pLat : Latitude of point to test.
/// @param pLon : Longitude of point to test.
/// @param poly : list of points of the polygon.
bool isPointInPoly(double pLat, double pLon, std::vector<std::pair<double, double>> poly);
} // namespace HeexUtils
