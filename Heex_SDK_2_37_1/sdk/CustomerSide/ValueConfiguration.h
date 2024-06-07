///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file ValueConfiguration.cpp
///
/// @brief Class collection to store a configuration value of type double, string, and double interval.
/// If this does not suit your needs, you can implement a new class inheriting from ValueConfiguration.
///
/// @author Romain Grave
/// Contact: romain@heex.io
///
/// @date 2022-03-16
///

#pragma once

#include <iostream>
#include <vector>

/// @brief This struct store all params of a value conf to be handled.
struct ValueConfHeader
{
  /// @brief The struct is not valid until filled correctly.
  ValueConfHeader() = default;

  bool valid{false};
  std::string uuid;
  std::string cvUuid;
  std::string name;
  std::string type;
  std::vector<std::string> params;
};

/// @brief Mother class for value configurations.
class ValueConfiguration
{
public:
  /// @brief Construct a valid ValueConfiguration
  ///
  /// @param h Build the parsed params from the configuration command.
  ValueConfiguration(const ValueConfHeader& h);

  /// @brief Build a non valid ValueConf.
  ValueConfiguration();

  /// @brief virtual destructor.
  virtual ~ValueConfiguration() = default;

  /// @brief Parse the ValueConfiguration command into a header struct.
  ///
  /// @param ValueConfiguration command.
  ///
  /// @return Header struct containing all param from the command.
  static ValueConfHeader parseStrValue(const std::string& strValue);

  /// @brief return true if the valueConfiguration is valid and has been parsed with success.
  ///
  /// @return validity status.
  virtual const bool& isValid() { return _valid; }

  /// @brief Getter on uuid (D-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX).
  ///
  /// @return uuid.
  const std::string& getUuid() { return _uuid; }

  /// @brief Getter on uuid (CV-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX).
  ///
  /// @return cvUuid.
  const std::string& getCvUuid() { return _cvUuid; }

  /// @brief Getter on name of the condition/strategy.
  ///
  /// @param name.
  const std::string& getName() { return _name; }

  ///
  /// @brief Getter on type of the condition/strategy.
  ///
  /// @return const std::string&
  const std::string& getType() { return _type; }

  ///
  /// @brief Get the Header object
  ///
  /// @return const ValueConfHeader
  const ValueConfHeader getHeader()
  {
    ValueConfHeader h;
    h.valid  = _valid;
    h.name   = _name;
    h.type   = _type;
    h.uuid   = _uuid;
    h.cvUuid = _cvUuid;
    h.params = _params;

    return h;
  }

protected:
  bool _valid;
  std::string _uuid;
  std::string _cvUuid;
  std::string _name;
  std::string _type;
  std::vector<std::string> _params; //<! contains valueConfig's value field in [0], and possible unparsed args in [1:]
};

/// @brief Class inherited from ValueConfiguration to store a double value.
class ValueConfigurationDouble : public ValueConfiguration
{
public:
  /// @brief Constructor to build a ValueConfigurationDouble.
  ///
  /// @param h all params from the ValueConfiguration command stored in a ValueConfHeader.
  ValueConfigurationDouble(const ValueConfHeader& h);

  /// @brief virtual destructor.
  virtual ~ValueConfigurationDouble() = default;

  /// @brief Getter on the double value.
  ///
  /// @return value.
  double getValue() { return _value; }

private:
  double _value;
};

/// @brief Class inherited from ValueConfiguration to store an interval on double value.
class ValueConfigurationDoubleInterval : public ValueConfiguration
{
public:
  /// @brief Constructor to build a ValueConfigurationDoubleInterval.
  ///
  /// @param h all params from the ValueConfiguration command stored in a ValueConfHeader.
  ValueConfigurationDoubleInterval(const ValueConfHeader& h);

  /// @brief virtual destructor.
  virtual ~ValueConfigurationDoubleInterval() = default;

  /// @brief Getter on the low double value of the interval.
  ///
  /// @return value.
  double getLowValue() { return _lowValue; }

  /// @brief Getter on the high value of the interval.
  ///
  /// @return value.
  double getHighValue() { return _highValue; }

private:
  double _lowValue;
  double _highValue;
};

class ValueConfigurationBoolean : public ValueConfiguration
{
public:
  /// @brief Constructor to build a ValueConfigurationBoolean.
  ///
  /// @param h all params from the ValueConfiguration command stored in a ValueConfHeader.
  ValueConfigurationBoolean(const ValueConfHeader& h);

  /// @brief virtual destructor.
  virtual ~ValueConfigurationBoolean() = default;

  /// @brief Getter on the string value.
  ///
  /// @return value.
  const bool& getValue() { return _value; }

private:
  bool _value;
};

class ValueConfigurationThreshold : public ValueConfiguration
{
public:
  enum Comparison
  {
    Above,
    Equal,
    Below
  };
  /// @brief Constructor to build a ValueConfigurationThreshold.
  ///
  /// @param h all params from the ValueConfiguration command stored in a ValueConfHeader.
  ValueConfigurationThreshold(const ValueConfHeader& h);

  /// @brief virtual destructor.
  virtual ~ValueConfigurationThreshold() = default;

  /// @brief Getter on the string value.
  ///
  /// @return value.
  const double& getValue() { return _value; }

  const Comparison& getComparison() { return _comparison; }

private:
  Comparison _comparison;
  double _value;
};

class ValueConfigurationString : public ValueConfiguration
{
public:
  /// @brief Constructor to build a ValueConfigurationString.
  ///
  /// @param h all params from the ValueConfiguration command stored in a ValueConfHeader.
  ValueConfigurationString(const ValueConfHeader& h);

  /// @brief virtual destructor.
  virtual ~ValueConfigurationString() = default;

  /// @brief Getter on the string value.
  ///
  /// @return value.
  const std::vector<std::string>& getValues() { return _values; }

  friend std::ostream& operator<<(std::ostream& os, const ValueConfigurationString& vcs)
  {
    os << "[";
    for (const std::string& s : vcs._values)
    {
      os << s;
      if (s != vcs._values.back())
      {
        os << ", ";
      }
    }
    os << "]" << std::endl;
    return os;
  }

private:
  std::vector<std::string> _values;
};

///@brief behavior expected of a Zone Trigger. Default shall be "In".
///
enum class ZoneBehavior
{
  In,    // Trigger is active when inside a zone. DEFAULT
  Out,   // Trigger is active when outside a zone.
  Enter, // Trigger is active when entering a zone.
  Exit   // Trigger is active when exiting a zone.
};

struct ZoneCircle
{
  double radius;
  std::pair<double, double> center;
  ZoneBehavior behavior; ///< each zone has a behavior associated to it (in/out/enter/exit)

  ///@brief Construct a new Zone Circle object
  ///
  ///@param r radius
  ///@param c center point
  ///@param b [OPTIONAL] zone's behavior. Defaults to 'In'
  ZoneCircle(double r, std::pair<double, double> c, ZoneBehavior b = ZoneBehavior::In)
  {
    center   = c;
    radius   = r;
    behavior = b;
  }
};

struct ZonePolygon
{
  std::vector<std::pair<double, double>> points;
  ZoneBehavior behavior; ///< each zone has a behavior associated to it (in/out/enter/exit)
  ///@brief Construct a new Zone Polygon object
  ///
  ///@param p all the polygon's defining points
  ///@param b [OPTIONAL] zone's behavior. Defaults to 'In'
  ZonePolygon(std::vector<std::pair<double, double>> p, ZoneBehavior b = ZoneBehavior::In)
  {
    points   = p;
    behavior = b;
  }
};

/// @brief Class inherited from ValueConfiguration to store an array of zones.
class ValueConfigurationZone : public ValueConfiguration
{
public:
  /// @brief Constructor to build a ValueConfigurationZone.
  ///
  /// @param h all params from the ValueConfiguration command stored in a ValueConfHeader.
  ValueConfigurationZone(const ValueConfHeader& h);

  /// @brief virtual destructor.
  virtual ~ValueConfigurationZone() = default;

  std::vector<ZoneCircle> getCircles() { return _circles; }

  std::vector<ZonePolygon> getPolygons() { return _polygons; }

private:
  std::vector<ZoneCircle> _circles;
  std::vector<ZonePolygon> _polygons;
};
