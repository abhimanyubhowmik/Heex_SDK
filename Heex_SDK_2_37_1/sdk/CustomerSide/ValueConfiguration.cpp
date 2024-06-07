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

#include "ValueConfiguration.h"

#include <boost/regex.hpp>
#include <sstream>

#include "HeexUtils.h"
#include "Tools.h"

///@brief maps the string value of the zone behavior to a ZoneBehavior struct. Unknown shall be defaulted to "In"
///
///@param zoneBehaviorString
///@return ZoneBehaviorStruct
ZoneBehavior mapBehaviorStringToStruct(const std::string& zoneBehaviorString)
{
  ZoneBehavior zoneBehaviorStruct{ZoneBehavior::In};
  // convert the string to enum
  if (HeexUtils::caseInsensitiveStringCompare(zoneBehaviorString, "in"))
  {
    zoneBehaviorStruct = ZoneBehavior::In;
  }
  else if (HeexUtils::caseInsensitiveStringCompare(zoneBehaviorString, "out"))
  {
    zoneBehaviorStruct = ZoneBehavior::Out;
  }
  else if (HeexUtils::caseInsensitiveStringCompare(zoneBehaviorString, "enter"))
  {
    zoneBehaviorStruct = ZoneBehavior::Enter;
  }
  else if (HeexUtils::caseInsensitiveStringCompare(zoneBehaviorString, "exit"))
  {
    zoneBehaviorStruct = ZoneBehavior::Exit;
  }
  else
  {
    // keep zoneBehaviorStruct intialized 'In'
  }
  return zoneBehaviorStruct;
}

const std::unordered_map<ZoneBehavior, std::string> MAP_BEHAVIOR_STRUCT_TO_STRING =
    {{ZoneBehavior::In, "in"}, {ZoneBehavior::Out, "out"}, {ZoneBehavior::Enter, "enter"}, {ZoneBehavior::Exit, "exit"}};

ValueConfiguration::ValueConfiguration() : _valid(false) {}

ValueConfiguration::ValueConfiguration(const ValueConfHeader& h) : _valid(h.valid), _uuid(h.uuid), _cvUuid(h.cvUuid), _name(h.name), _type(h.type), _params(h.params) {}

ValueConfHeader ValueConfiguration::parseStrValue(const std::string& strValue)
{
  ValueConfHeader h;

  h.params = HeexUtils::split(strValue, ' ');
  if (h.params.size() > 4)
  {
    h.uuid   = h.params[0];
    h.cvUuid = h.params[1];
    h.name   = h.params[2];
    h.type   = h.params[3];
    Heex::Tools::decodeReplacementCodeWithEscapeCharacters(h.name);
    Heex::Tools::decodeReplacementCodeWithEscapeCharacters(h.type);
    if (h.uuid.size() != 0 && h.cvUuid.size() != 0 && h.name.size() != 0 && h.type.size() != 0)
    {
      h.params.erase(h.params.begin(), h.params.begin() + 4);
      HEEX_LOG(debug) << "args : " << h.uuid << " " << h.name << " on cv : " << h.cvUuid << std::endl;
      h.valid = true;
    }
    for (std::vector<std::string>::iterator it = h.params.begin(); it != h.params.end(); ++it)
    {
      Heex::Tools::decodeReplacementCodeWithEscapeCharacters((*it));
    }
  }
  return h;
}

ValueConfigurationDouble::ValueConfigurationDouble(const ValueConfHeader& h) : ValueConfiguration(h)
{
  if (_valid == true && _params.size() != 0)
  {
    _value = atof(_params[0].c_str());
    HEEX_LOG(debug) << "value  : " << _value << std::endl;
    _valid = true;
  }
}

ValueConfigurationDoubleInterval::ValueConfigurationDoubleInterval(const ValueConfHeader& h) : ValueConfiguration(h)
{
  // _params contains at [0] the value "[strLow,strHigh]", e.g. "[29,72]" OR "[[29,72]]"
  if (_valid == true && _params.size() != 0)
  {
    // Check if the list is itself in a list.
    if (_params[0].size() >= 4 && _params[0][0] == '[' && _params[0][1] == '[' && _params[0][_params[0].size() - 2] == ']' && _params[0][_params[0].size() - 1] == ']')
    {
      _params[0].pop_back();  // Remove ] at the end
      _params[0].erase(0, 1); // Remove [ at the beginning
    }

    const std::string startStr("[");
    const std::string middleStr(",");
    const std::string endStr("]");

    const size_t start  = _params[0].find(startStr);
    const size_t middle = _params[0].find(middleStr, start + startStr.size());
    const size_t end    = _params[0].find(endStr, middle + middleStr.size());

    std::string strLow(_params[0].substr(startStr.size(), middle - startStr.size()));
    std::string strHigh(_params[0].substr(middle + middleStr.size(), end - middle - middleStr.size()));
    HEEX_LOG(debug) << " strLow : '" << strLow << " strHigh " << strHigh << "'" << std::endl;

    if (strLow.size() != 0 && strHigh.size() != 0)
    {
      _lowValue  = atof(strLow.c_str());
      _highValue = atof(strHigh.c_str());
      HEEX_LOG(debug) << "values  : '" << this->getLowValue() << " " << this->getHighValue() << "'" << std::endl;
      _valid = true;
    }
  }
}

ValueConfigurationBoolean::ValueConfigurationBoolean(const ValueConfHeader& h) : ValueConfiguration(h)
{
  // _params contains at [0] the value "value", e.g. "true"
  if (_valid == true && _params.size() != 0)
  {
    while (_params[0].size() >= 2 && _params[0][0] == '[' && _params[0][_params[0].size() - 1] == ']')
    {
      _params[0].pop_back();  // Remove ] at the end
      _params[0].erase(0, 1); // Remove [ at the beginning
    }

    if (_params[0].size() >= 2 && _params[0][0] == '\"' && _params[0][_params[0].size() - 1] == '\"')
    {
      _params[0].pop_back();  // Remove " at the end
      _params[0].erase(0, 1); // Remove " at the beginning
    }

    if (HeexUtils::caseInsensitiveStringCompare(_params[0], "true"))
    {
      _value = true;
      _valid = true;
    }
    else if (HeexUtils::caseInsensitiveStringCompare(_params[0], "false"))
    {
      _value = false;
      _valid = true;
    }
    else
    {
      HEEX_LOG(error) << "ValueConfigurationBoolean::ValueConfigurationBoolean Invalid boolean value: '" << _params[0] << "'" << std::endl;
      _valid = false;
    }
  }
}

ValueConfigurationThreshold::ValueConfigurationThreshold(const ValueConfHeader& h) : ValueConfiguration(h)
{
  // _params contains at [0] the value "[strComparison,strValue]", e.g. "["Above",72]"
  if (_valid == true && _params.size() != 0)
  {
    while (_params[0].size() >= 2 && _params[0][0] == '[' && _params[0][_params[0].size() - 1] == ']')
    {
      _params[0].pop_back();  // Remove ] at the end
      _params[0].erase(0, 1); // Remove [ at the beginning
    }

    const std::string startStr("\"");
    const std::string middleStr("\",");
    const size_t start  = _params[0].find(startStr);
    const size_t middle = _params[0].find(middleStr, start + startStr.size());
    const size_t end    = _params[0].size();

    std::string strComparison(_params[0].substr(startStr.size(), middle - startStr.size()));
    std::string strValue(_params[0].substr(middle + middleStr.size(), end - middle - middleStr.size()));
    HEEX_LOG(debug) << "strValue : " << strValue << " strConparison : " << strComparison << std::endl;

    if (strComparison.size() != 0 && strValue.size() != 0)
    {
      if (HeexUtils::caseInsensitiveStringCompare(strComparison, "Above"))
      {
        _comparison = Above;
      }
      else if (HeexUtils::caseInsensitiveStringCompare(strComparison, "Below"))
      {
        _comparison = Below;
      }
      else if (HeexUtils::caseInsensitiveStringCompare(strComparison, "Equal"))
      {
        _comparison = Equal;
      }
      else
      {
        HEEX_LOG(error) << "ValueConfigurationThreshold::ValueConfigurationThreshold Invalid comparison." << std::endl;
        _valid = false;
        return;
      }
      _value = atof(strValue.c_str());
      HEEX_LOG(debug) << "value  : '" << this->getValue() << " comparison : " << this->getComparison() << "'" << std::endl;
      _valid = true;
    }
  }
}

ValueConfigurationString::ValueConfigurationString(const ValueConfHeader& h) : ValueConfiguration(h)
{
  // _params contains at [0] the value "[str1,str2,str3]", e.g. "["right","left","low"]" OR "[["right","left","low"]]" OR "["Right"]" OR "[["Right"]]"
  if (_valid == true && _params.size() != 0)
  {
    std::string param;
    for (unsigned int i = 0; i < _params.size(); ++i)
    {
      param += _params[i];

      if (i + 1 < _params.size())
      {
        param += " ";
      }
    }

    while (param.size() >= 2 && param[0] == '[' && param[param.size() - 1] == ']')
    {
      param.pop_back();  // Remove ] at the end
      param.erase(0, 1); // Remove [ at the beginning
    }

    if (param.find(',') != std::string::npos)
    {
      _values = HeexUtils::split(param, ',');
      for (unsigned int i = 0; i < _values.size(); ++i)
      {
        if (_values[i].size() > 1 && _values[i].back() == '"')
        {
          _values[i].pop_back(); // Remove " at the end
        }
        if (_values[i].size() > 1 && _values[i].front() == '"')
        {
          _values[i].erase(0, 1); // Remove " at the beginning
        }
        HEEX_LOG(debug) << "add _values[" << i << "] : " << _values[i] << std::endl;
      }
    }
    else
    {
      HEEX_LOG(debug) << "will one value : " << param << std::endl;
      if (param.size() >= 2 && param[0] == '\"' && param[param.size() - 1] == '\"')
      {
        param.pop_back();  // Remove " at the end
        param.erase(0, 1); // Remove " at the beginning
      }
      HEEX_LOG(debug) << "Add one value : " << param << std::endl;
      _values.push_back(param);
    }
    _valid = true;
  }
}
#ifndef HEEX_SDK_NO_BOOST_JSON
  #include <boost/json/src.hpp>

ValueConfigurationZone::ValueConfigurationZone(const ValueConfHeader& h) : ValueConfiguration(h)
{
  HEEX_LOG(trace) << "ValueConfigurationZone::ValueConfigurationZone()" << std::endl;
  if (_valid == true && _params.size() != 0)
  {
    // Check if the list is itself in a list.
    if (_params[0].size() >= 4 && _params[0][0] == '[' && _params[0][1] == '[' && _params[0][_params[0].size() - 2] == ']' && _params[0][_params[0].size() - 1] == ']')
    {
      _params[0].pop_back();  // Remove [ at the end
      _params[0].erase(0, 1); // Remove ] at the beginning
    }

    boost::json::error_code ec;
    boost::json::value jv = boost::json::parse(_params[0], ec);

    if (ec)
    {
      HEEX_LOG(error) << "Parsing failed: " << ec.message() << "\n";
    }

    const boost::json::array& values = jv.as_array();

    ZoneBehavior behavior = ZoneBehavior::In;
    for (boost::json::value v : values)
    {
      const boost::json::object& obj = v.as_object();
      const std::string name         = boost::json::value_to<std::string>(obj.at("name"));

      if (obj.contains("behavior"))
      {
        const std::string behaviorString = boost::json::value_to<std::string>(obj.at("behavior"));
        behavior                         = mapBehaviorStringToStruct(behaviorString);
      }

      if (HeexUtils::caseInsensitiveStringCompare(name, "circle"))
      {
        const boost::json::object& vals      = obj.at("vals").as_object();
        const double radius                  = boost::json::value_to<double>(vals.at("radius"));
        const std::string center             = boost::json::value_to<std::string>(vals.at("center"));
        std::vector<std::string> centerCoord = HeexUtils::split(center, ',');
        const ZoneCircle c(radius, std::make_pair(std::stod(centerCoord[0]), std::stod(centerCoord[1])), behavior);
        _circles.push_back(c);
        HEEX_LOG(debug) << "New '" << MAP_BEHAVIOR_STRUCT_TO_STRING.at(behavior) << "' Circle of centre : " << std::stod(centerCoord[0]) << " " << std::stod(centerCoord[1])
                        << " and radius : " << radius << std::endl;
      }
      else if (HeexUtils::caseInsensitiveStringCompare(name, "polygon"))
      {
        HEEX_LOG(debug) << "Add new '" << MAP_BEHAVIOR_STRUCT_TO_STRING.at(behavior) << "' Polygon :" << std::endl;
        const boost::json::array& vals = obj.at("vals").as_array();
        std::vector<std::pair<double, double>> points;
        for (const boost::json::value& v : vals)
        {
          const std::string val          = boost::json::value_to<std::string>(v);
          std::vector<std::string> coord = HeexUtils::split(val, ',');
          points.push_back(std::make_pair(std::stod(coord[0]), std::stod(coord[1])));
          HEEX_LOG(debug) << "point - " << std::stod(coord[0]) << " " << std::stod(coord[1]) << std::endl;
        }
        _polygons.push_back(ZonePolygon(points, behavior));
      }
    }
    _valid = true;
  }
}
#endif
