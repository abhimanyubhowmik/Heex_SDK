///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "../include/Tools.h"

#include <algorithm> // std::count
#include <boost/algorithm/string/replace.hpp>
#include <regex>

#include "../include/AgentArgs.h"
#include "../include/RecorderArgs.h"
#include "HeexUtils.h"

void Heex::Tools::printMap(std::map<std::string, std::string>& map)
{
  for (std::map<std::string, std::string>::iterator it = map.begin(); it != map.end(); ++it)
  {
    HEEX_LOG(debug) << it->first << " => " << it->second << '\n';
  }
  HEEX_LOG(debug) << std::endl;
}

void Heex::Tools::encodeEscapeCharactersWithReplacementCode(std::string& msg)
{
  boost::replace_all(msg, " ", ESCAPE_CHARACTER_REPLACEMENT_SPACE);
  boost::replace_all(msg, ";", ESCAPE_CHARACTER_REPLACEMENT_SEMICOLON);
  boost::replace_all(msg, ":", ESCAPE_CHARACTER_REPLACEMENT_COLON);
}

void Heex::Tools::decodeReplacementCodeWithEscapeCharacters(std::string& msg)
{
  boost::replace_all(msg, ESCAPE_CHARACTER_REPLACEMENT_SPACE, " ");
  boost::replace_all(msg, ESCAPE_CHARACTER_REPLACEMENT_SEMICOLON, ";");
  boost::replace_all(msg, ESCAPE_CHARACTER_REPLACEMENT_COLON, ":");
}

bool Heex::Tools::checkUuid(const std::string& uuid, const Heex::Tools::UuidPrefixType& uuidPrefix)
{
  std::vector<std::string> uuidChunks = HeexUtils::split(uuid, '-');
  if (uuidChunks.size() <= 1)
  {
    return false;
  }
  // Guard to check for if the prefix of the uuid macth the one of the expected uuidPrefix
  const std::string prefixStr = uuidChunks[0];
  bool prefixFound            = false;
  for (std::unordered_map<std::string, UuidPrefixType>::const_iterator it = Heex::Tools::UUID_PREFIX_CAST.begin(); it != Heex::Tools::UUID_PREFIX_CAST.end(); ++it)
  {
    if ((prefixStr == (*it).first) && (uuidPrefix == (*it).second))
    {
      prefixFound = true;
      break;
    }
  }
  if (prefixFound == false)
  {
    return false;
  }

  // All Heex Uuid are as such: X-xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx
  return uuidChunks.size() == 6;
}

bool Heex::Tools::checkTimestamp(const std::string& timestamp)
{
  /**
   * ISO 8601 format is accepted. Example formats are
   * YYYY-MM-DDTHH:MM:SS.fffffffff where T is the date-time separator and .f the fractional seconds. Accept precision from 0 to 9 f.
   * YYYY-MM-DDTHH:MM:SS,fffffffff where T is the date-time separator and ,f the fractional seconds. Accept precision from 0 to 9 f.
   * YYYY-MM-DDTHH:MM:SS,fff+xx:00 where T is the date-time separator and ,f the fractional seconds and xx is the timezone difference
   * YYYY-MM-DDTHH:MM:SS,fffZ where T is the date-time separator and ,f the fractional seconds and the time is in UTC
   * YYYY-MM-DDTHH:MM:SSZ where T is the date-time separator and ,f the fractional seconds and the time is in UTC
   * Time zone offsets are between -12:00 and 14:00
   */
  const std::string patternValidTimestamp = "[0-9]{4}-(0[1-9]|1[0-2])-(0[1-9]|[1-2][0-9]|3[0-1])T(2[0-3]|[01][0-9]):[0-5][0-9]:[0-5][0-9]"
                                            "((.|,)[0-9]{1,9})?"
                                            "([+](1[0-3]|0[0-9]):[0-5][0-9]|[+14:00]|[-](1[0-1]|0[0-9]):[0-5][0-9]|[-12:00]|Z)?$";
  const std::regex r(patternValidTimestamp);
  return std::regex_match(timestamp, r);
}

bool Heex::Tools::checkTimeIntervalDuration(const std::string& value)
{
  const std::string patternValidInterval = "[+-]?([0-9]*[.])?[0-9]+"; //should be a float
  const std::regex r(patternValidInterval);
  return std::regex_match(value, r);
}

bool Heex::Tools::checkValidInterval(const std::string& boundary1, const std::string& boundary2)
{
  // -5 5
  // Check if the first value is greater
  if (std::stof(boundary1) >= std::stof(boundary2))
  {
    return false;
  }
  return true;
}

bool Heex::Tools::checkCoupleValuesStr(const std::string& filepath)
{
  //Basic check to assert same number of couple and item delimiter
  const size_t cntCpleSep   = std::count(filepath.begin(), filepath.end(), Heex::HEEX_COUPLE_SEPARATOR);
  const size_t cntKeyValSep = std::count(filepath.begin(), filepath.end(), Heex::HEEX_KEY_VALUE_SEPARATOR);
  return cntCpleSep == cntKeyValSep;
}

bool Heex::Tools::checkCoupleValuesVector(const std::vector<Heex::RecorderArgs::ContextValue>& valuesVector)
{
  for (const Heex::RecorderArgs::ContextValue& contextValue : valuesVector)
  {
    if (contextValue.key.empty())
    {
      return false;
    }
  }
  return true;
}

bool Heex::Tools::checkDetectors(const std::string& detectors)
{
  //check if the uuids are valid
  const std::vector<std::string> values = HeexUtils::split(detectors, ';');
  for (const std::string& v : values)
  {
    if (!Heex::Tools::checkUuid(v, Heex::Tools::UuidPrefixType::Monitor))
    {
      return false;
    }
  }
  return true;
}

bool Heex::Tools::checkRecordingsFilepath(const std::string& v)
{
  //check if the filepath is not empty and does not contain any escaped characters
  const size_t cntCpleSep   = std::count(v.begin(), v.end(), Heex::HEEX_COUPLE_SEPARATOR);
  const size_t cntKeyValSep = std::count(v.begin(), v.end(), Heex::HEEX_KEY_VALUE_SEPARATOR);
  const size_t cntKeySpace  = std::count(v.begin(), v.end(), Heex::HEEX_MSG_ARG_SEPARATOR);
  return (cntCpleSep == cntKeyValSep && cntKeySpace == 0);
}

bool Heex::Tools::checkRecordInterval(const std::string& intervalValue)
{
  return Heex::Tools::checkTimeIntervalDuration(intervalValue);
}

bool Heex::Tools::checkAgentNonConsistentUuidType(
    const std::string uuid,
    const Heex::Tools::UuidPrefixType expectedType,
    const std::vector<std::string>& excludedPrefixes,
    std::string& msg)
{
  // Initial computation to see if the uuid is not using an excluded prefix
  bool isNotUsingExcludedPrefix = true;
  for (const std::string& excludedPrefix : excludedPrefixes)
  {
    if (uuid.compare(0, excludedPrefix.size(), excludedPrefix) == 0)
    {
      isNotUsingExcludedPrefix = false;
      break;
    }
  }

  // Check if UUID is consistent to the expected type and not using an excluded prefix
  if (Heex::Tools::checkUuid(uuid, expectedType) && isNotUsingExcludedPrefix)
  {
    return true;
  }

  // Manage when it doesn't Retrieve candidates for the expected type to print human readable list.
  std::string candidatePrefixesStr("(");
  size_t i = 0;
  for (const auto& uuidPrefixPair : Heex::Tools::UUID_PREFIX_CAST)
  {
    // Also exclude UUID prefixes from the excluded prefixes list
    if (uuidPrefixPair.second == expectedType && std::find(excludedPrefixes.begin(), excludedPrefixes.end(), uuidPrefixPair.first) == excludedPrefixes.end())
    {
      if (i > 0)
      {
        candidatePrefixesStr += ", ";
      }
      candidatePrefixesStr += uuidPrefixPair.first;
      ++i;
    }
  }
  candidatePrefixesStr += ")";

  msg = "This Agent UUID " + uuid + " has a prefix not consistent with its implementation type. Expected prefix is " + candidatePrefixesStr +
        ". Running it with an unconsistent type is at your own risk.";
  return false;
}

void Heex::Tools::encodeValuesInMap(std::unordered_map<std::string, std::vector<std::string>>& m, std::string key, std::string value)
{
  Heex::Tools::encodeEscapeCharactersWithReplacementCode(key);
  Heex::Tools::encodeEscapeCharactersWithReplacementCode(value);
  m[key].push_back(value);
}

bool Heex::Tools::parseConstantValueMsg(const std::string& value, std::pair<std::string, std::string>& cv)
{
  // Expect to parse string formatted as "Key Value".i.e rostopic /demo/bool. Key=rostopic and value=/demo/bool
  // The following format is also supported: rostopic /demo/bool /gps/fix Key=rostopic and value=/demo/bool /gps/fix
  // The following format is also supported: rostopic ["/demo/bool" "/gps/fix"]  Key=rostopic and value=["/demo/bool" "/gps/fix"]

  // Find the first separator
  const std::string::size_type sepPos = value.find(' ');
  if (sepPos == std::string::npos)
  {
    HEEX_LOG(error) << "parseConstantValueMsg Bad msg format : received " << value;
    return false;
  }
  std::string keyPart   = value.substr(0, sepPos);
  std::string valuePart = value.substr(sepPos + 1);

  if (keyPart == std::string() || valuePart == std::string())
  {
    HEEX_LOG(error) << "parseConstantValueMsg Bad msg format";
    return false;
  }
  Heex::Tools::decodeReplacementCodeWithEscapeCharacters(keyPart);
  Heex::Tools::decodeReplacementCodeWithEscapeCharacters(valuePart);
  cv = std::make_pair(keyPart, valuePart);
  return true;
}

std::vector<std::string> Heex::Tools::parseConstantValueValuePart(std::string value)
{
  std::vector<std::string> values;
  // if the value is an array (i.e, start with [), split the values into a vector
  if (value[0] == '[' && value[value.size() - 1] == ']')
  {
    const std::regex regex("\"([^\"]*)\"");

    std::vector<std::string> extractedValues;
    std::smatch match;

    // Find all matches in the input string
    while (std::regex_search(value, match, regex))
    {
      extractedValues.push_back(match[1].str());
      value = match.suffix().str(); // Update the remaining part of the input string
    }
    values = extractedValues;
  }
  else
  {
    values.push_back(value);
  }

  return values;
}
