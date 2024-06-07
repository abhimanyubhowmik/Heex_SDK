///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "HeexUtilsLog.h"
#include "RecorderArgs.h"

namespace Heex
{
namespace Tools
{
/// @def FILEPATH_CHARACTER_REPLACEMENT The string constant for space replacement.
const std::string ESCAPE_CHARACTER_REPLACEMENT_SPACE     = "&#32";
/// @def FILEPATH_CHARACTER_REPLACEMENT The string constant for semicolon replacement.
const std::string ESCAPE_CHARACTER_REPLACEMENT_SEMICOLON = "&#59";
/// @def FILEPATH_CHARACTER_REPLACEMENT The string constant for colon replacement.
const std::string ESCAPE_CHARACTER_REPLACEMENT_COLON     = "&#58";

/// @brief Encode method to replace all characters to escape within the provided std::string
///
/// @param msg Pass-by-reference string that will be modified by encoding.
void encodeEscapeCharactersWithReplacementCode(std::string& msg);

/// @brief Decode method to replace all characters to escape within the provided std::string
///
/// @param msg Pass-by-reference string that will be modified by decoding.
void decodeReplacementCodeWithEscapeCharacters(std::string& msg);

/// Enum to describe the type of prefix before any uuid.
enum UuidPrefixType
{
  Monitor,
  Recorder,
  Trigger,
  Event
};

/// Static container pair UuidPrefixType (enum) with their 1-or-2 letter(s) prefix (prefix as string). Enables enum to be casted from a std::string quickly.
static const std::unordered_map<std::string, UuidPrefixType> UUID_PREFIX_CAST = {
    {"D", UuidPrefixType::Monitor},
    {"CV", UuidPrefixType::Monitor},
    {"M", UuidPrefixType::Monitor},
    {"R", UuidPrefixType::Recorder},
    {"SV", UuidPrefixType::Recorder},
    {"T", UuidPrefixType::Trigger},
    {"E", UuidPrefixType::Event}};

/// Check if the provided uuid is valid and correspond to the right type if provided
bool checkUuid(const std::string& uuid, const UuidPrefixType& uuidPrefix);

/// Determine if the agent UUID doesn't match the expected type and store the warning message by-reference
bool checkAgentNonConsistentUuidType(const std::string uuid, const Heex::Tools::UuidPrefixType expectedType, const std::vector<std::string>& excludedPrefixes, std::string& msg);

/// Check if the provided timestamp is valid
bool checkTimestamp(const std::string& timestamp);

/// Check if the provided time interval duration is valid
bool checkTimeIntervalDuration(const std::string& timeIntervalDuration);

// Check if boundary2 is greater than boundary2
bool checkValidInterval(const std::string& boundary1, const std::string& boundary2);

/// Check if the provided timestamp is valid
bool checkCoupleValuesStr(const std::string& coupleargs);

/// Check if the provided timestamp is valid
bool checkCoupleValuesVector(const std::vector<Heex::RecorderArgs::ContextValue>& coupleargs);

/// Check if the provided Detectors is valid
bool checkDetectors(const std::string& coupleargs);

/// @brief Check if the provided recordingsFilepath is valid: does not contain any escaped characters. Manage two possibilities: only encoded filepath or couple [R-UUID:filepath;]
///
/// @param filepath Structure containing the filepath or filepath couple
/// @return true if filepath valid
/// @return false if filepath is not correctly formated
bool checkRecordingsFilepath(const std::string& filepath);

/// Check if the provided recordInterval is valid
bool checkRecordInterval(const std::string& recordInterval);

/// Print content of the vector
template <class T> void printVector(std::vector<T>& values)
{
  for (T value : values)
  {
    HEEX_LOG(debug) << value;
  }
  HEEX_LOG(debug) << std::endl;
}

/// Print content of the map
void printMap(std::map<std::string, std::string>& map);

/// @brief Add key to Map. Format string with ReplacementCode
///
/// @param m map to encode
/// @param key key
/// @param value value
void encodeValuesInMap(std::unordered_map<std::string, std::vector<std::string>>& m, std::string key, std::string value);

/// @brief Method to parse constant values.
/// @param value Content of the message to parse.
/// @return a pair of key and value
bool parseConstantValueMsg(const std::string& value, std::pair<std::string, std::string>& cv);

/// @brief Method to split constant-values values into a vector
/// @param value value part of the constant value
/// @return values vector of values
std::vector<std::string> parseConstantValueValuePart(std::string value);
} // namespace Tools
} // namespace Heex
