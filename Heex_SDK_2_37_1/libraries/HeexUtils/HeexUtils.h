///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file HeexConfig.h
///
/// @brief Creation of a HeexConfig class
///
/// @author Romain Grave
/// Contact: romain@heex.io
///
/// @date 2021-07-28
///
#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

///
/// @class HeexUtils
/// @brief Collection of commonly used functions.
namespace HeexUtils
{

/// @brief Convert degrees to radians
///
/// @param degVal : degree value to be converted
double radians(double degVal);

/// @brief Convert radians to degrees.
///
/// @param radVal : radians to be converted.
double degrees(double radVal);

/// @brief Generates a random string.
///
/// @param size : size of the random string to be generated.
/// @param charToExclude : ascii character between 48 '0' and 122 'z' that you don't whant in your string.
std::string generateRandomString(size_t size, const std::string& charToExclude);

/// @brief
/// @param msg : string to search for quoted parameter.
/// @param param : output parameter.
bool extractNextQuotedParameter(std::string& msg, std::string& param);

/// @brief Split the provided string by one delimiter character. Handle when a delimiter remains at the end.
///
/// @return Returns the different strings inside a vector
std::vector<std::string> split(const std::string& s, char delimiter);

/// @brief return true if both string are equal without case sensitivity.
bool caseInsensitiveStringCompare(const std::string& str1, const std::string& str2);

/// @brief returns given string in lower case
/// @param str string to convert
///
/// @return Returns the string in lower case
std::string convertStrToLower(const std::string& str);

/// @brief return system time in iso format timestamp.
std::string getTimestampStr();

/// @brief
/// @param timestamp in iso 8601 format
std::string convertTimestampToUTCStr(std::string timestamp);

/// @brief Convert iso type string date and time to unix timestamp.
///
/// @return Returns unix timestamp.
long double isoExtendedTimeToUnixTime(const std::string& date);

///@brief Convert unix (posix) timestamp to iso type string date and time.
///
/// @return Returns a time as std::string with the IsoExtendedTime format.
std::string unixTimeToIsoExtendedTime(long double& date);

/// @brief Convert str to a vector of vectors of strings
///
/// @param str string to decode
/// @param tupleSeparator character that separates elements from the same tuple
/// @param itemSeparator  character that separates elements from the same vector
/// @return std::vector<std::vector<std::string>>
std::vector<std::vector<std::string>> decodeStringAsVector(std::string str, const char tupleSeparator, const char itemSeparator);

/// @brief Convert 2D vector into a single encoded string
///
/// @param vec vector to encode
/// @param tupleSeparator character that separates elements from the same tuple
/// @param itemSeparator  character that separates elements from the same vector
/// @return std::string
std::string encodeVectorAsString(std::vector<std::vector<std::string>> vec, const char tupleSeparator, const char itemSeparator);

/// @brief Convert Map of key and vector into a single encoded string
///
/// @param m map to encode
/// @param tupleSeparator character that separates elements from the same tuple
/// @param itemSeparator  character that separates elements from the same vector
/// @return std::string
std::string formatMapAsString(const std::unordered_map<std::string, std::vector<std::string>>& m, const char tupleSeparator, const char itemSeparator);

/// @brief Convert 1D vector into a single encoded string
///
/// @param vec vector to encode
/// @param itemSeparator  character that separates elements from the same vector
/// @return std::string
std::string encodeVectorAsString(std::vector<std::string> vec, const char itemSeparator);

/// @brief Get the right Linux 7z version depending on OS architecture.
///
/// @return std::string
std::string getLinuxZipVersion();

/// @brief Generates the command for the dataCollector to build the recording archive depending on the file type and os
///
/// @param absolutePath7zExecutable the absolute path of the 7z executable
/// @param outputFile the name of the final archive to be created
/// @param filepaths pair of recorder files/folders paths and bool to specify if content of folder or the folder shall be zipped
///
/// @return std::string cmd to be executed
std::string generateArchiveCreationCommand(std::string absolutePath7zExecutable, std::string outputFile, std::vector<std::pair<std::string, bool>> filepaths);

/// @brief return the part of 'line' founnd inbetween 'start' and 'end'. //!\\ 'line' is altered, with just post 'end' left.
std::string parseValue(std::string& line, const std::string& start, const std::string& end);

/// @brief Return the name of the executable.
std::string getExecutableName();

/// @brief Return the name of the module.
std::string getModuleName();

/// @brief Return the list of arguments of the executable. Support a limited range of platform compared to getExecutableName or getModuleName.
std::vector<std::string> getExecutableArgs();

///@brief Return the name of the current Python script. Return empty string if not found or available.
std::string getPythonScriptName();

/// @brief Run the provided command and log its outputs
///
/// @param cmd Command to execute
/// @param prefix Prefix to add on every new log line. Default is empty.
/// @return std::string Entire command output
int executeCommandWithLog(const std::string& cmd, const std::string& prefix = "");
} // namespace HeexUtils
