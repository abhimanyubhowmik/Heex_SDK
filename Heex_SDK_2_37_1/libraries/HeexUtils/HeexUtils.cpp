///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "HeexUtils.h"

#include <wchar.h>

#include <array>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#if defined(_WIN32) || defined(_WIN64)
  #include <windows.h>
#elif defined(_POSIX_VERSION) || defined(__linux__)
  #include <unistd.h>
#endif

#include "HeexUtilsFileOperations.h"
#include "HeexUtilsLog.h"
#include "whereami.h"

namespace HeexUtils
{
double radians(double degVal)
{
  return degVal / 180.0 * 3.14159265359;
}

double degrees(double radVal)
{
  return radVal / 3.14159265359 * 180.0;
}

std::string generateRandomString(size_t size, const std::string& charToExclude)
{
  std::string res;
  while (res.size() < size)
  {
    int r         = std::rand();
    r             = 48 + (r % (122 - 48)); // We want chars from '0' -> 48 to 'z' -> 122 in ascii table.
    bool excluded = false;
    for (std::string::const_iterator it = charToExclude.cbegin(); it != charToExclude.cend(); ++it)
    {
      if (r == (*it))
      {
        excluded = true;
      }
    }
    if (excluded == false)
    {
      res += std::to_string(r);
    }
  }
  return res;
}

bool extractNextQuotedParameter(std::string& msg, std::string& param)
{
  try
  {
    const size_t firstQuote  = msg.find_first_of("\"");
    const size_t secondQuote = msg.find_first_of("\"", firstQuote + 1);
    param                    = msg.substr(firstQuote + 1, secondQuote - firstQuote - 1);
    /// take the quote '\"' and the space behind it (s+2) or up to the end if size is reached (size remaining).
    msg                      = msg.substr(((secondQuote + 2 < msg.size()) ? secondQuote + 2 : msg.size()), std::string::npos);
    if (param.size() == 0 || firstQuote == std::string::npos || secondQuote == std::string::npos)
    {
      return false;
    }
    return true;
  }
  catch (const std::logic_error& e)
  {
    HEEX_LOG(error) << e.what() << '\n';
    return false;
  }
}

std::vector<std::string> split(const std::string& s, char delimiter)
{
  std::vector<std::string> result;
  std::stringstream ss(s);
  std::string item;

  while (getline(ss, item, delimiter))
  {
    result.push_back(item);
  }

  return result;
}

bool caseInsensitiveStringCompare(const std::string& str1, const std::string& str2)
{
  if (str1.size() != str2.size())
  {
    return false;
  }

  for (std::string::const_iterator c1 = str1.begin(), c2 = str2.begin(); c1 != str1.end(); ++c1, ++c2)
  {
    if (std::tolower(static_cast<unsigned char>(*c1)) != std::tolower(static_cast<unsigned char>(*c2)))
    {
      return false;
    }
  }
  return true;
}

std::string convertStrToLower(const std::string& str)
{
  std::string result = str;
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) { return std::tolower(c); });
  return result;
}

std::string getTimestampStr()
{
  std::stringstream t;

  const boost::posix_time::ptime timeUTC = boost::posix_time::microsec_clock::universal_time();
  t << boost::posix_time::to_iso_extended_string(timeUTC);

  return t.str();
}

std::string convertTimestampToUTCStr(std::string timestamp)
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
  // Parse ISO 8601 string into a ptime object
  const size_t posDelimiter = timestamp.find("T");
  if (posDelimiter == std::string::npos)
  {
    HEEX_LOG(error) << "HeexUtils | convertTimestampToUTCStr: "
                    << "T delimiter is not present in timestamp";
    return std::string();
  }

  // Timestamp is already in UTC
  size_t pos = timestamp.find("Z");
  if (pos != std::string::npos)
  {
    return timestamp.substr(0, pos);
  }

  // Timestamp is already in UTC
  pos = timestamp.find_first_of("+-", posDelimiter);
  if (pos == std::string::npos)
  {
    return timestamp;
  }

  const bool aheadTimezone = (timestamp[pos] == '+');

  //compute local time
  const std::string localTime = timestamp.substr(0, pos);
  timestamp                   = timestamp.substr(pos + 1, std::string::npos);

  pos                     = timestamp.find(":");
  const std::string hours = timestamp.substr(0, pos);

  timestamp = timestamp.substr(pos + 1, std::string::npos);

  const std::string minutes = timestamp.substr(0, 2);

  const boost::posix_time::ptime localTimePt = boost::posix_time::from_iso_extended_string(localTime);
  const boost::posix_time::hours hoursPt     = boost::posix_time::hours(std::stoi(hours));
  const boost::posix_time::minutes minutesPt = boost::posix_time::minutes(std::stoi(minutes));

  boost::posix_time::ptime utcTimePt;
  if (aheadTimezone == false)
  {
    utcTimePt = localTimePt + hoursPt + minutesPt;
  }
  else
  {
    utcTimePt = localTimePt - (hoursPt + minutesPt);
  }

  // Convert ptime object to UTC time
  return boost::posix_time::to_iso_extended_string(utcTimePt);
}

long double isoExtendedTimeToUnixTime(const std::string& date)
{
  const std::string ms("0" + date.substr(19, std::string::npos));
  const boost::posix_time::ptime bpTime     = boost::posix_time::from_iso_extended_string(date);
  const boost::posix_time::ptime bpZero     = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
  const boost::posix_time::time_duration td = bpTime - bpZero;
  return td.total_seconds() + std::stod(ms);
}

std::string unixTimeToIsoExtendedTime(long double& date)
{
  const time_t value(static_cast<time_t>(date));
  return boost::posix_time::to_iso_extended_string(boost::posix_time::from_time_t(value));
}

std::vector<std::vector<std::string>> decodeStringAsVector(std::string str, const char tupleSeparator, const char itemSeparator)
{
  std::vector<std::vector<std::string>> res;

  for (const std::string& tupleAsString : HeexUtils::split(str, itemSeparator))
  {
    const std::vector<std::string> tupleAsVector = HeexUtils::split(tupleAsString, tupleSeparator);
    res.push_back(tupleAsVector);
  }
  return res;
}

std::string encodeVectorAsString(std::vector<std::vector<std::string>> vec, const char tupleSeparator, const char itemSeparator)
{
  std::stringstream res;

  for (std::vector<std::string> tupleAsVector : vec)
  {
    for (unsigned int i = 0; i < tupleAsVector.size(); ++i)
    {
      res << tupleAsVector[i];
      if (i != tupleAsVector.size() - 1 || i == 0)
      {
        res << tupleSeparator;
      }
    }
    res << itemSeparator;
  }

  return res.str();
}

std::string formatMapAsString(const std::unordered_map<std::string, std::vector<std::string>>& m, const char tupleSeparator, const char itemSeparator)
{
  std::vector<std::vector<std::string>> keyValuesMsg;

  // Concantenante into a vector as {{key,values}, {key, values}}
  for (std::unordered_map<std::string, std::vector<std::string>>::const_iterator it = m.begin(); it != m.end(); ++it)
  {
    std::vector<std::string> keyValue;
    keyValue.push_back(it->first);
    keyValue.insert(keyValue.end(), std::begin(it->second), std::end(it->second));
    keyValuesMsg.push_back(keyValue);
  }
  std::string msg = HeexUtils::encodeVectorAsString(keyValuesMsg, tupleSeparator, itemSeparator);
  return msg;
}

std::string encodeVectorAsString(std::vector<std::string> vec, const char itemSeparator)
{
  std::stringstream res;

  for (const std::string& param : vec)
  {
    res << param << itemSeparator;
  }

  return res.str();
}

std::string getLinuxZipVersion()
{
#if defined(__x86_64__) || defined(_M_X64)
  return "7z86_64";
#elif defined(i386) || defined(__i386__) || defined(__i386) || defined(_M_IX86)
  return "7z86";
#elif defined(__aarch64__) || defined(_M_ARM64)
  return "7zARM64";
#else
  return "UNKNOWN";
#endif
}

std::string generateArchiveCreationCommand(std::string absolutePath7zExecutable, std::string outputFile, std::vector<std::pair<std::string, bool>> filepaths)
{
  std::string cmd = "\"" + absolutePath7zExecutable + "\" a -tzip -bsp1 -bse1 -mx1 \"" + outputFile + "\"";
  for (const std::pair<std::string, bool>& filepath : filepaths)
  {
    boost::filesystem::path path(HeexUtils::FileOperations::getCorrectlyEncodedPath(filepath.first));
    if (filepath.second) // Zip folder contents
    {
      path /= "*";
    }
    cmd += " \"" + HeexUtils::FileOperations::getUtf8EncodedPath(path) + "\"";
  }
  return cmd;
}

std::string parseValue(std::string& line, const std::string& start, const std::string& end)
{
  if (line.size() == 0 || start.size() == 0 || end.size() == 0)
  {
    return "";
  }

  const size_t begS = line.find(start);
  const size_t endS = begS + start.size();

  const size_t begE = line.find(end, endS);
  const size_t endE = begE + end.size();

  if (begS == std::string::npos || endS == std::string::npos || begE == std::string::npos || endE == std::string::npos)
  {
    return "";
  }

  std::string ret(line.substr(endS, begE - endS));
  line = line.substr(endE);
  return ret;
}

std::string getExecutableName()
{
  // NOLINTBEGIN
  int dirnameLength = 0;
  const int length  = wai_getExecutablePath(NULL, 0, NULL);
  char* path        = (char*)malloc(length + 1);
  wai_getExecutablePath(path, length, &dirnameLength);
  path[length] = '\0';
  std::string name(path);

  const size_t pos = name.find_last_of("/\\") + 1;
  if (pos < name.size())
  {
    name = name.substr(pos);
  }
  return name;
  // NOLINTEND
}
} // namespace HeexUtils

std::string HeexUtils::getModuleName()
{
  // NOLINTBEGIN
  int dirnameLength = 0;
  const int length  = wai_getModulePath(NULL, 0, NULL);
  char* path        = (char*)malloc(length + 1);
  wai_getModulePath(path, length, &dirnameLength);
  path[length] = '\0';
  std::string name(path);

  const size_t pos = name.find_last_of("/\\") + 1;
  if (pos < name.size())
  {
    name = name.substr(pos);
  }
  return name;
  // NOLINTEND
}

std::vector<std::string> HeexUtils::getExecutableArgs()
{
#if defined(_WIN32) || defined(_WIN64)
  LPWSTR lpCommandLine = GetCommandLineW();
  int argc;
  LPWSTR* argv = CommandLineToArgvW(lpCommandLine, &argc);

  std::vector<std::string> args(argc);
  for (int i = 0; i < argc; i++)
  {
    // Convert a wide Unicode string to an UTF8 string
    int argLength = WideCharToMultiByte(CP_UTF8, 0, argv[i], -1, NULL, 0, NULL, NULL);
    if (argLength > 0)
    {
      std::vector<char> buf(argLength);
      WideCharToMultiByte(CP_UTF8, 0, argv[i], -1, buf.data(), argLength, NULL, NULL);
      args[i] = std::string(buf.data());
    }
  }
  LocalFree(argv);
  return args;
#elif defined(_POSIX_VERSION) || defined(__linux__)
  // Construct the path to the /proc/<pid>/cmdline file for the current process
  const pid_t pid                       = getpid();
  const std::string procDirPath         = "/proc/" + std::to_string(pid) + "/cmdline";
  bool firstArgExecutableNameGotRemoved = false;

  // Open the file and read its contents
  std::ifstream file(procDirPath.c_str());
  if (!file)
  {
    // An error has occured: File can't be read. Return empty args list.
    return std::vector<std::string>{};
  }

  std::vector<std::string> args;
  std::string arg;
  while (std::getline(file, arg, '\0'))
  {
    if (firstArgExecutableNameGotRemoved == false)
    {
      firstArgExecutableNameGotRemoved = true;
      continue;
    }

    // Add any new argument separated by '\0' to the vector
    args.push_back(arg);
  }
  file.close();
  return args;
#else
  // Not implemented. Return empty args list.
  return std::vector<std::string>{};
#endif
}

std::string HeexUtils::getPythonScriptName()
{
  const std::vector<std::string> executableArgs = HeexUtils::getExecutableArgs();
  for (const auto& arg : executableArgs)
  {
    if (arg.find(".py") != std::string::npos)
    {
      // Only extract the script filename with extension
      const boost::filesystem::path scriptPath = arg;
      return HeexUtils::FileOperations::getUtf8EncodedPath(scriptPath.filename());
    }
  }
  return "";
}

int HeexUtils::executeCommandWithLog(const std::string& cmd, const std::string& prefix)
{
  // Prepare result and custom deleter
  int result             = -1;
  const auto pipeDeleter = [&result](FILE* stream)
  {
#if defined(_POSIX_VERSION)
    result = stream ? pclose(stream) : -1;
#elif defined(_WIN32) || defined(_WIN64)
    result = stream ? _pclose(stream) : -1;
#else
    result = stream ? pclose(stream) : -1;
#endif
  };

  // Scope where the unique_ptr shall live. Set result on scope exit or if it returns prematurely.
  {
    // Create pipe using unique_ptr to add a custom deleter if it throws
#if defined(_POSIX_VERSION)
    const std::unique_ptr<FILE, decltype(pipeDeleter)> logStream(popen(cmd.c_str(), "r"), pipeDeleter);
#elif defined(_WIN32) || defined(_WIN64)
    // NOTE: _wpopen can only be used on Windows NT
    // Use wstring to handle UNICODE characters
    // add quote so that wpopen handles spaces in the command
    const std::wstring cmdWideStr = L"\"" + HeexUtils::FileOperations::getCorrectlyEncodedPath(cmd) + L"\"";
    const std::unique_ptr<FILE, decltype(pipeDeleter)> logStream(_wpopen(cmdWideStr.c_str(), L"rt"), pipeDeleter);
#else
    const std::unique_ptr<FILE, decltype(pipeDeleter)> logStream(popen(cmd.c_str(), "r"), pipeDeleter);
#endif
    if (logStream == NULL)
    {
      HEEX_LOG(error) << "HeexUtils::executeCommandWithLog | Failed to run the command: Can't create the pipe.\n";
      // Pipe get close by the deleter but we can disregard its return value.
      return -1;
    }

    // Read pipe until end of file, or an error occurs. This part can throw.
    try
    {
      std::array<char, 256> buffer = {'\0'};
      ;
      while (fgets(buffer.data(), static_cast<int>(buffer.size()), logStream.get()) != nullptr)
      {
        // Trim progression if a % is detected. Use reverse iterator to find last occurence of progress.
        auto lastPercent = std::find(buffer.rbegin(), buffer.rend(), '%');
        if (lastPercent != buffer.rend())
        {
          if (lastPercent + 4 < buffer.rend())
          {
            std::string progress = std::string(lastPercent, lastPercent + 4);
            std::reverse(progress.begin(), progress.end());
            HEEX_LOG(info) << prefix << "\t" << progress;
            continue;
          }
        }

        // Prints with HEEX_LOG on every new non-empty line. Add provided prefix.
        if (buffer.size() != 0 && buffer[0] != '\n')
        {
          HEEX_LOG(info) << prefix << "\t" << buffer.data();
        }
      }
    }
    catch (const std::exception& e)
    {
      HEEX_LOG(error) << "HeexUtils::executeCommandWithLog | Failed to run the command. Failed to read the pipe: " << e.what() << '\n';
      // Pipe get close by the deleter but we can disregard its return value.
      return -1;
    }
  }

  // Return pipe final value
  return result;
}
