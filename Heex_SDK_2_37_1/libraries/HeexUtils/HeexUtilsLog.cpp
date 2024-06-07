///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "HeexUtilsLog.h"

#include "HeexUtils.h"
#include "HeexUtilsFileOperations.h"

#if defined(_POSIX_VERSION)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-copy"
#endif
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>

#if defined(_POSIX_VERSION)
  #pragma GCC diagnostic pop
  #include <sys/stat.h>
#endif

#include "HeexUtilsFileOperations.h"

namespace HeexUtils
{
Log::Log()
{
  std::string displayName = HeexUtils::getExecutableName();

  // When dealing with Python script, replace the display name with the python script name.
  // It covers any cases when the library is used as a module, not only when using Python SDK.
  if (displayName != HeexUtils::getModuleName())
  {
    const std::string candidateScriptName = HeexUtils::getPythonScriptName();

    // Make sure we are really using a python script.
    if (!candidateScriptName.empty())
    {
      displayName = candidateScriptName;
    }
  }

  this->init(displayName);
}

void Log::init(const std::string& executableName, bool /*logInTerminal*/, boost::log::trivial::severity_level severity)
{
  boost::log::add_common_attributes();
  boost::log::core::get()->set_filter(boost::log::trivial::severity >= severity);

  auto fmtTimeStamp                  = boost::log::expressions::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f");
  //auto fmtThreadId = boost::log::expressions::attr<boost::log::attributes::current_thread_id::value_type>("ThreadID");
  auto fmtSeverity                   = boost::log::expressions::attr<boost::log::trivial::severity_level>("Severity");
  const boost::log::formatter logFmt = boost::log::expressions::format("[%1%] [%2%] [%3%] %4%") % fmtTimeStamp % executableName % fmtSeverity % boost::log::expressions::smessage;

  auto consoleSink = boost::log::add_console_log(std::clog);
  consoleSink->set_formatter(logFmt);

  /// This vector will store the log folder options. First part would be the part of the path that should exist. Second part would be the part of the path that would be created if inexistant.
  std::vector<std::pair<std::string, std::string>> logPathOptions;

  logPathOptions.push_back(std::pair<std::string, std::string>("./", "logs"));
  logPathOptions.push_back(std::pair<std::string, std::string>("~/", ".heex/logs"));
  std::string logFolder = this->selectBestLogOption(logPathOptions);

  if (logFolder.empty() == true)
  {
    return;
  }

  _file_sink = boost::log::add_file_log(
      boost::log::keywords::target = logFolder,
#if BOOST_VERSION >= 107000
      boost::log::keywords::file_name        = logFolder + "/" + executableName + ".log",
      boost::log::keywords::target_file_name = logFolder + "/" + executableName + "_%3N.log",
#else
      boost::log::keywords::file_name = logFolder + "/" + executableName + "_%3N.log",
#endif
      boost::log::keywords::rotation_size         = 10 * 1024 * 1024,   // 10 MB
      boost::log::keywords::max_size              = 5 * 24 * 60 * 60,   // 5 days
      boost::log::keywords::open_mode             = std::ios_base::app, // Open for append and immediately rotate
      boost::log::keywords::time_based_rotation   = boost::log::sinks::file::rotation_at_time_point(0, 0, 0),
      /// Rotate only on startup to cover also the crash / kill cases
      boost::log::keywords::enable_final_rotation = false,
      boost::log::keywords::auto_flush            = true // Log entries get written immediately rather than by block
  );
  _file_sink->set_formatter(logFmt);
}

void Log::disableLogToFile()
{
  // Make sure the sink for the file hasn't been explicitly deleted before
  if (_file_sink)
  {
    // Remove the sink: any log records written to the logging system will no longer be forwarded to the file
    const boost::shared_ptr<boost::log::core> core = boost::log::core::get();
    core->remove_sink(_file_sink);

    _file_sink.reset();
  }
}

void Log::setMinimumLogLevel(const std::string& severityStr)
{
  boost::log::trivial::severity_level level = boost::log::trivial::info;
  if (HeexUtils::caseInsensitiveStringCompare(severityStr, "trace"))
  {
    level = boost::log::trivial::trace;
  }
  if (HeexUtils::caseInsensitiveStringCompare(severityStr, "debug"))
  {
    level = boost::log::trivial::debug;
  }
  if (HeexUtils::caseInsensitiveStringCompare(severityStr, "info"))
  {
    level = boost::log::trivial::info;
  }
  if (HeexUtils::caseInsensitiveStringCompare(severityStr, "warning"))
  {
    level = boost::log::trivial::warning;
  }
  if (HeexUtils::caseInsensitiveStringCompare(severityStr, "error"))
  {
    level = boost::log::trivial::error;
  }
  if (HeexUtils::caseInsensitiveStringCompare(severityStr, "fatal"))
  {
    level = boost::log::trivial::fatal;
  }
  if (HeexUtils::caseInsensitiveStringCompare(severityStr, "default"))
  {
    level = boost::log::trivial::HEEX_MAX_LOG_LEVEL;
  }

  this->setMinimumLogLevel(level);
}

void Log::setMinimumLogLevel(boost::log::trivial::severity_level severity)
{
  boost::log::core::get()->set_filter(boost::log::trivial::severity >= severity);
}

void Log::printLoggerHeader()
{
  // Make the logger header printable only once
  if (_fileSinkCurrentFileName.empty())
  {
    HEEX_LOG(info) << "Logging to file at " << this->getActiveLogFilepath();
  }
}

std::string Log::getActiveLogFilepath()
{
  if (_file_sink == nullptr)
  {
    return std::string();
  }

  // Update if file name has changed
  _fileSinkCurrentFileName = HeexUtils::FileOperations::getUtf8EncodedPath(_file_sink->locked_backend().get()->get_current_file_name());
  return _fileSinkCurrentFileName;
}

std::string Log::selectBestLogOption(const std::vector<std::pair<std::string, std::string>>& logPathOptions)
{
  std::string selectedPath;

  for (std::vector<std::pair<std::string, std::string>>::const_iterator optionIt = logPathOptions.begin(); optionIt != logPathOptions.end() && selectedPath.empty(); ++optionIt)
  {
    //std::cout << std::endl << "Check parent : " << optionIt->first << std::endl;
    //Resolve parent path
    std::string parentPath(optionIt->first);
    if (parentPath.find('~') != std::string::npos && HeexUtils::FileOperations::expandTilde(parentPath) == false)
    {
      continue;
    }

    //std::cout << "Expanded parent : " << parentPath << std::endl;
    //Test parent path existance
    const boost::filesystem::path parentPathBFP(parentPath);
    boost::system::error_code ec;
    if (boost::filesystem::exists(parentPathBFP, ec) == false || ec.value() != 0 || boost::filesystem::is_directory(parentPathBFP, ec) == false || ec.value() != 0)
    {
      continue;
    }

    //std::cout << "Parent exists!" << std::endl;
    //Test parent path to write access
    bool hasWriteAccess = false;
    try
    {
      // Attempt to create a temporary file in the folder
      const boost::filesystem::path tempFile = parentPathBFP / "HeexTempfile.txt";
      std::ofstream file(tempFile.string());
      if (file)
      {
        file.close();
        hasWriteAccess = true;
        boost::filesystem::remove(tempFile); // Clean up the temporary file
      }
      else
      {
        hasWriteAccess = false;
      }
    }
    catch (const boost::filesystem::filesystem_error& e)
    {
      // Handle exceptions (e.g., folder does not exist)
      std::cerr << "Error checking for write access on (" << parentPath << "): " << e.what() << std::endl;
    }

    if (hasWriteAccess == false)
    {
      continue;
    }
    //std::cout << "Parent is writable!" << std::endl;

    //fragment child path
    std::string childPath(optionIt->second);
    boost::replace_all(childPath, "\\", "/");
    std::vector<std::string> fragmentedChildPath;
    boost::split(fragmentedChildPath, childPath, boost::is_any_of("/"));

    boost::filesystem::path pathBFP(parentPath);
    for (const std::string& childDir : fragmentedChildPath)
    {
      boost::system::error_code ecPathBFP;
      pathBFP = pathBFP / childDir;
      //std::cout << "Check child existance : " << pathBFP << std::endl;
      // Check existance
      if (boost::filesystem::exists(pathBFP, ecPathBFP) == false && ec.value() != 0)
      {
        //std::cout << "Child exist : " << pathBFP << std::endl;
        if (boost::filesystem::is_directory(pathBFP, ecPathBFP) == false || ec.value() != 0)
        {
          // In case there is a file instead!
          continue;
        }
      }
      else if (boost::filesystem::create_directory(pathBFP, ecPathBFP) == false || ec.value() != 0)
      {
        continue;
      }
    }
    selectedPath = pathBFP.string();
    //std::cout << "GOOD path found : " << selectedPath << std::endl;
  }
  return selectedPath;
}
} // namespace HeexUtils
