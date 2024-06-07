///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#pragma once

#include <boost/log/trivial.hpp>
#include <boost/version.hpp>
#include <string>

#if defined(_POSIX_VERSION)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-copy"
#endif
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#if defined(_POSIX_VERSION)
  #pragma GCC diagnostic pop
#endif
namespace HeexUtils
{
#ifndef HEEX_MAX_LOG_LEVEL
  #define HEEX_MAX_LOG_LEVEL info
#endif

// Define HEEX_LOG(lvl) using BOOST_LOG_TRIVIAL(lvl).
// Try to strip the line if the lvl is below HEEX_MAX_LOG_LEVEL
#if BOOST_VERSION >= 107500 // BOOST_VERSION >= 1.75.0
  // Expands to 'if constexpr' when supported, or 'if' otherwise.
  // See BOOST_IF_CONSTEXPR in https://www.boost.org/doc/libs/1_75_0/libs/config/doc/html/boost_config/boost_macro_reference.html#boost_config.boost_macro_reference.macros_that_allow_use_of_c__17_features_with_c__14_or_earlier_compilers
  // Alternative: https://stackoverflow.com/a/69109527
  #define HEEX_LOG(lvl)                                                                                                          \
    BOOST_IF_CONSTEXPR(boost::log::trivial::lvl >= boost::log::trivial::severity_level(boost::log::trivial::HEEX_MAX_LOG_LEVEL)) \
    BOOST_LOG_TRIVIAL(lvl)
#else
// Compatibility strip. Don't use either BOOST_IF_CONSTEXPR and if constexpr
  #define HEEX_LOG(lvl)                                                                                           \
    if (boost::log::trivial::lvl >= boost::log::trivial::severity_level(boost::log::trivial::HEEX_MAX_LOG_LEVEL)) \
    BOOST_LOG_TRIVIAL(lvl)
#endif

class Log
{
public:
  // Singleton log
  static Log& instance()
  {
    static Log singletonInstance;
    return singletonInstance;
  }

  /// Log should not be cloneable.
  Log(Log& other)       = delete;
  Log(const Log& other) = delete;

  /// Log should not be assignable.
  Log& operator=(Log&)       = delete;
  Log& operator=(const Log&) = delete;

  /// @brief Permanently disable logging ouput to files.
  void disableLogToFile();

  /// @brief Set the maximum level of log to the provided level from a string.
  /// Choose a level between "fatal", "error", "info", "debug" and "trace".
  /// If the provided string does not match one of these level, it will be default to "info".
  ///
  /// @param severityStr
  void setMinimumLogLevel(const std::string& severityStr);

  /// @brief Set the maximum level of log to the provided level from a string
  void setMinimumLogLevel(boost::log::trivial::severity_level severity);

  /// @brief Print the logger header. Requires at least one HEEX_LOG(info) to be performed before calling this
  void printLoggerHeader();

  /// @brief Get the filepath for the current active log
  std::string getActiveLogFilepath();

private:
  Log();

  /// @brief Initialize logging. All the user has to do is call the function with it's prefered args and log using : HEEX_LOG(info) << "This is an informational severity message";
  ///
  /// @param executableName : Set executable name to be added each line. If empty, executableName is skiped.
  /// @param logInTerminal : If true, log are output to std::cout.
  /// @param severity : Level in the following order : trace, debug, info, warning, error, fatal.
  void init(const std::string& executableName, bool logInTeminal = true, boost::log::trivial::severity_level severity = boost::log::trivial::HEEX_MAX_LOG_LEVEL);

  std::string selectBestLogOption(const std::vector<std::pair<std::string, std::string>>& logPathOptions);

  /// @brief Cache the pointer to the file sink to facilitate its removal
  boost::shared_ptr<boost::log::sinks::synchronous_sink<boost::log::sinks::text_file_backend>> _file_sink;

  /// @brief Cache the current filename to the log file to facilitate its display
  std::string _fileSinkCurrentFileName;
};

static const auto& initiator = Log::instance();
} // namespace HeexUtils
