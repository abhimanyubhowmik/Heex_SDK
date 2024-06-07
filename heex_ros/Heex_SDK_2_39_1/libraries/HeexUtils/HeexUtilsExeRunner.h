///
/// Copyright (c) 2024 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#pragma once
#include <string>
#include <vector>

class TestsHeexUtilsExeRunner;

namespace HeexUtils
{
///
/// @brief The ExeRunner class
/// @details This class is used to run a sepcific executable with given arguments.
///          It allows exe to either be run as a process or as a service. It is compiled based on the platform (windows / linux)
class ExeRunner
{
public:
  friend class ::TestsHeexUtilsExeRunner;

  ///@brief Constructor of the ExeRunner class
  ExeRunner()          = default;
  virtual ~ExeRunner() = default;

  ExeRunner(const ExeRunner&)            = delete; // Copy construct
  ExeRunner(ExeRunner&&)                 = delete; // Move construct
  ExeRunner& operator=(const ExeRunner&) = delete; // Copy assign
  ExeRunner& operator=(ExeRunner&&)      = delete; // Move assign

  ///@brief Runs an executable with given arguments as a detached process.
  ///@details Runs on Linux by calling 'nohup', on windows by calling 'start /b'
  ///
  ///@param runAsAdmin [Optional] run with elevated rights or not. Default true. No impact on windows
  ///@return 0 success, 1 failed
  virtual int runExecutableAsDetachedProcess(const bool runAsAdmin = true);

  ///@brief Runs an executable with given arguments as a service. If on windows, the path to nssm.exe needs to be set if not in env variable using setNssmPath()
  ///
  ///@param serviceName
  ///@return 0 success, 1 failed
  virtual int runExecutableAsService(const std::string& serviceName);

  ///@brief Sets the path to the executable that shall be run
  ///
  ///@param executablePath
  ///@return 0 success, 1 failed
  int setExecutablePath(const std::string& executablePath);

  ///@brief Sets the arguments that are needed for the exe
  ///
  ///@param argumentList
  void setArgumentList(const std::vector<std::string>& argumentList) { _argumentList = argumentList; };

  ///@brief Removes given service. If it was already not present, simply returns success
  ///
  ///@param serviceName
  ///@return 0 success, 1 failed
  virtual int removeService(const std::string& serviceName);

#if defined(_WIN32) || defined(_WIN64)
  ///@brief sets the _pathToNssm variable (expected full path to nssm.exe file with filename and extension)
  ///
  ///@param pathToNssm
  void setNssmPath(const std::string& pathToNssm);
#endif

private:
  ///@brief Sets the full commandLine for given executablePath and arguments
  ///
  ///@return 0 success, 1 failed
  int setCommandLine();

#if defined(_POSIX_VERSION) || defined(__linux__)
  ///@brief Create a Service File object
  ///
  ///@param pathToServiceFile - includes the file+extension
  ///@param serviceName
  ///@return 0 success, 1 failed
  bool createLinuxServiceFile(const std::string& pathToServiceFile);

  ///@brief Runs the given commandLine via std::system using unix's 'nohup' command
  ///
  ///@param commandLine commandLine ran by std::system inside nohup
  ///@return 0 success, 1 failed
  int runLinuxDetachedProcess(const std::string& commandLine);

  ///@brief Runs the given commandLine as a service via systemctl
  ///
  ///@param serviceName
  ///@return 0 success, 1 failed
  int runExecutableAsLinuxService(const std::string& serviceName, const std::string& pathToServiceFile);

#elif defined(_WIN32) || defined(_WIN64)
  ///@brief Runs the given commandLine as a service via systemctl
  ///
  ///@param serviceName
  ///@return 0 success, 1 failed
  int runExecutableAsWindowsService(const std::string& serviceName);

  ///@brief calls given commandLine via CreateProcessA and waits for the process to finish before returning a value
  ///
  ///@param commandLine
  ///@return 0 success, 1 failed
  int runWindowsProcessWaitFinish(const std::string& commandLine);

  ///@brief Runs the given commandLine via std::system using unix's 'nohup' command
  ///
  ///@param commandLine commandLine ran by std::system inside nohup
  ///@return 0 success, 1 failed
  int runWindowsDetachedProcess(const std::string& commandLine);
#endif

  std::string _commandLine;               ///< commandLine that shall be run by either the process or service
  std::string _executablePath;            ///< full path to the exe that needs running
  std::vector<std::string> _argumentList; ///< list of potential arguments to be added to the command
#if defined(_WIN32) || defined(_WIN64)
  std::string _pathToNssm; ///< full path to nssm.exe file
#endif
};

} // namespace HeexUtils
