///
/// Copyright (c) 2024 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "HeexUtilsExeRunner.h"

#include <boost/filesystem.hpp>
#include <chrono>
#include <codecvt>
#include <locale>
#include <thread>

#include "HeexUtilsFileOperations.h"
#include "HeexUtilsLog.h"

#if defined(_POSIX_VERSION) || defined(__linux__)
  #include <sys/wait.h>
  #include <unistd.h>
#elif defined(_WIN32) || defined(_WIN64)
  #include "Windows.h"
#endif

namespace HeexUtils
{
int ExeRunner::setExecutablePath(const std::string& executablePath)
{
  if (!HeexUtils::FileOperations::isFileExistAndAccess(executablePath))
  {
    HEEX_LOG(error) << "HeexUtils::ExeRunner | couldn't find executable file :" << executablePath << std::endl;
    return EXIT_FAILURE;
  }
  _executablePath = executablePath;
  return EXIT_SUCCESS;
}

int ExeRunner::setCommandLine()
{
  if (_executablePath.empty())
  {
    HEEX_LOG(error) << "No executablePath has been set, impossible to setup a commandline. Please call setExecutablePath()" << std::endl;
    return EXIT_FAILURE;
  }
  _commandLine = "\"" + boost::filesystem::absolute(_executablePath).string() + "\"";
  // If user has added arguments to the process, we concatenate them into a single 'commandLine' string
  for (const auto& arg : _argumentList)
  {
    _commandLine += " \"" + arg + "\"";
  }
  return EXIT_SUCCESS;
}

int ExeRunner::runExecutableAsDetachedProcess(const bool runAsAdmin /* = true */)
{
  if (this->setCommandLine() != EXIT_SUCCESS)
  {
    return EXIT_FAILURE;
  }
#if defined(_POSIX_VERSION) || defined(__linux__)
  if (runAsAdmin)
  {
    if (geteuid() == 0) // admin rights are available
    {
      _commandLine = "sudo " + _commandLine;
    }
    else
    {
      HEEX_LOG(warning) << "trying to run exeRunner with admin rights, program is not running with priviledged rights. Running command without sudo." << std::endl;
    }
  }
  const int ret = HeexUtils::ExeRunner::runLinuxDetachedProcess(_commandLine);
#elif defined(_WIN32) || defined(_WIN64)
  const int ret = HeexUtils::ExeRunner::runWindowsDetachedProcess(_commandLine);
#endif
  return ret;
}

int ExeRunner::runExecutableAsService(const std::string& serviceName)
{
  if (this->setCommandLine() != EXIT_SUCCESS)
  {
    return EXIT_FAILURE;
  }
#if defined(_POSIX_VERSION) || defined(__linux__)
  // We need to create a service file
  const std::string currPath          = HeexUtils::FileOperations::getUtf8EncodedPath(boost::filesystem::current_path());
  const std::string pathToServiceFile = HeexUtils::FileOperations::concatenatePaths(currPath, serviceName + ".service");
  if (!createLinuxServiceFile(pathToServiceFile))
  {
    HEEX_LOG(error) << "HeexUtils::runExecutableAsService | Couldn't create a service file" << std::endl;
    return EXIT_FAILURE;
  }
  const int ret = runExecutableAsLinuxService(serviceName, pathToServiceFile);
  HeexUtils::FileOperations::removeFileIfExist(pathToServiceFile);
#elif defined(_WIN32) || defined(_WIN64)
  const int ret = runExecutableAsWindowsService(serviceName);
#endif
  return ret;
}

int ExeRunner::removeService(const std::string& serviceName)
{
  HEEX_LOG(debug) << "ExeRunner::removeService | Removing service : " << serviceName << std::endl;
#if defined(_POSIX_VERSION) || defined(__linux__)
  const std::string checkServiceCmd = "sudo systemctl list-unit-files | grep " + serviceName;
  const std::string stopCommand     = "sudo systemctl stop " + serviceName;
  const std::string removeCommand   = "sudo systemctl disable " + serviceName;
#elif defined(_WIN32) || defined(_WIN64)
  const std::string checkServiceCmd = "sc.exe query " + serviceName;
  const std::string stopCommand     = "sc.exe stop " + serviceName;
  const std::string removeCommand   = "sc.exe delete " + serviceName;
#endif
  int isServiceExistReturn = std::system(checkServiceCmd.c_str());
  if (isServiceExistReturn != 0)
  {
    HEEX_LOG(debug) << "ExeRunner::removeService | Service was already not present : " << serviceName << std::endl;
    return EXIT_SUCCESS;
  }

  // Now we run stop and remove. No checks on return code since service could already have been stopped or disabled and only one operation was missing
  static_cast<void>(std::system(stopCommand.c_str()));
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  static_cast<void>(std::system(removeCommand.c_str()));
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

#if defined(_POSIX_VERSION) || defined(__linux__)
  // Remove the .service file
  const std::string rmServiceFileCmd = "sudo rm /etc/systemd/system/" + serviceName + ".service > /dev/null 2>&1";
  static_cast<void>(std::system(rmServiceFileCmd.c_str()));
  // On linux, we have to execute 'sudo systemctl daemon-reload' to apply the removal
  const std::string serviceCmd = "sudo systemctl daemon-reload";
  static_cast<void>(std::system(serviceCmd.c_str()));
#endif
  // check if it's still here
  isServiceExistReturn = std::system(checkServiceCmd.c_str());

  if (isServiceExistReturn != 0)
  {
    HEEX_LOG(debug) << "ExeRunner::removeService | Service removed. " << std::endl;
    return EXIT_SUCCESS;
  }
  else
  {
    HEEX_LOG(error) << "ExeRunner::removeService | Failed to remove the service " << serviceName << std::endl;
    return EXIT_FAILURE;
  }
}

#if defined(_POSIX_VERSION) || defined(__linux__)
// LINUX SPECIFIC FUNCTIONS :

bool ExeRunner::createLinuxServiceFile(const std::string& pathToServiceFile)
{
  std::stringstream serviceFileContent;

  serviceFileContent << "[Unit]" << std::endl;
  serviceFileContent << "Description=HeexUtils::ExeRunner::createLinuxServiceFile created service" << std::endl;
  serviceFileContent << std::endl;
  serviceFileContent << "[Service]" << std::endl;
  serviceFileContent << "Type=simple" << std::endl;
  serviceFileContent << "ExecStart=/bin/bash -c '" << _commandLine << "'" << std::endl;
  serviceFileContent << "WorkingDirectory=" << boost::filesystem::current_path().string() << std::endl;
  serviceFileContent << "User=root" << std::endl; // TODO: if needed, this can become more tuneable and set a user
  serviceFileContent << "Restart=on-failure" << std::endl;
  serviceFileContent << std::endl;
  serviceFileContent << "[Install]" << std::endl;
  serviceFileContent << "WantedBy=default.target" << std::endl;

  return HeexUtils::FileOperations::writeFileContent(pathToServiceFile, serviceFileContent.str());
}

int ExeRunner::runLinuxDetachedProcess(const std::string& commandLine)
{
  HEEX_LOG(info) << "HeexUtils::runLinuxDetachedProcess | Running command : '" << commandLine << "'" << std::endl;
  auto nohupCommand = "nohup " + commandLine + " > /dev/null 2>&1 &";
  auto ret          = std::system(nohupCommand.c_str());
  // since we run in a completely other process, we don't check the output of the given command.
  // this could potentially be improved but it would likely induce forking, redirecting the output, and reading the output.
  return ret;
}

int ExeRunner::runExecutableAsLinuxService(const std::string& serviceName, const std::string& pathToServiceFile)
{
  // Execute 'sudo systemctl daemon-reload'
  std::string serviceCmd = "sudo systemctl daemon-reload";
  int result             = std::system(serviceCmd.c_str());
  if (result != EXIT_SUCCESS)
  {
    HEEX_LOG(error) << "Error reloading systemd daemon" << std::endl;
    return EXIT_FAILURE;
  }
  // copy file into /etc/systemd/system/
  const std::string copyCmd = "sudo cp " + pathToServiceFile + " /etc/systemd/system/";
  result                    = std::system(copyCmd.c_str());

  // Execute 'sudo systemctl enable pathToServiceFile'
  if (result == EXIT_SUCCESS)
  {
    // If copy succeeded, we only need to enable via the service name
    serviceCmd = "sudo systemctl enable " + serviceName;
  }
  else
  {
    // else we can try and enable the full service path
    serviceCmd = "sudo systemctl enable " + pathToServiceFile;
  }
  result = std::system(serviceCmd.c_str());
  if (result != EXIT_SUCCESS)
  {
    HEEX_LOG(error) << "Error enabling service : " << serviceCmd << std::endl;
    return EXIT_FAILURE;
  }

  // Execute 'sudo systemctl start serviceName'
  serviceCmd = "sudo systemctl start " + serviceName;
  result     = std::system(serviceCmd.c_str());
  if (result != EXIT_SUCCESS)
  {
    HEEX_LOG(error) << "Error starting service : " << serviceCmd << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

#elif defined(_WIN32) || defined(_WIN64)
// WINDOWS SPECIFIC FUNCTIONS :
void ExeRunner::setNssmPath(const std::string& pathToNssm)
{
  boost::filesystem::path p(pathToNssm);
  _pathToNssm = HeexUtils::FileOperations::getUtf8EncodedPath(p);
};

int ExeRunner::runWindowsDetachedProcess(const std::string& commandLine)
{
  const std::string processCmd = "cmd /c start \"\" " + commandLine;
  HEEX_LOG(info) << "HeexUtils::runWindowsDetachedProcess | Running command : '" << processCmd << "'" << std::endl;
  const auto ret = std::system(processCmd.c_str());

  if (ret == -1)
  {
    HEEX_LOG(error) << "HeexUtils::runWindowsDetachedProcess | Command execution failed" << std::endl;
    return EXIT_FAILURE;
  }

  HEEX_LOG(info) << "HeexUtils::runWindowsDetachedProcess | Command successfully ran." << std::endl;
  return EXIT_SUCCESS;
}

int ExeRunner::runWindowsProcessWaitFinish(const std::string& commandLine)
{
  STARTUPINFOA si;
  PROCESS_INFORMATION pi;

  ZeroMemory(&si, sizeof(si));
  si.cb = sizeof(si);
  ZeroMemory(&pi, sizeof(pi));

  // Create a pipe for the child process's output
  HANDLE hChildStdoutRead, hChildStdoutWrite;
  SECURITY_ATTRIBUTES saAttr;
  saAttr.nLength              = sizeof(SECURITY_ATTRIBUTES);
  saAttr.bInheritHandle       = TRUE;
  saAttr.lpSecurityDescriptor = NULL;
  if (!CreatePipe(&hChildStdoutRead, &hChildStdoutWrite, &saAttr, 0))
  {
    HEEX_LOG(error) << "HeexUtils::runWindowsProcessA | CreatePipe failed" << std::endl;
    return EXIT_FAILURE;
  }

  // Redirect the child process's standard output to the write end of the pipe
  si.dwFlags |= STARTF_USESTDHANDLES;
  si.hStdOutput = hChildStdoutWrite;
  si.hStdError  = hChildStdoutWrite;

  // Create a new process to run commandLine
  if (!CreateProcessA(NULL, const_cast<LPSTR>(commandLine.c_str()), NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi))
  {
    HEEX_LOG(error) << "HeexUtils::runWindowsProcessA | Failed to create the process using CreateProcess" << std::endl;
    CloseHandle(hChildStdoutRead);
    CloseHandle(hChildStdoutWrite);
    return EXIT_FAILURE;
  }
  CloseHandle(hChildStdoutWrite);

  // Wait for the child process to finish
  if (WaitForSingleObject(pi.hProcess, INFINITE) == WAIT_FAILED)
  {
    HEEX_LOG(error) << "HeexUtils::runWindowsProcessA | Error while waiting for the process" << std::endl;
    CloseHandle(hChildStdoutRead);
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
    return EXIT_FAILURE;
  }

  // Close handles
  CloseHandle(hChildStdoutRead);
  CloseHandle(pi.hProcess);
  CloseHandle(pi.hThread);

  return EXIT_SUCCESS;
}

int ExeRunner::runExecutableAsWindowsService(const std::string& serviceName)
{
  if (_pathToNssm.empty())
  {
    // check if nssm is installed
    if (std::system("nssm --version") == EXIT_SUCCESS)
    {
      _pathToNssm = "nssm";
    }
    else if (std::system("nssm.exe --version") == EXIT_SUCCESS)
    {
      _pathToNssm = "nssm.exe";
    }
    else
    {
      // can't find nssm, we fail
      HEEX_LOG(error) << "Couldn't find the path to nssm executable" << std::endl;
      return EXIT_FAILURE;
    }
  }
  if (std::system(std::string("\"" + _pathToNssm + "\" --version").c_str()) == EXIT_FAILURE)
  {
    // can't find nssm, we fail
    HEEX_LOG(error) << _pathToNssm << " is not a runnable nssm executable" << std::endl;
    return EXIT_FAILURE;
  }

  const std::string installCmd = "\"" + _pathToNssm + "\" install " + serviceName + " " + _commandLine;
  HEEX_LOG(debug) << "Service install command:  " << installCmd << std::endl;
  int result = runWindowsProcessWaitFinish(installCmd);
  if (result != EXIT_SUCCESS)
  {
    HEEX_LOG(error) << "Error installing " << serviceName << " service using NSSM" << std::endl;
    return EXIT_FAILURE;
  }

  const char* environment_paths   = std::getenv("PATH");
  const std::string setAppEnvPath = "\"" + _pathToNssm + "\" set " + serviceName + " AppEnvironmentExtra PATH=\"" + environment_paths + "\"";
  HEEX_LOG(debug) << "Service set AppEnvironmentExtra command:  " << setAppEnvPath << std::endl;
  result = runWindowsProcessWaitFinish(setAppEnvPath);
  if (result != EXIT_SUCCESS)
  {
    HEEX_LOG(error) << "Error setting the " << serviceName << "'s service environment paths" << std::endl;
    return EXIT_FAILURE;
  }

  // Now we set the AppExit to 'Exit' so that it only runs once and not in a loop
  const std::string setAppExitCmd = "\"" + _pathToNssm + "\" set " + serviceName + " AppExit Default Exit";
  HEEX_LOG(debug) << "Service set AppExit command:  " << setAppExitCmd << std::endl;
  result = runWindowsProcessWaitFinish(setAppExitCmd);
  if (result != EXIT_SUCCESS)
  {
    HEEX_LOG(error) << "Error setting the " << serviceName << "'s service AppExit to 'Exit'" << std::endl;
    return EXIT_FAILURE;
  }

  const std::string startCmd = "\"" + _pathToNssm + "\" start " + serviceName;
  HEEX_LOG(debug) << "Service start command:  " << startCmd << std::endl;
  result = runWindowsProcessWaitFinish(startCmd);
  if (result != EXIT_SUCCESS)
  {
    HEEX_LOG(error) << "Error starting " << serviceName << " service using NSSM" << std::endl;
    return EXIT_FAILURE; // Exit with error
  }
  HEEX_LOG(info) << "successfully started a NSSM service." << std::endl;

  return EXIT_SUCCESS;
}
#endif

} // namespace HeexUtils
