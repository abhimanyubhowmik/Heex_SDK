///
/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file HeexUtilsFileLock.cpp
/// @author Quentin Souvignet (quentin@heex.io)
/// @brief Source file for utf8 compatible file_lock from bitcoin core implementation.
/// @version 1.0
/// @date 2023-09-29
#include "HeexUtilsFileLock.h"

#include <string>

#include "HeexUtilsFileOperations.h"
#include "HeexUtilsLog.h"

#ifdef _WIN32
  #define NOMINMAX
  #include <windows.h>

  #include <codecvt>
  #include <limits>
#else
  #include <fcntl.h>
#endif

namespace HeexUtils
{

#ifdef _WIN32

static std::string GetErrorReason()
{
  wchar_t* err;
  FormatMessageW(
      FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
      nullptr,
      GetLastError(),
      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
      reinterpret_cast<WCHAR*>(&err),
      0,
      nullptr);
  std::wstring err_str(err);
  LocalFree(err);
  return std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>>().to_bytes(err_str);
}

FileLock::FileLock(const boost::filesystem::path& file)
{
  const std::string mutexFilepath = HeexUtils::FileOperations::getUtf8EncodedPath(file);
  if (HeexUtils::FileOperations::isFileExistAndAccess(mutexFilepath) == false)
  {
    HEEX_LOG(trace) << "FileLock::FileLock | ReCreating lock file at " << mutexFilepath << std::endl;
    std::ofstream o = HeexUtils::FileOperations::getOfstream(mutexFilepath, std::ios_base::app);
  }

  _hFile = CreateFileW(
      file.wstring().c_str(), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (_hFile == INVALID_HANDLE_VALUE)
  {
    _reason = GetErrorReason();
  }
}

FileLock::~FileLock()
{
  if (_hFile != INVALID_HANDLE_VALUE)
  {
    CloseHandle(_hFile);
  }
}

bool FileLock::tryLock()
{
  if (_hFile == INVALID_HANDLE_VALUE)
  {
    return false;
  }
  _OVERLAPPED overlapped = {};
  if (!LockFileEx(_hFile, LOCKFILE_EXCLUSIVE_LOCK | LOCKFILE_FAIL_IMMEDIATELY, 0, std::numeric_limits<DWORD>::max(), std::numeric_limits<DWORD>::max(), &overlapped))
  {
    _reason = GetErrorReason();
    return false;
  }
  return true;
}

bool FileLock::lock()
{
  if (_hFile == INVALID_HANDLE_VALUE)
  {
    return false;
  }

  _OVERLAPPED overlapped = {};

  while (true)
  {
    if (LockFileEx(_hFile, LOCKFILE_EXCLUSIVE_LOCK, 0, std::numeric_limits<DWORD>::max(), std::numeric_limits<DWORD>::max(), &overlapped))
    {
      return true; // Lock acquired successfully
    }

    DWORD lastError = GetLastError();
    if (lastError != ERROR_LOCK_VIOLATION)
    {
      _reason = GetErrorReason();
      return false; // An error other than a lock violation occurred
    }

    Sleep(100); // Wait for a short duration before retrying
  }
}
#else

static std::string getErrorReason()
{
  return std::strerror(errno);
}

FileLock::FileLock(const boost::filesystem::path& file)
{
  const std::string mutexFilepath = HeexUtils::FileOperations::getUtf8EncodedPath(file);
  if (HeexUtils::FileOperations::isFileExistAndAccess(mutexFilepath) == false)
  {
    HEEX_LOG(trace) << "FileLock::FileLock | ReCreating lock file at " << mutexFilepath << std::endl;
    const std::ofstream o = HeexUtils::FileOperations::getOfstream(mutexFilepath, std::ios_base::app);
  }

  _fd = open(file.string().c_str(), O_RDWR);
  if (_fd == -1)
  {
    _reason = getErrorReason();
  }
}

FileLock::~FileLock()
{
  if (_fd != -1)
  {
    close(_fd);
  }
}

bool FileLock::tryLock()
{
  if (_fd == -1)
  {
    return false;
  }
  struct flock lock
  {
  };
  lock.l_type   = F_WRLCK;
  lock.l_whence = SEEK_SET;
  lock.l_start  = 0;
  lock.l_len    = 0;
  if (fcntl(_fd, F_SETLK, &lock) == -1)
  {
    _reason = getErrorReason();
    return false;
  }
  return true;
}

bool FileLock::lock()
{
  if (_fd == -1)
  {
    return false;
  }

  struct flock fl
  {
  };
  std::memset(&fl, 0, sizeof(fl));

  fl.l_type   = F_WRLCK;
  fl.l_whence = SEEK_SET;
  fl.l_start  = 0;
  fl.l_len    = 0;

  while (fcntl(_fd, F_SETLKW, &fl) == -1)
  {
    if (errno != EINTR)
    {
      // An error other than being interrupted occurred
      _reason = std::strerror(errno);
      return false;
    }
  }

  return true; // Lock acquired successfully
}
#endif

} // namespace HeexUtils
