///
/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file HeexUtilsFileLock.h
/// @author Quentin Souvignet (quentin@heex.io)
/// @brief Source file for utf8 compatible file_lock from bitcoin core implementation.
/// @version 1.0
/// @date 2023-09-29
#pragma once

#include <stdio.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/detail/utf8_codecvt_facet.hpp>
#include <boost/filesystem/fstream.hpp>
#include <string>

namespace HeexUtils
{
class FileLock
{
public:
  FileLock()                = delete;
  FileLock(const FileLock&) = delete;
  FileLock(FileLock&&)      = delete;

  /// @brief Construct a new File Lock object
  /// Check lock file existence. Create one if doesn't exist. Guard from invalid path by throwing exception throw.
  /// @param file
  explicit FileLock(const boost::filesystem::path& file);
  ~FileLock();
  bool tryLock();
  bool lock();
  std::string getReason() { return _reason; }

private:
  std::string _reason;
#ifndef _WIN32
  int _fd = -1;
#else
  void* _hFile = (void*)-1; // INVALID_HANDLE_VALUE
#endif
};

} // namespace HeexUtils
