///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file HeexUtilsFileOperations.cpp
/// @author Matthieu Carré (matthieu@heex.io)
/// @brief Source file for commonly used functions for file operations.
/// @version 1.0
/// @date 2022-03-14

#ifdef _WIN32
  #include <windows.h>

  #include <codecvt>
#endif

#include <stdio.h>
#include <stdlib.h>

#include <boost/filesystem.hpp>
#include <iostream>
#include <sstream>

#include "HeexUtilsFileOperations.h"
#include "HeexUtilsLog.h"

bool HeexUtils::FileOperations::isFileExist(const std::string& filepath)
{
  if (filepath.empty())
  {
    HEEX_LOG(debug) << "HeexUtils::fileExists | File at path is empty, so doesn't exist." << std::endl;
    return false;
  }
  boost::filesystem::path p(HeexUtils::FileOperations::getCorrectlyEncodedPath(filepath));
  boost::system::error_code ec;
  if (boost::filesystem::exists(p, ec) && ec.value() == 0) // does p actually exist and check does not result in error?
  {
    if (boost::filesystem::is_regular_file(p, ec) && ec.value() == 0) // is p a file and check does not result in error?
    {
      // File already exists
      HEEX_LOG(debug) << "HeexUtils::fileExists | File at path " << p << " exists." << std::endl;
      return true;
    }
    HEEX_LOG(error) << "HeexUtils::fileExists | File at path " << p << " exists, but is not valid for a regular file\n";
    throw std::system_error(ec);
  }
  else if (ec.value() == boost::system::errc::errc_t::no_such_file_or_directory)
  {
    HEEX_LOG(debug) << "HeexUtils::fileExists | File at path " << p << " doesn't exist."
                    << "\n";
  }
  else
  {
    HEEX_LOG(error) << "HeexUtils::fileExists | An error occured when checking file at path " << p << ". Error message: " << ec.message() << "\n";
    throw std::system_error(ec);
  }
  return false;
}

bool HeexUtils::FileOperations::isFileContentMatch(const std::string& filepath, const std::string& content)
{
  // Try access file first
  std::ifstream mf = HeexUtils::FileOperations::getIfstream(filepath, std::fstream::in);

  // Is mutex file accessible ?
  if (!mf.is_open())
  {
    // File doesn't exist or an error has happened
    HEEX_LOG(error) << "HeexUtils::isFileContentMatch | Error reading file at " << filepath << std::endl;
    return false;
  }

  // File exist, Read content and Release file from reading
  std::stringstream buffer;
  buffer << mf.rdbuf();
  mf.close();
  std::string contentRead = buffer.str(); // str holds the content of the file

  // Is mutex locked ?
  HEEX_LOG(debug) << "HeexUtils::isFileContentMatch | Match between:" << std::endl << contentRead << std::endl << " == " << std::endl << content << std::endl << "?" << std::endl;
  return contentRead == content;
}

bool HeexUtils::FileOperations::removeFileIfExist(const std::string& filepath)
{
  // Remove file from temporary storage of DS
  boost::system::error_code ec;
  const boost::filesystem::path path{HeexUtils::FileOperations::getCorrectlyEncodedPath(filepath)};
  const uintmax_t res = boost::filesystem::remove_all(path, ec);
  if (ec == boost::system::errc::success)
  {
    HEEX_LOG(debug) << "HeexUtils::removeFileIfExist | Removed " << res << " files at " << getUtf8EncodedPath(path) << std::endl; // DEBUG
    return true;
  }
  else
  {
    HEEX_LOG(error) << "HeexUtils::removeFileIfExist | Error code on deletion : " << ec << "(" << ec.value() << ")" << std::endl;
    return false;
  }
}

bool HeexUtils::FileOperations::isFileExistAndAccess(const std::string& filepath)
{
  if (filepath.empty())
  {
    HEEX_LOG(debug) << "HeexUtils::isFileExistAndAccess | File at path is empty, so doesn't exist." << std::endl;
    return false;
  }
  boost::filesystem::path p(HeexUtils::FileOperations::getCorrectlyEncodedPath(filepath));
  boost::system::error_code ec;

  // Boost::Filesystem existence check. Might fail on ARM platforms.
  if (boost::filesystem::exists(p, ec) && ec.value() == 0) // does p actually exist and check does not result in error?
  {
    if (boost::filesystem::is_regular_file(p) && ec.value() == 0) // is p a file and check does not result in error?
    {
      return true;
    }
    else
    {
      HEEX_LOG(debug) << "HeexUtils::isFileExistAndAccess | Path " << p << "exists, but isn't a regular file\n";
      return false;
    }
  }
  else if (ec.value() == boost::system::errc::errc_t::no_such_file_or_directory)
  {
    HEEX_LOG(debug) << "HeexUtils::isFileExistAndAccess | File at path " << p << " doesn't exist. Error: " << ec.message() << "\n";
    return false;
  }

  // Fallback for specific ARM platforms
  /// Check if not a "Function not implemented" when OS does not fully support boost filesystem
  if (ec.value() == boost::system::errc::function_not_supported)
  {
    // Log if the previous method isn't supported on the platform (eg. ARM)
    HEEX_LOG(debug) << "HeexUtils::isFileExistAndAccess | Trying recovery as a filesystem error have been met: " << ec.message() << " (" << ec.value() << ")" << '\n';
    try
    {
      // Propose alternative to the check for file only (if platform can perform the above steps without "Function not implemented" error)
      // NOLINTBEGIN
      FILE* file = nullptr;
#if defined(_POSIX_VERSION)
      file = fopen(filepath.c_str(), "r");
      if (file != nullptr)
#elif defined(_WIN32) || defined(_WIN64)
      errno_t err = fopen_s(&file, filepath.c_str(), "r");
      if (err == 0)
#endif
      {
        fclose(file);
        return true;
      }
      // NOLINTEND
    }
    catch (const std::exception& e)
    {
      HEEX_LOG(error) << e.what() << '\n';
    }

    // Return false on failure to open or any exception
    return false;
  }

  // Error is no longer mitigable error. Returning false.
  HEEX_LOG(debug) << "HeexUtils::isFileExistAndAccess | Filesystem error have been met: " << ec.message() << " (" << ec.value() << ")" << '\n';
  return false;
}

bool HeexUtils::FileOperations::isFolderExistAndAccess(const std::string& folderpath)
{
  boost::filesystem::path p(HeexUtils::FileOperations::getCorrectlyEncodedPath(folderpath));
  boost::system::error_code ec;

  // Boost::Filesystem existence check. Might fail on ARM platforms.
  if (boost::filesystem::exists(p, ec) && ec.value() == 0) // does p actually exist and check does not result in error?
  {
    if (boost::filesystem::is_directory(p, ec) && ec.value() == 0) // is p a file and check does not result in error?
    {
      return true;
    }
    else
    {
      HEEX_LOG(error) << "HeexUtils::isFolderExistAndAccess | Path " << p << "exists, but isn't a directory\n";
      return false;
    }
  }
  else if (ec.value() == boost::system::errc::errc_t::no_such_file_or_directory)
  {
    HEEX_LOG(error) << "HeexUtils::isFolderExistAndAccess | File at path " << p << " doesn't exist. Error: " << ec.message() << "\n";
    return false;
  }

  /// TODO: Fallback for specific ARM platforms

  // Error is no longer mitigable error. Returning false.
  HEEX_LOG(error) << "HeexUtils::isFolderExistAndAccess | Filesystem error have been met: " << ec.message() << " (" << ec.value() << ")" << '\n';
  return false;
}

bool HeexUtils::FileOperations::writeFileContent(const std::string& filepath, const std::string& content)
{
  std::ofstream ofs = HeexUtils::FileOperations::getOfstream(filepath, std::ofstream::out);
  if (!ofs) // Check if the file was successfully opened for writing
  {
    return false;
  }

  ofs << content;

  if (ofs.fail()) // Check for any write errors
  {
    return false;
  }

  ofs.close();
  return !ofs.fail(); // Check for any close errors
}

bool HeexUtils::FileOperations::createDirectory(const std::string& folderpath)
{
  const boost::filesystem::path folderpath2 = folderpath;
  boost::system::error_code ec;
  // HEEX_LOG(debug) << "HeexUtils::createDirectory | Creating operating directory at  : " << getUtf8EncodedPath(folderpath2) << std::endl;
  boost::filesystem::create_directory(folderpath2, ec);
  if (ec.value() != boost::system::errc::success && ec.value() != boost::system::errc::file_exists)
  {
    HEEX_LOG(error) << "HeexUtils::createDirectory | Can't create folder at " + getUtf8EncodedPath(folderpath2.parent_path()) + ". Filesystem error caused by: " + ec.message()
                    << std::endl;
    return false;
  }
  ec.clear();
  return true;
}

bool HeexUtils::FileOperations::expandTilde(std::string& filepath)
{
  std::string homeDir;
#if defined(_POSIX_VERSION)
  homeDir = getenv("HOME");
#elif defined(_WIN32) || defined(_WIN64)
  char* buf = nullptr;
  size_t sz = 0;
  if (_dupenv_s(&buf, &sz, "USERPROFILE") == 0 && buf != nullptr)
  {
    homeDir = buf;
    free(buf);
    sz = 0;
  }
  else
  {
    std::string homeDrive;
    std::string homePath;
    if (_dupenv_s(&buf, &sz, "HOMEDRIVE") == 0 && buf != nullptr)
    {
      homeDrive = buf;
      free(buf);
      sz = 0;
    }
    if (_dupenv_s(&buf, &sz, "HOMEPATH") == 0 && buf != nullptr)
    {
      homePath = buf;
      free(buf);
      sz = 0;
    }
    homeDir = homeDrive + homePath;
  }
#else
  #error "Your Platform OS does not seems to be supported yet! Please contact your Heex referee to request a support of your platform."
#endif

  if (homeDir.empty())
  {
    HEEX_LOG(error) << "error: HOME variable not set." << std::endl;
    return false;
  }

  filepath = homeDir + filepath.substr(1, std::string::npos);
  return true;
}

std::string HeexUtils::FileOperations::getAbsolutePath(const std::string& rawPath, const std::string& dataSenderConfFolderAbsolutePath)
{
  // relative function is to resolve the '.' in the path, absolute is to get the absolute path and canonical let the path from 'my/path/../' to 'my/'
  if (rawPath.empty())
  {
    HEEX_LOG(error) << "HeexUtils::getAbsolutePath | rawPath is empty" << std::endl;
    return rawPath;
  }
  if (boost::filesystem::path(getCorrectlyEncodedPath(rawPath)).is_absolute())
  {
    HEEX_LOG(warning) << "HeexUtils::getAbsolutePath | rawPath is absolute" << std::endl;
    return rawPath;
  }
  else
  {
    // To convert uft8 string to boost path we must go through getCorrectlyEncodedPath
    boost::filesystem::path relative = boost::filesystem::path(getCorrectlyEncodedPath(rawPath));
    if (!dataSenderConfFolderAbsolutePath.empty())
    {
      relative = boost::filesystem::path(getCorrectlyEncodedPath(dataSenderConfFolderAbsolutePath)) / relative;
    }

    const boost::filesystem::path relativePath = boost::filesystem::relative(relative);
    const boost::filesystem::path absolutePath = boost::filesystem::absolute(relativePath);
    // We use getUtf8EncodedPath to convert boost path to utf8 string
    return getUtf8EncodedPath(absolutePath);
  }
}

#ifdef _WIN32
std::wstring HeexUtils::FileOperations::utf8ToUtf16(std::string str)
{
  std::wstring ret;
  int len = MultiByteToWideChar(CP_UTF8, 0, str.c_str(), str.length(), NULL, 0);
  if (len > 0)
  {
    ret.resize(len);
    MultiByteToWideChar(CP_UTF8, 0, str.c_str(), str.length(), &ret[0], len);
  }
  return ret;
}
#endif

#ifdef _WIN32
std::wstring HeexUtils::FileOperations::getCorrectlyEncodedPath(const std::string& filepath)
{
  return utf8ToUtf16(filepath);
}
#else
std::string HeexUtils::FileOperations::getCorrectlyEncodedPath(const std::string& filepath)
{
  return filepath;
}
#endif

std::string HeexUtils::FileOperations::getUtf8EncodedPath(const boost::filesystem::path p)
{
#ifdef _WIN32
  std::wstring utf16String = p.wstring();
  std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
  return converter.to_bytes(utf16String);
#else
  return p.string();
#endif
}

std::ofstream HeexUtils::FileOperations::getOfstream(const std::string& filepath, std::ios_base::openmode mode)
{
  std::ofstream ofs;
  ofs.open(getCorrectlyEncodedPath(filepath), mode);
  if (!ofs)
  {
    ofs.open(filepath, mode);
    if (!ofs)
    {
      HEEX_LOG(error) << "HeexUtils::getOfstream | Error opening file at " << filepath << " for writing." << std::endl;
      HEEX_LOG(error) << "HeexUtils::getOfstream | Error code: " << ofs.rdstate() << std::endl;
    }
  }
  return ofs;
}

std::ofstream HeexUtils::FileOperations::getOfstream(const boost::filesystem::path& filepath, std::ios_base::openmode mode)
{
  return getOfstream(getUtf8EncodedPath(filepath), mode);
}

std::ifstream HeexUtils::FileOperations::getIfstream(const std::string& filepath, std::ios_base::openmode mode)
{
  std::ifstream ifs;
  ifs.open(getCorrectlyEncodedPath(filepath), mode);
  if (!ifs)
  {
    ifs.open(filepath, mode);
    if (!ifs)
    {
      HEEX_LOG(error) << "HeexUtils::getIfstream | Error opening file at " << filepath << " for reading." << std::endl;
      HEEX_LOG(error) << "HeexUtils::getIfstream | Error code: " << ifs.rdstate() << std::endl;
    }
  }

  return ifs;
}

std::ifstream HeexUtils::FileOperations::getIfstream(const boost::filesystem::path& filepath, std::ios_base::openmode mode)
{
  return getIfstream(getUtf8EncodedPath(filepath), mode);
}

std::uintmax_t HeexUtils::FileOperations::getFileSize(const std::string& filepath)
{
  bool doesFileExist{false};
  try
  {
    doesFileExist = isFileExist(filepath);
  }
  catch (const std::exception& e)
  {
    HEEX_LOG(info) << "HeexUtils::getFileSize | " << e.what();
  }
  if (doesFileExist == false)
  {
    HEEX_LOG(info) << "HeexUtils::getFileSize | File at path " << filepath << " doesn't exist." << std::endl;
    return 0;
  }

  boost::system::error_code ec;
  const boost::filesystem::path path{HeexUtils::FileOperations::getCorrectlyEncodedPath(filepath)};
  const std::uintmax_t res = boost::filesystem::file_size(path, ec);
  if (ec == boost::system::errc::success)
  {
    HEEX_LOG(debug) << "HeexUtils::getFileSize | File at path " << getUtf8EncodedPath(path) << " has size " << res << std::endl; // DEBUG
    return res;
  }
  else
  {
    HEEX_LOG(error) << "HeexUtils::getFileSize | Error code on getting file size : " << ec << "(" << ec.value() << ")" << std::endl;
    return 0;
  }
}

std::uintmax_t HeexUtils::FileOperations::getFileSize(const boost::filesystem::path& filepath)
{
  return getFileSize(getUtf8EncodedPath(filepath));
}

std::uintmax_t HeexUtils::FileOperations::getFolderSize(const std::string& filepath)
{
  if (isFolderExistAndAccess(filepath) == false)
  {
    HEEX_LOG(info) << "HeexUtils::getFolderSize | Folder at path " << filepath << " doesn't exist." << std::endl;
    return 0;
  }
  std::uintmax_t totalSize = 0;
  boost::filesystem::recursive_directory_iterator it(HeexUtils::FileOperations::getCorrectlyEncodedPath(filepath));
  const boost::filesystem::recursive_directory_iterator endit;

  while (it != endit)
  {
    boost::system::error_code ec;
    if (boost::filesystem::is_regular_file(it->path(), ec))
    {
      totalSize += boost::filesystem::file_size(it->path(), ec);
    }
    if (ec.value() != 0)
    {
      HEEX_LOG(error) << "HeexUtils::getFolderSize | Error code on getting file size : " << ec << "(" << ec.value() << ")" << std::endl;
    }

    try
    {
      ++it;
    }
    catch (const std::exception& e)
    {
      HEEX_LOG(error) << "HeexUtils::getFolderSize | " << e.what();
      break;
    }
  }

  return totalSize;
}

std::string HeexUtils::FileOperations::getParentFolder(const std::string& filepath)
{
  const boost::filesystem::path p(HeexUtils::FileOperations::getCorrectlyEncodedPath(filepath));
  return getUtf8EncodedPath(p.parent_path());
}

std::string HeexUtils::FileOperations::getFileOrFolderName(const std::string& path)
{
  const boost::filesystem::path p(HeexUtils::FileOperations::getCorrectlyEncodedPath(path));

  std::string name{""};
  // Extract filename or folder name:
  if ("." != p.filename()) // path doesn't end with a trailing '/'
  {
    name = HeexUtils::FileOperations::getUtf8EncodedPath(p.filename());
  }
  else
  {
    name = HeexUtils::FileOperations::getUtf8EncodedPath(p.parent_path().filename());
  }

  return name;
}

std::string HeexUtils::FileOperations::concatenatePaths(const std::string& path1, const std::string& path2)
{
  boost::filesystem::path p1(HeexUtils::FileOperations::getCorrectlyEncodedPath(path1));
  const boost::filesystem::path p2(HeexUtils::FileOperations::getCorrectlyEncodedPath(path2));
  p1 /= p2;

  return HeexUtils::FileOperations::getUtf8EncodedPath(p1);
}

bool HeexUtils::FileOperations::copyFileTo(const std::string& srcPath, const std::string& dstPath)
{
  if (!HeexUtils::FileOperations::isFileExist(srcPath))
  {
    HEEX_LOG(error) << "HeexUtils::copyFileTo | source file does not exist : " << srcPath << std::endl;
    return false;
  }
  const boost::filesystem::path fsSrcPath(HeexUtils::FileOperations::getCorrectlyEncodedPath(srcPath));
  const boost::filesystem::path fsDstPath(HeexUtils::FileOperations::getCorrectlyEncodedPath(dstPath));
  boost::system::error_code ec;

  boost::filesystem::copy_file(fsSrcPath, fsDstPath, boost::filesystem::copy_option::overwrite_if_exists, ec);

  if (ec == boost::system::errc::success)
  {
    HEEX_LOG(debug) << "HeexUtils::copyFileTo | Successfully copied file to " << getUtf8EncodedPath(fsDstPath) << std::endl;
    return true;
  }
  else
  {
    HEEX_LOG(error) << "HeexUtils::copyFileTo | Failed copying file to " << getUtf8EncodedPath(fsDstPath) << std::endl;
    return false;
  }
}

bool HeexUtils::FileOperations::isDiskSpaceEnough(boost::filesystem::path diskPath, const unsigned long long expectedDiskSpace)
{
  boost::filesystem::path tempDir = diskPath.parent_path() / "tmpDirDiskSpace";
  boost::system::error_code ec;
  boost::filesystem::create_directories(tempDir, ec);
  if (ec.value() != boost::system::errc::success && ec.value() != boost::system::errc::file_exists)
  {
    // let's try inside given path
    tempDir = diskPath / "tmpDirDiskSpace";
    boost::filesystem::create_directories(tempDir, ec);
    if (ec.value() != boost::system::errc::success && ec.value() != boost::system::errc::file_exists)
    {
      HEEX_LOG(error) << "HeexUtils::FileOperations::isDiskSpaceEnough | Can't create folder around " << diskPath.string() << ". Filesystem error caused by: " << ec.message()
                      << std::endl;
      return false;
    }
  }

  boost::filesystem::space_info spaceInfo{};
  try
  {
    spaceInfo = boost::filesystem::space(tempDir);
  }
  catch (const std::exception& e)
  {
    HEEX_LOG(error) << "HeexUtils::FileOperations::isDiskSpaceEnough | Error when checking disk space caused by" << e.what() << std::endl;
    boost::filesystem::remove(tempDir);
    return false;
  }
  boost::filesystem::remove(tempDir);
  if (spaceInfo.available < expectedDiskSpace)
  {
    HEEX_LOG(info) << "HeexUtils::FileOperations::isDiskSpaceEnough | Expected disk space (" << expectedDiskSpace << ") is bigger than the space available (" << spaceInfo.available
                   << ")" << std::endl;
    return false;
  }
  return true;
}
