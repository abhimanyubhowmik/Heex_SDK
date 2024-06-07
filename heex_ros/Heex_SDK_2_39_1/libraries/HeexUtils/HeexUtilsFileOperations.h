///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file HeexUtilsFileOperations.h
/// @author Matthieu Carré (matthieu@heex.io)
/// @brief Header file for commonly used functions for file operations.
/// @version 1.0
/// @date 2022-03-14
#pragma once

#include <fstream>
#include <string>

// forward declaration of boost::filesystem::path;
namespace boost
{
namespace filesystem // NOLINT no CamelCase as it's forward declaration of existing name
{
class path;
}
} // namespace boost

///
/// @class HeexUtils
/// @brief Collection of commonly used functions for file operations.
namespace HeexUtils
{
namespace FileOperations
{
/// @brief Returns if the file exists. May only work on most platforms.
///
/// @param filepath Path to the file
/// @return true File exists.
/// @return false File doesn't exist or an error has occurred.
bool isFileExist(const std::string& filepath);

/// @brief Returns if the file exists and can be open. May work for all platforms.
///
/// @param filepath Path to the file
/// @return true File exists.
/// @return false File doesn't exist or an error has occurred.
bool isFileExistAndAccess(const std::string& filepath);

/// @brief Returns if the folder exists and can be open. May work for all platforms.
///
/// @param folderpath Path to the folder
/// @return true Folder exists.
/// @return false Folder doesn't exist or an error has occurred.
bool isFolderExistAndAccess(const std::string& folderpath);

/// @brief Write content to the file and replace content if it already exists.
///
/// @param filepath Path to the file.
/// @return true Content does match.
/// @return false File content doesn't exist or an error has occurred.
bool writeFileContent(const std::string& filepath, const std::string& content);

/// @brief Load content of the file and compare it to the provided content string.
///
/// @param filepath Path to the file.
/// @return true Content does match.
/// @return false File content doesn't exist or an error has occurred.
bool isFileContentMatch(const std::string& filepath, const std::string& content);

/// @brief Remove the file at the given filepath if it exists.
///
/// @param filepath Path to the file.
/// @return true Removal succeed.
/// @return false An error has occurred.
bool removeFileIfExist(const std::string& filepath);

/// @brief Create a folder at the provided path if it is possible.
///
/// @param folderpath Path to the folder.
/// @return true Creation succeed.
/// @return false An error has occurred.
bool createDirectory(const std::string& folderpath);

/// @brief expand the filepath with the home directory if filepath begins with tilde.
///
/// @param filepath Path to the file.
/// @return true if success
/// @return false otherwise
bool expandTilde(std::string& filepath);

/// @brief Copies file at srcPath to dstPath
///
/// @param srcPath std::string path encoded in utf8
/// @param dstPath std::string path encoded in utf8
/// @return true if success false otherwise
bool copyFileTo(const std::string& srcPath, const std::string& dstPath);

/// @brief Get the absolute path of the path (from the DataSender.conf) given as parameter
///
/// @param rawPath the path to check
/// @param dataSenderConfFolderAbsolutePath the absolute path of the DataSender.conf
/// @return the path to use, transformed in absolute if rawPath was relative
std::string getAbsolutePath(const std::string& rawPath, const std::string& dataSenderConfFolderAbsolutePath);

#ifdef _WIN32
/// @brief Convert a utf8 string to a utf16 wstring
///
/// @param str the utf8 string to convert
/// @return the utf16 wstring
std::wstring utf8ToUtf16(std::string str);
#endif

/// @brief Get the utf8 encoded path
///
/// @param p the boost::filesystem::path to encode
/// @return the string path encoded in utf8
std::string getUtf8EncodedPath(const boost::filesystem::path p);

/// @brief Get a std::ofstream using a utf8 encoded string filepath
///
/// @param filepath std::string filepath encoded in utf8
/// @param mode the mode to open the file
/// @return the std::ofstream
std::ofstream getOfstream(const std::string& filepath, std::ios_base::openmode mode = std::ofstream::out);

/// @brief Get a std::ofstream using a boost::filesystem::path
///
/// @param filepath the boost::filesystem::path
/// @param mode the mode to open the file
/// @return the std::ofstream
std::ofstream getOfstream(const boost::filesystem::path& filepath, std::ios_base::openmode mode = std::ofstream::out);

/// @brief Get a std::ifstream using a utf8 encoded string filepath
///
/// @param filepath std::string filepath encoded in utf8
/// @param mode the mode to open the file
/// @return the std::ifstream
std::ifstream getIfstream(const std::string& filepath, std::ios_base::openmode mode = std::ifstream::in);

/// @brief Get a std::ifstream using a boost::filesystem::path
///
/// @param filepath the boost::filesystem::path
/// @param mode the mode to open the file
/// @return the std::ifstream
std::ifstream getIfstream(const boost::filesystem::path& filepath, std::ios_base::openmode mode = std::ifstream::in);

/// @brief Get a const wchar_t * or a const char * depending on the platform
///
/// @param filepath std::string filepath encoded in utf8
/// @return the const wchar_t * or const char *
#ifdef _WIN32
std::wstring getCorrectlyEncodedPath(const std::string& filepath);
#else
std::string getCorrectlyEncodedPath(const std::string& filepath);
#endif

/// @brief Get the size of a file
///
/// @param filepath std::string filepath encoded in utf8
/// @return the size of the file if it exists, 0 otherwise
std::uintmax_t getFileSize(const std::string& filepath);

/// @brief Get the size of a file
///
/// @param filepath boost::filesystem::path
/// @return the size of the file if it exists, 0 otherwise
std::uintmax_t getFileSize(const boost::filesystem::path& filepath);

/// @brief Get the size of a folder
///
/// @param filepath std::string filepath encoded in utf8
/// @return the size of the folder if it exists, 0 otherwise
std::uintmax_t getFolderSize(const std::string& filepath);

///@brief Get the parent folder of a file
///
///@param filepath std::string filepath encoded in utf8
///@return the utf8 encoded parent folder of the file
std::string getParentFolder(const std::string& filepath);

/// @brief returns the filename or foldername of given path
///
/// @param path std::string path encoded in utf8
/// @return file or folder name
std::string getFileOrFolderName(const std::string& path);

/// @brief appends the 2 paths in order path1 / path2. No exist() checks, simply appends the paths
///
/// @param path1 std::string path encoded in utf8
/// @param path2 std::string path encoded in utf8
/// @return the utf8 encoded appended paths
std::string concatenatePaths(const std::string& path1, const std::string& path2);

///@brief checks if there is enough disk space at given path
///
/// @param diskPath boost::filesystem::path path to space we want to test
/// @param expectedDiskSpace [bytes] unsigned long long disk space needed
/// @return isDiskSpaceEnough
bool isDiskSpaceEnough(boost::filesystem::path diskPath, const unsigned long long expectedDiskSpace);

} // namespace FileOperations
} // namespace HeexUtils
