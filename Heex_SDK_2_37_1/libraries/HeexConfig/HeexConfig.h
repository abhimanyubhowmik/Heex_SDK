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
/// @author Charles Azarian
/// Contact: charles@heex.io
///
/// @date 2021-07-06
///
#pragma once

#include <string.h>

#include <map>
#include <mutex>
#include <string>
#include <vector>

///
/// @class HeexConfig
/// @brief Class used to read and gather data from a configuration file using a singleton design pattern.
class HeexConfig
{
public:
  /// @brief Static creation and management of HeexConfig singleton
  static HeexConfig& instance()
  {
    static HeexConfig singletonInstance;
    return singletonInstance;
  }

  // Delete copy constructor and assignment operator to prevent copying
  HeexConfig(const HeexConfig&)            = delete;
  HeexConfig& operator=(const HeexConfig&) = delete;

  /// @brief Clear the instance of HeexConfig
  void clear();

  /// @brief Parses the configuration file and stores the couples (key,value) in memory
  ///
  /// @param fileName Path to the file. Can be absolute or relative.
  void readConfFile(const std::string& fileName);

  /// @brief Extracts the value associated with the key from the parsed config file. Use mutex. Returns empty string as error.
  ///
  /// @param path Key name
  const std::string getConf(const std::string& path);

  /// @brief Extracts the string value associated with the key from the parsed config file. If the key doesn't exist, returns the default value.
  ///
  /// @param path Key
  /// @param defaultValue Value in case of inexisting key.
  const std::string getConf(const std::string& path, const std::string& defaultValue);

  /// @brief Converts the value associated with the key from a string to an int.
  ///
  /// @param path Key
  /// @param defaultValue Value in case of inexisting key.
  int getConf(const std::string& path, int defaultValue);

  /// @brief Converts the value associated with the key from a string to an unsigned short.
  ///
  /// @param path Key
  /// @param defaultValue Value in case of inexisting key.
  unsigned short getConf(const std::string& path, unsigned short defaultValue);
  /// @brief Converts the value associated with the key from a string to a double.
  ///
  /// @param path Key
  /// @param defaultValue Value in case of inexisting key.
  double getConf(const std::string& path, double defaultValue);

  /// @brief Converts the value associated with the key from a string to a float.
  ///
  /// @param path Key
  /// @param defaultValue Value in case of inexisting key.
  float getConf(const std::string& path, float defaultValue);

  /// @brief Constructs a string vector of valid pointers in the configuration file. Uses mutex. Returns nothing if there is no file.
  std::vector<std::string> getClasses();

  /// @brief Constructs a string vector of keys found within a class. Uses mutex. Returns nothing if there is no file.
  ///
  /// getKeys will return an error if a nonexisting file is used and will use the previous values.
  ///
  /// @param className A class in the configuration file.
  std::vector<std::string> getKeys(std::string className);

  ///@brief Sets the disableWarning acting on getConf function attribute to true or false.
  ///
  ///@param disableWarning
  void setDisableWarning(bool disableWarning) { _disableWarning = disableWarning; }

private:
  /// Empty constructor
  HeexConfig() = default;

  /// @brief Stores the couple (key,value) in memory.
  ///
  /// @param key the key
  /// @param value the data value to be associated with the key
  void addToConf(const std::string& key, std::string value);

  /// @brief Map containing each couple (key, value) as (std::string,std::string).
  ///
  /// @param _HeexConfig
  std::map<std::string, std::string> _HeexConfig;

  /// @brief Mutex preventing multi-access.
  ///
  /// @param _mtx
  std::mutex _mtx;

  bool _disableWarning{false};
};
