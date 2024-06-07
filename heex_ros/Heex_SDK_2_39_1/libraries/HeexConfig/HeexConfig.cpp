///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "HeexConfig.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "HeexUtilsFileOperations.h"
#include "HeexUtilsLog.h"

void HeexConfig::clear()
{
  const std::lock_guard<std::mutex> lock(_mtx);
  _HeexConfig.clear();
}

void HeexConfig::readConfFile(const std::string& fileName)
{
  std::ifstream infile = HeexUtils::FileOperations::getIfstream(fileName);
  std::string path;

  if (infile.is_open() == true)
  {
    std::string line;
    while (std::getline(infile, line))
    {
      if (line != "" && line[0] != '#')
      {
        line = line.substr(0, line.find("//"));

        if (line[0] == '*')
        {
          path = line.substr(line.find_first_of('*') + 1, std::string::npos) + "/";
        }
        else
        {
          const size_t lineStart = line.find_first_not_of(" \t");
          if (lineStart != std::string::npos)
          {
            line = line.substr(lineStart, std::string::npos);
          }

          const std::string key(path + line.substr(0, line.find_first_of(' ')));
          line = line.substr(line.find_first_of(' ') + 1, std::string::npos);
          std::string value(line.substr(0, line.find_first_of("#\t\n")));

          if (value.size() > 0)
          {
            int cutHere = static_cast<int>(value.size()) - 1;
            while (value[cutHere] == ' ' && cutHere > 0)
            {
              cutHere--;
            }
            cutHere++;

            value = value.substr(0, cutHere);
          }

          this->addToConf(key, value);
        }
      }
    }
    infile.close();
  }
  else
  {
    HEEX_LOG(error) << "Error while opening file : " << fileName << std::endl;
  }
}

const std::string HeexConfig::getConf(const std::string& path)
{
  const std::lock_guard<std::mutex> lock(_mtx);
  for (std::map<std::string, std::string>::iterator it = _HeexConfig.begin(); it != _HeexConfig.end(); ++it)
  {
    if (it->first == path)
    {
      return it->second;
    }
  }

  if (_disableWarning == false)
  {
    HEEX_LOG(warning) << "HeexConfig Error, not found parameter : " << path << std::endl;
  }
  return "";
}

const std::string HeexConfig::getConf(const std::string& path, const std::string& defaultValue)
{
  std::string strValue = this->getConf(path);
  if (strValue == "")
  {
    return defaultValue;
  }
  return strValue;
}

int HeexConfig::getConf(const std::string& path, int defaultValue)
{
  const std::string strValue = this->getConf(path);
  if (strValue == "")
  {
    return defaultValue;
  }

  return atoi(strValue.c_str());
}

unsigned short HeexConfig::getConf(const std::string& path, unsigned short defaultValue)
{
  const std::string strValue = this->getConf(path);
  if (strValue == "")
  {
    return defaultValue;
  }

  return static_cast<unsigned short>(atoi(strValue.c_str()));
}
double HeexConfig::getConf(const std::string& path, double defaultValue)
{
  const std::string strValue = this->getConf(path);
  if (strValue == "")
  {
    return defaultValue;
  }

  return atof(strValue.c_str());
}

float HeexConfig::getConf(const std::string& path, float defaultValue)
{
  const std::string strValue = this->getConf(path);
  if (strValue == "")
  {
    return defaultValue;
  }

  return static_cast<float>(atof(strValue.c_str()));
}

std::vector<std::string> HeexConfig::getClasses()
{
  std::vector<std::string> classes;
  std::string prev;

  const std::lock_guard<std::mutex> lock(_mtx);
  for (std::map<std::string, std::string>::iterator it = _HeexConfig.begin(); it != _HeexConfig.end(); ++it)
  {
    if (it->first.substr(0, it->first.find_first_of("/")) != prev)
    {
      prev = it->first.substr(0, it->first.find_first_of("/"));
      classes.push_back(prev);
    }
  }

  return (classes);
}

std::vector<std::string> HeexConfig::getKeys(std::string className)
{
  std::vector<std::string> classes;

  const std::lock_guard<std::mutex> lock(_mtx);
  for (std::map<std::string, std::string>::iterator it = _HeexConfig.begin(); it != _HeexConfig.end(); ++it)
  {
    if (strncmp(it->first.c_str(), className.c_str(), className.length()) == 0)
    {
      classes.push_back(it->first.substr(it->first.find_first_of("/") + 1, it->first.length() - (it->first.find_first_of("/") + 1)));
    }
  }

  return (classes);
}

///
/// Private

void HeexConfig::addToConf(const std::string& key, std::string value)
{
  const std::lock_guard<std::mutex> lock(_mtx);
  bool found = false;
  for (std::map<std::string, std::string>::iterator it = _HeexConfig.begin(); it != _HeexConfig.end(); ++it)
  {
    if (it->first == key)
    {
      it->second = value;
      found      = true;
    }
  }

  if (found == false)
  {
    _HeexConfig.insert(std::pair<std::string, std::string>(key, value));
  }
}
