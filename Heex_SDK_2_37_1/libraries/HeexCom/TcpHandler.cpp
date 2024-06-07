///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "TcpHandler.h"

#include <boost/algorithm/string.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>
#include <memory>

Heex::Com::TcpHandler::TcpHandler(unsigned int id) : _id(id) {}

Heex::Com::TcpHandler::~TcpHandler() = default;

void Heex::Com::TcpHandler::computeReceivedMessage(std::string newBuff)
{
  std::string allBuffer(_buffRest + newBuff);
  size_t sep = allBuffer.find_first_of("\n\r");
  while (sep != std::string::npos)
  {
    const std::string cmd(allBuffer.substr(0, sep));

    if (cmd.size() != 0)
    {
      const size_t spacer = cmd.find_first_of(" \t\n\r");
      const std::string cmdName(cmd.substr(0, spacer));
      const std::string args = cmd.substr(spacer + 1, std::string::npos);

      for (CmdCallbacks::iterator it = _cbs.begin(); it != _cbs.end(); ++it)
      {
        if (boost::iequals((*it).first, cmdName) == true)
        {
          (*it).second(args, _id);
        }
        if ((*it).first == "*")
        {
          (*it).second(cmd, _id);
        }
      }
    }

    allBuffer = allBuffer.substr(sep + 1, std::string::npos);
    sep       = allBuffer.find_first_of("\n\r");
  }
  _buffRest = allBuffer;
}

void Heex::Com::TcpHandler::subscribeToCmd(std::string cmd, CmdCallback cb)
{
  _cbs.push_back(std::pair<std::string, CmdCallback>(cmd, cb));
}

void Heex::Com::TcpHandler::subscribeToAllCmd(CmdCallback cb)
{
  _cbs.push_back(std::pair<std::string, CmdCallback>("*", cb));
}
