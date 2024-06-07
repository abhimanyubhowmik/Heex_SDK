///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <string>
#include <vector>

#define TCP_SERVER_BUFFER_LENGTH 1024

typedef boost::function<void(std::string, int)> CmdCallback;
typedef std::pair<std::string, CmdCallback> StrToCmdCb;
typedef std::vector<StrToCmdCb> CmdCallbacks;
typedef std::function<void(int)> ChangeCallback;
typedef std::vector<ChangeCallback> ChangeCallbacks;

namespace Heex
{
namespace Com
{
// Characterizes every Tcp elements that share a common implementation on a list of features.
// They are callbacks, commands, buffer management, communication (send, ), subscribe and )
// This class aims to be inherited by any Tcp element. It is mainly targetted for Tcp client or Tcp server session.
class TcpHandler
{
public:
  TcpHandler(unsigned int id);
  virtual ~TcpHandler();

  void subscribeToCmd(std::string cmd, CmdCallback cb);
  void subscribeToAllCmd(CmdCallback cb);

protected:
  void computeReceivedMessage(std::string newBuff);

  //Client id
  unsigned int _id;
  //Buffer that contains the received characters from ASIO
  char _buff[TCP_SERVER_BUFFER_LENGTH]{};
  std::string _buffRest;

  CmdCallbacks _cbs;
};
} // namespace Com
} // namespace Heex
