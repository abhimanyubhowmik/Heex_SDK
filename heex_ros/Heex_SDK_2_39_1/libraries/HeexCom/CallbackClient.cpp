///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "CallbackClient.h"

CallbackClient::CallbackClient(std::string serverIp, unsigned short serverPort, bool waitForServer) : TcpClient(serverIp, serverPort, waitForServer)
{
  HEEX_LOG(trace) << "CallbackClient::CallbackClient";
}

void CallbackClient::subscribeToSend(NoComCallback cb)
{
  _vecSendCb.push_back(cb);
}

void CallbackClient::send(const std::string& msg)
{
  for (std::vector<NoComCallback>::iterator it = _vecSendCb.begin(); it != _vecSendCb.end(); ++it)
  {
    (*it)(msg);
  }
}
