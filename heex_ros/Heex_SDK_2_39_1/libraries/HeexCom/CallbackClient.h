///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include "HeexUtilsLog.h"
#include "TcpClient.h"

typedef boost::function<void(std::string)> NoComCallback;

class CallbackClient : public TcpClient
{
public:
  CallbackClient(std::string serverIp, unsigned short serverPort, bool waitForServer = false);

  virtual void subscribeToSend(NoComCallback cb);

  virtual void send(const std::string& msg) override;

  virtual void runIos() override {}
  virtual void connect() override {}

protected:
  std::vector<NoComCallback> _vecSendCb;
};
