///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include <boost/asio.hpp>
#include <boost/function.hpp>

#include "TcpHandler.h"

// #define TCP_SERVER_BUFFER_LENGTH 24

class TcpSession : public std::enable_shared_from_this<TcpSession>, public Heex::Com::TcpHandler
{
public:
  TcpSession(boost::asio::io_service& ios, unsigned int id);
  void start();
  const unsigned int& getId() { return _id; }
  boost::asio::ip::tcp::socket& getSocket() { return _socket; }
  void subscribeToClientQuit(ChangeCallback cb);
  // void subscribeToCmd(std::string cmd, CmdCallback cb);
  void talk(const std::string& msg);
  void close();

private:
  void handleRead(std::shared_ptr<TcpSession>& s, const boost::system::error_code& err, size_t bytesTransferred);
  void handleWrite(const boost::system::error_code& err, std::size_t bytesTransfered);
  // void computeReceivedMessage(std::string newBuff);
  void triggerEndClientCbk();

  boost::asio::ip::tcp::socket _socket;
  // unsigned int _id;
  // char _buff[TCP_SERVER_BUFFER_LENGTH];
  // std::string _buffRest;

  // CmdCallbacks _cbs;
  ChangeCallbacks _endClientCbs;
};
