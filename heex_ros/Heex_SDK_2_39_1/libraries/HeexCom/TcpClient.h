///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#pragma once

#include <atomic>
#include <boost/asio.hpp>
#include <thread>

#include "TcpHandler.h"

class TcpClient : public Heex::Com::TcpHandler
{
public:
  typedef std::function<void(void)> ClientChangeCallback;
  typedef std::vector<ClientChangeCallback> ClientChangeCallbacks;

  TcpClient(std::string serverIp, unsigned short serverPort, bool retryOnFailure = false);
  virtual ~TcpClient();

  virtual void runIos();
  virtual void connect();
  virtual void stop();
  virtual void subscribeToServerConnection(TcpClient::ClientChangeCallback cb);
  virtual void subscribeToServerDisconnection(TcpClient::ClientChangeCallback cb);
  virtual void send(const std::string& event);
  virtual void sendSync(const std::string& event);

  inline bool isConnected() { return _connectionStatus; }
  inline void setWaitForServer(bool wait) { _retryOnFailure = wait; }

protected:
  /// Initialize read buffer and perform binding on message. Same than TcpSession start()
  void initiateReceptionFromServer();
  void handlerSend(const boost::system::error_code&, std::size_t);
  /// handleRead. Same than TcpSession start()
  void handleRead(const boost::system::error_code& err, size_t bytesTransferred);
  void handleConnect(const boost::system::error_code& err, const boost::asio::ip::basic_endpoint<boost::asio::ip::tcp>& endpoints);
  void triggerServerConnectionCbs();
  void triggerServerDisconnectionCbs();

  boost::asio::io_service _ios;
  std::string _serverIp;
  unsigned short _serverPort;
  boost::asio::ip::tcp::socket _socket;
  std::atomic<bool> _retryOnFailure;
  std::atomic<bool> _connectionStatus{false};
  std::atomic<bool> _exitTcpClient{false};
  std::atomic<unsigned int> _messagesToBeSent{0};
  std::thread _ioThread;
  std::thread _connectThread;
  ClientChangeCallbacks _onServerConnectionCbs;
  ClientChangeCallbacks _onServerDisconnectionCbs;
};
