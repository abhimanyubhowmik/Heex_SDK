///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "TcpClient.h"

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>
#include <thread>

#include "HeexUtilsLog.h"

TcpClient::TcpClient(std::string serverIp, unsigned short serverPort, bool retryOnFailure)
    : Heex::Com::TcpHandler(serverPort),
      _serverIp(serverIp),
      _serverPort(serverPort),
      _socket(_ios),
      _retryOnFailure(retryOnFailure)
{
  _ioThread         = std::thread(boost::bind(&TcpClient::runIos, this));
  _connectThread    = std::thread(boost::bind(&TcpClient::connect, this));
  _connectionStatus = false;
}

TcpClient::~TcpClient()
{
  this->stop();
}

void TcpClient::runIos()
{
  while (_exitTcpClient == false)
  {
    _ios.reset();
#ifdef HEEXSDK_LEGACY_1804
    _ios.run_one();
#else
    _ios.run_for(std::chrono::milliseconds(10));
#endif
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  _ios.stop();
}

void TcpClient::connect()
{
  if (_connectionStatus == true)
  {
    return;
  }

  const boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(_serverIp), _serverPort);
  boost::system::error_code err;

  _socket.connect(endpoint, err);

  while (_retryOnFailure && err && _exitTcpClient == false)
  {
    _socket.connect(endpoint, err);

    if (err) // We want to return instantly if we are connected
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  if (!err)
  {
    HEEX_LOG(debug) << "Connected to " << _serverIp << ":" << _serverPort << std::endl;
    _connectionStatus = true;
    this->triggerServerConnectionCbs();
    this->initiateReceptionFromServer();
  }
  else if (_exitTcpClient == false)
  {
    // if we are here, it means that we ask for exitTcpClient before connection or that we failed to connect
    HEEX_LOG(warning) << "TcpClient::connect not connected: " << err.message() << std::endl;
    return;
  }
  else
  {
    HEEX_LOG(debug) << "TcpClient::connect asked to stop" << std::endl;
    return;
  }
}

void TcpClient::stop()
{
  while (_messagesToBeSent > 0 && _connectionStatus)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    HEEX_LOG(debug) << "TcpClient::stop waiting for send to come back : " << _messagesToBeSent << std::endl;
  }

  _exitTcpClient = true;
  // We indicate that we will no longer write data on this socket
  if (_socket.is_open())
  {
    boost::system::error_code ec;
    _socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    if (ec)
    {
      HEEX_LOG(debug) << "TcpClient::stop (shutdown): " << ec.message() << std::endl;
    }
  }
  _socket.close();
  _connectionStatus = false;

  if (_connectThread.joinable())
  {
    _connectThread.join();
  }

  if (_ioThread.joinable())
  {
    _ioThread.join();
  }
}

void TcpClient::subscribeToServerConnection(TcpClient::ClientChangeCallback cb)
{
  _onServerConnectionCbs.push_back(cb);
  HEEX_LOG(trace) << "TcpClient::subscribeToServerConnection" << std::endl;
}

void TcpClient::subscribeToServerDisconnection(TcpClient::ClientChangeCallback cb)
{
  _onServerDisconnectionCbs.push_back(cb);
  HEEX_LOG(trace) << "TcpClient::subscribeToServerDisconnection" << std::endl;
}

void TcpClient::send(const std::string& event)
{
  if (_connectionStatus == false)
  {
    return;
  }

  ++_messagesToBeSent;
  _socket.async_send(boost::asio::buffer(event + '\n'), boost::bind(&TcpClient::handlerSend, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void TcpClient::sendSync(const std::string& event)
{
  if (_connectionStatus == false)
  {
    return;
  }

  const std::size_t ret = _socket.send(boost::asio::buffer(event + '\n'));
  if (ret == 0)
  {
    HEEX_LOG(error) << _socket.remote_endpoint().address().to_string() << " err (sendSync)" << std::endl;
  }
}

void TcpClient::initiateReceptionFromServer()
{
  memset(&_buff, 0, sizeof(_buff));
  _socket.async_read_some(
      boost::asio::buffer(_buff, TCP_SERVER_BUFFER_LENGTH),
      boost::bind(&TcpClient::handleRead, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void TcpClient::handleRead(const boost::system::error_code& err, size_t bytesTransferred)
{
  if (!err)
  {
    std::string newBuff(_buff);
    memset(&_buff, 0, sizeof(_buff));
    newBuff = newBuff.substr(0, bytesTransferred);
    this->computeReceivedMessage(newBuff);
    _socket.async_read_some(
        boost::asio::buffer(_buff, TCP_SERVER_BUFFER_LENGTH),
        boost::bind(&TcpClient::handleRead, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
  }
  else if ((err == boost::asio::error::eof) || (err == boost::asio::error::connection_reset))
  {
    _connectionStatus = false;
    HEEX_LOG(debug) << "Server disconnected." << std::endl;
    this->triggerServerDisconnectionCbs();

    if (_retryOnFailure && _exitTcpClient == false)
    { // if we should try reconnecting after error
      _socket.close();
      _socket = boost::asio::ip::tcp::socket(_ios);
      this->connect();
    }
  }
  else if (err == boost::asio::error::operation_aborted)
  {
    HEEX_LOG(debug) << "TcpClient::handleRead socket closed." << std::endl;
  }
  else
  {
    HEEX_LOG(error) << "TcpClient::handleRead err (recv): " << err.message() << std::endl;
  }
}

void TcpClient::handlerSend(const boost::system::error_code& err, std::size_t)
{
  if (err)
  {
    HEEX_LOG(error) << _socket.remote_endpoint().address().to_string() << " err (send): " << err.message() << std::endl;
  }

  if (_messagesToBeSent > 0)
  {
    --_messagesToBeSent;
  }
  else
  {
    HEEX_LOG(warning) << "TcpClient::handlerSend unexpected send handle." << std::endl;
  }
}

void TcpClient::triggerServerConnectionCbs()
{
  for (ClientChangeCallbacks::iterator it = _onServerConnectionCbs.begin(); it != _onServerConnectionCbs.end(); ++it)
  {
    (*it)();
  }
}

void TcpClient::triggerServerDisconnectionCbs()
{
  for (ClientChangeCallbacks::iterator it = _onServerDisconnectionCbs.begin(); it != _onServerDisconnectionCbs.end(); ++it)
  {
    (*it)();
  }
}
