///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file TcpServer.h
/// @author Momar TOURÉ (momar.faly@heex.io)
/// @brief Header file that defines the TcpServer class. Used to initiate the listening of the Heex interprocess communication on given port.
/// @date 2022-05-02
///
///
#pragma once

#include <boost/asio.hpp>
#include <memory>

#include "TcpSession.h"

/// @brief Set up a TcpServer to enable interprocess communications with some clients on the provided port. Commands and their respective callbacks allow to offer different strategies from the received messages.
/// Reading and writing strategies shall be customized using the command assignations through callback functions (subscribeToCmd, subscribeToAllCmd, subscribeToNewClient, subscribeToClientQuit)
class TcpServer
{
public:
  ///
  /// @brief Deploy a TcpServer on the provided service and socket.
  ///
  /// @param ios The reference to a service from boost::asio (C++ library for networking)
  /// @param port The port number
  TcpServer(boost::asio::io_service& ios, unsigned short port);

  /// Virtual destructor to enable destruction of derivated class without warnings
  virtual ~TcpServer() = default;

  ///
  /// @brief ubscribes to any new client connection with the execution of the provided callback function.
  ///
  /// @param cb Callback function to execute on NewClient
  void subscribeToNewClient(ChangeCallback cb);

  ///
  /// @brief Subscribes to any client disconnection with the execution of the provided callback function.
  ///
  /// @param cb Callback function to execute on ClientQuit
  void subscribeToClientQuit(ChangeCallback cb);

  ///
  /// @brief Subscribes to the command with the execution of the provided callback function.
  ///
  /// @param cmd The command formulated in the client message with potential additional arguments. Msg served to the callback function will be truncated from the command.
  /// @param cb Callback function to execute on Cmd
  void subscribeToCmd(std::string cmd, CmdCallback cb);

  ///
  /// @brief Subscribes to all commands with the execution of the provided callback function. This callback function will be called on all message as any potential cmds will be considered.
  ///
  /// @param cb Callback function to execute on Cmd
  void subscribeToAllCmd(CmdCallback cb);

  ///
  /// @brief Send the provided message to the client with the matching id.
  ///
  /// @param clientId the id of the client.
  /// @param msg the server message to send to the client.
  virtual void talkTo(unsigned int clientId, const std::string& msg);

  ///
  /// @brief Get the Client Ip object
  ///
  /// @param id the id of the client of which we want the Ip address
  /// @return Returns the ip address of the client. If the client is not found, returns an error return value of "ClientNotFound".
  std::string getClientIp(unsigned int id);
  void close(unsigned int id);

protected:
  ///
  /// @brief In case of a recognized command, updates the session. If there is an error, returns an error message and resets the session.
  ///
  /// @param session client session pointer.
  /// @param err error code.
  void handleAccept(std::shared_ptr<TcpSession> session, const boost::system::error_code& err);

  ///
  /// @brief Iterates through the (command, callback function) pairs and updates them. Used when a session update is needed (e.g new command).
  ///
  /// @param session client session pointer.
  void propagateSubscriptions(std::shared_ptr<TcpSession> session);

  ///
  /// @brief Triggers the relevant callback functions on new client.
  ///
  /// @param session client session pointer.
  void triggerNewClientCbk(std::shared_ptr<TcpSession> session);

  ///
  /// @brief Iterates through every client to terminate connection with in order to perform their respective ChangeCallbacks callback (function on connection termination).
  ///
  /// @param session client session pointer.
  void triggerEndClientCbk(std::shared_ptr<TcpSession> session);

  boost::asio::io_service& _ios;
  boost::asio::ip::tcp::acceptor _acceptor;
  std::vector<std::shared_ptr<TcpSession>> _sessions;
  unsigned short _port;
  unsigned int _sessionId;
  CmdCallbacks _cbs;
  ChangeCallbacks _newClientCbs;
  ChangeCallbacks _endClientCbs;
};
