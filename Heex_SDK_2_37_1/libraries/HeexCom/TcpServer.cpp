///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "TcpServer.h"

#include <boost/bind/bind.hpp>
#include <iostream>

#include "HeexUtilsLog.h"

TcpServer::TcpServer(boost::asio::io_service& ios, unsigned short port) : _ios(ios), _acceptor(ios, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)), _port(port)
{
  _sessionId                                = 0;
  const std::shared_ptr<TcpSession> session = std::make_shared<TcpSession>(_ios, _sessionId);
  _acceptor.async_accept(session->getSocket(), boost::bind(&TcpServer::handleAccept, this, session, boost::asio::placeholders::error));
}

void TcpServer::handleAccept(std::shared_ptr<TcpSession> session, const boost::system::error_code& err)
{
  if (!err)
  {
    session->start();
    _sessions.push_back(session);
    this->propagateSubscriptions(session);
    this->triggerNewClientCbk(session);
    _sessionId++;
    session = std::make_shared<TcpSession>(_ios, _sessionId);
    _acceptor.async_accept(session->getSocket(), boost::bind(&TcpServer::handleAccept, this, session, boost::asio::placeholders::error));
  }
  else
  {
    HEEX_LOG(error) << "TcpServer::handleAccept err: " + err.message() << std::endl;
    session.reset();
  }
}

void TcpServer::subscribeToNewClient(ChangeCallback cb)
{
  _newClientCbs.push_back(cb);
  HEEX_LOG(trace) << "TcpServer::subscribeToNewClient" << std::endl;
}

void TcpServer::subscribeToClientQuit(ChangeCallback cb)
{
  _endClientCbs.push_back(cb);
  HEEX_LOG(trace) << "TcpServer::subscribeToClientQuit" << std::endl;
}

void TcpServer::subscribeToAllCmd(CmdCallback cb)
{
  _cbs.push_back(std::pair<std::string, CmdCallback>("*", cb));
  HEEX_LOG(trace) << "TcpServer::subscribeToCmd *" << std::endl;
}

void TcpServer::subscribeToCmd(std::string cmd, CmdCallback cb)
{
  _cbs.push_back(std::pair<std::string, CmdCallback>(cmd, cb));
  HEEX_LOG(trace) << "TcpServer::subscribeToCmd " << cmd << std::endl;
}

std::string TcpServer::getClientIp(unsigned int id)
{
  for (std::vector<std::shared_ptr<TcpSession>>::iterator it = _sessions.begin(); it != _sessions.end(); ++it)
  {
    if (id == (*it)->getId())
    {
      return (*it)->getSocket().remote_endpoint().address().to_string();
    }
  }
  return "ClientNotFound";
}

void TcpServer::talkTo(unsigned int clientId, const std::string& msg)
{
  for (std::vector<std::shared_ptr<TcpSession>>::iterator it = _sessions.begin(); it != _sessions.end(); ++it)
  {
    if (clientId == (*it)->getId())
    {
      (*it)->talk(msg + '\n');
      break;
    }
  }
}

void TcpServer::close(unsigned int id)
{
  for (std::vector<std::shared_ptr<TcpSession>>::iterator it = _sessions.begin(); it != _sessions.end(); ++it)
  {
    if (id == (*it)->getId())
    {
      (*it)->close();
      _sessions.erase(it);
      break;
    }
  }
}

///
/// private

void TcpServer::propagateSubscriptions(std::shared_ptr<TcpSession> session)
{
  for (CmdCallbacks::iterator it = _cbs.begin(); it != _cbs.end(); ++it)
  {
    session->subscribeToCmd((*it).first, (*it).second);
  }

  for (ChangeCallbacks::iterator it = _endClientCbs.begin(); it != _endClientCbs.end(); ++it)
  {
    session->subscribeToClientQuit((*it));
  }
}

void TcpServer::triggerNewClientCbk(std::shared_ptr<TcpSession> session)
{
  for (ChangeCallbacks::iterator it = _newClientCbs.begin(); it != _newClientCbs.end(); ++it)
  {
    (*it)(session->getId());
  }
}
