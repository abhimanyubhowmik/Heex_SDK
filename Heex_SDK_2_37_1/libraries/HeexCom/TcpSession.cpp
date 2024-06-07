///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "TcpSession.h"

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>
#include <memory>

#include "HeexUtilsLog.h"

TcpSession::TcpSession(boost::asio::io_service& ios, unsigned int id) : Heex::Com::TcpHandler(id), _socket(ios) {}

void TcpSession::start()
{
  memset(&_buff, 0, sizeof(_buff));
  _socket.async_read_some(
      boost::asio::buffer(_buff, TCP_SERVER_BUFFER_LENGTH),
      boost::bind(&TcpSession::handleRead, this, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void TcpSession::subscribeToClientQuit(ChangeCallback cb)
{
  _endClientCbs.push_back(cb);
}

// void TcpSession::subscribeToCmd(std::string cmd, CmdCallback cb)
// {
//   _cbs.push_back(std::pair<std::string, CmdCallback >(cmd, cb));
// }

void TcpSession::talk(const std::string& msg)
{
  _socket.async_send(boost::asio::buffer(msg), boost::bind(&TcpSession::handleWrite, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void TcpSession::close()
{
  _socket.close();
}

void TcpSession::handleRead(std::shared_ptr<TcpSession>&, const boost::system::error_code& err, size_t size)
{
  if (!err)
  {
    std::string newBuff(_buff);
    memset(&_buff, 0, sizeof(_buff));
    newBuff = newBuff.substr(0, size);
    this->computeReceivedMessage(newBuff);
    _socket.async_read_some(
        boost::asio::buffer(_buff, TCP_SERVER_BUFFER_LENGTH),
        boost::bind(&TcpSession::handleRead, this, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
  }
  else if ((err == boost::asio::error::eof) || (err == boost::asio::error::connection_reset))
  {
    this->triggerEndClientCbk();
  }
  else
  {
    HEEX_LOG(error) << "TcpSession::handleRead Error (recv): " << err.message() << std::endl;
  }
}

void TcpSession::handleWrite(const boost::system::error_code& err, std::size_t size)
{
  if (err)
  {
    HEEX_LOG(error) << "TcpSession::handleWrite Err : " << err << " transfered : " << size << std::endl;
  }
}

// void TcpSession::computeReceivedMessage(std::string newBuff)
// {
//   std::string allBuffer(_buffRest + newBuff);
//   size_t sep = allBuffer.find_first_of("\n\r");
//   while (sep != std::string::npos)
//   {
//     std::string cmd(allBuffer.substr(0, sep));

//     if (cmd.size() != 0)
//     {
//       size_t spacer = cmd.find_first_of(" \t\n\r");
//       std::string cmdName(cmd.substr(0, spacer));
//       std::string args = cmd.substr(spacer + 1, std::string::npos);

//       for (CmdCallbacks::iterator it = _cbs.begin(); it != _cbs.end(); ++it)
//       {
//         if (boost::iequals((*it).first, cmdName) == true)
//         {
//           (*it).second(args, _id);
//         }
//         if ((*it).first == "*")
//         {
//           (*it).second(cmd, _id);
//         }
//       }
//     }

//     allBuffer = allBuffer.substr(sep + 1, std::string::npos);
//     sep = allBuffer.find_first_of("\n\r");
//   }
//   _buffRest = allBuffer;
// }

void TcpSession::triggerEndClientCbk()
{
  for (ChangeCallbacks::iterator it = _endClientCbs.begin(); it != _endClientCbs.end(); ++it)
  {
    (*it)(_id);
  }
}
