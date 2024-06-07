///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#include "Agent.h"

#include <sstream>
#define BOOST_NO_STD_WSTRING 0
#include <boost/date_time/posix_time/posix_time.hpp>

#include "AgentArgs.h"
#include "HeexConfig.h"
#include "HeexUtils.h"
#include "Tools.h"

Agent::Agent(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort, const std::string& implementationVersion, bool autoStartCom)
    : _agentType(Heex::HEEX_AGENT_TYPE_DEFAULT),
      _uuid(uuid),
      _agentReady(false),
      _tcpC(nullptr),
      _implementationVersion(implementationVersion),
      _versionChecker(Heex::HEEX_SYSTEM_SDK_VERSION)
{
  this->extractAgentInfo(uuid);
  HEEX_LOG(info) << "------------------- Running " << uuid << " (" << implementationVersion << ") using SDK " << Heex::HEEX_SYSTEM_SDK_VERSION << "-------------------"
                 << std::endl;
  HeexUtils::Log::instance().printLoggerHeader();
  if (autoStartCom == true)
  {
    this->initCom(serverIp, serverPort);
  }
}

Agent::Agent(const std::string& uuid, const std::string& configurationFile, const std::string& implementationVersion)
    : _agentType(Heex::HEEX_AGENT_TYPE_DEFAULT),
      _uuid(uuid),
      _agentReady(false),
      _implementationVersion(implementationVersion),
      _versionChecker(Heex::HEEX_SYSTEM_SDK_VERSION)
{
  HeexConfig::instance().readConfFile(configurationFile);
  const std::string serverIp(HeexConfig::instance().getConf("CustomerSide/serverIp", "127.0.0.1"));
  const int serverPort(HeexConfig::instance().getConf("CustomerSide/serverPort", 4242));
  this->extractAgentInfo(uuid);
  HeexUtils::Log::instance().printLoggerHeader();
  this->initCom(serverIp, serverPort);
}

Agent::~Agent()
{
  this->deleteAllValueConfiguration();
}

void Agent::extractAgentInfo(const std::string& agentInfo)
{
  // Try to extract the UUID and implementation version: A-HEEXXXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.2.3)
  const size_t versionSep    = agentInfo.find_first_of("(");
  const size_t versionSepEnd = agentInfo.find_first_of(")");
  if (versionSep != std::string::npos && versionSepEnd != std::string::npos && versionSep < versionSepEnd)
  {
    _uuid                  = agentInfo.substr(0, versionSep);
    // The implementationVersion parsed here prevail on the one set by defaulted or excplicitly in the constructor will get replaced.
    _implementationVersion = agentInfo.substr(versionSep + 1, versionSepEnd - versionSep - 1);
  }
  // If no version separator has matched, the uuid remains set to agentInfo value.
}

void Agent::initCom(const std::string& serverIp, const unsigned int& serverPort)
{
  _serverIp   = serverIp;
  _serverPort = static_cast<unsigned short>(serverPort);
  _tcpC       = std::make_shared<TcpClient>(serverIp, _serverPort, true);
  this->subscribeCom();
}

void Agent::subscribeCom()
{
  if (_tcpC != nullptr)
  {
    _tcpC->subscribeToServerConnection(std::bind(&Agent::onServerConnection, this));
    _tcpC->subscribeToServerDisconnection(std::bind(&Agent::onServerDisconnection, this));
    _tcpC->subscribeToCmd(Heex::HEEX_AGENT_IDENTIFICATION_REQUEST, std::bind(&Agent::onIdentificationRequest, this, std::placeholders::_1));
    _tcpC->subscribeToCmd(Heex::HEEX_AGENT_IDENTIFICATION_REJECTED, std::bind(&Agent::onIdentificationRejected, this, std::placeholders::_1));
    _tcpC->subscribeToCmd(Heex::HEEX_AGENT_IDENTIFICATION_ACCEPTED, std::bind(&Agent::onIdentificationAccepted, this, std::placeholders::_1));
    _tcpC->subscribeToCmd(Heex::HEEX_AGENT_CONFIGURATION, std::bind(&Agent::onConfigurationChanged, this, std::placeholders::_1));
  }
}

void Agent::setComInterface(std::shared_ptr<TcpClient> com)
{
  _tcpC = com;
}

void Agent::onServerConnection()
{
  HEEX_LOG(info) << "Connection to heexCore service " << _serverIp << ":" << _serverPort << " from " << _uuid << std::endl;
}

void Agent::onServerDisconnection()
{
  HEEX_LOG(info) << "Disconnection of heexCore service " << _serverIp << ":" << _serverPort << " from " << _uuid << std::endl;
  this->deleteAllValueConfiguration();
}

void Agent::onIdentificationRequest(const std::string&)
{
  std::string encodedImplementationVersion = _implementationVersion;
  Heex::Tools::encodeEscapeCharactersWithReplacementCode(encodedImplementationVersion);
  std::stringstream cmd;
  cmd << "Identification \"" << _uuid << "\" \"" << this->getTimestampStr() << "\" \"" << encodedImplementationVersion << "\" \"" << this->getSdkVersion() << "\" \"" << _agentType
      << "\"\n";
  _tcpC->send(cmd.str());
  HEEX_LOG(info) << cmd.str() << std::endl;
}

void Agent::onIdentificationRejected(const std::string& msg)
{
  HEEX_LOG(error) << "Identification rejected by SDE." << std::endl;
  HEEX_LOG(trace) << "Identification Rejection msg: \"" << msg << "\"" << std::endl;
  _tcpC->setWaitForServer(false);
  _tcpC->stop();

  // The msg payload can contain some couples and involves some warnings or info to log
  // NOTE: Only provide the part where couples are located. Currently, the whole message does suffice.
  this->getStaticInformationFromMsg(msg);
}

void Agent::onIdentificationAccepted(const std::string& msg)
{
  HEEX_LOG(debug) << "Identification accepted by SDE." << msg << std::endl;
  HEEX_LOG(trace) << "Identification Acception msg: \"" << msg << "\"" << std::endl;

  // The msg payload can contain some couples and involves some warnings or info to log
  // NOTE: Only provide the part where couples are located. Currently, the whole message does suffice.
  this->getStaticInformationFromMsg(msg);
}

void Agent::createValueConfiguration(const std::string& cmd)
{
  // Parse the head of the message.
  ValueConfHeader header = ValueConfiguration::parseStrValue(cmd);

  // Call child implementation of handleValueConfiguration method if available to parse the value.
  HEEX_LOG(debug) << "Agent::createValueConfiguration Call handleValueConfiguration " << cmd << std::endl;
  HEEX_LOG(info) << "Receiving ValueConfiguration " << header.cvUuid << " for " << header.name << "(" << header.uuid << ")"
                 << " with values : ==" << HeexUtils::encodeVectorAsString(header.params, ',') << "==" << std::endl;
  ValueConfiguration* vc = this->handleValueConfiguration(cmd, header);

  std::stringstream reply;
  if (vc != nullptr && vc->isValid() == true)
  {
    _valueConfigurations.push_back(vc);

    reply << "ValueConfigurationAck " << vc->getUuid() << " " << vc->getCvUuid();
  }
  else
  {
    reply << "BadValueConfiguration '" << cmd << "'";
  }
  _tcpC->send(reply.str());
}

void Agent::setConstantValues()
{
  _constantsValues.clear();
  const std::vector<std::string> constantValues = this->retrieveDynamicConfigValues(Heex::HEEX_CONSTANTS_VALUES_KEY);
  for (const std::string& cv : constantValues)
  {
    std::pair<std::string, std::string> constantValue;
    if (Heex::Tools::parseConstantValueMsg(cv, constantValue))
    {
      HEEX_LOG(info) << "Setting constant value " << constantValue.first << " = " << constantValue.second << std::endl;
      _constantsValues[constantValue.first] = Heex::Tools::parseConstantValueValuePart(constantValue.second);
    }
  }
}

void Agent::setConstantValues(const std::unordered_map<std::string, std::string>& constantValues)
{
  _constantsValues.clear();
  for (std::unordered_map<std::string, std::string>::const_iterator it = constantValues.begin(); it != constantValues.end(); ++it)
  {
    _constantsValues[it->first] = {it->second};
  }
}

void Agent::setConstantValues(const std::unordered_map<std::string, std::vector<std::string>>& constantValues)
{
  _constantsValues.clear();
  _constantsValues = constantValues;
}

void Agent::parseAndLoadNewConfiguration(const std::string& msg)
{
  std::vector<std::vector<std::string>> couplesInMsg = HeexUtils::decodeStringAsVector(msg, Heex::HEEX_KEY_VALUE_SEPARATOR, Heex::HEEX_COUPLE_SEPARATOR);
  for (std::vector<std::string>& couple : couplesInMsg)
  {
    // Decode the current couple for both key and value(s)
    std::string key = "";
    for (unsigned int i = 0; i < couple.size(); ++i)
    {
      std::string& item = couple[i];
      Heex::Tools::decodeReplacementCodeWithEscapeCharacters(item);
      if (i == 0)
      {
        key = item;
      }
      else
      {
        _dynamicConfigRegistry[key].push_back(item);
      }
    }
  }
}

void Agent::onConfigurationChanged(const std::string& msg)
{
  _agentReady = false;
  _dynamicConfigRegistry.clear();

  this->parseAndLoadNewConfiguration(msg);

  // set constant values
  this->setConstantValues();

  // reset and create new values configuration
  this->deleteAllValueConfiguration();

  const std::vector<std::string> valuesConfiguration = this->retrieveDynamicConfigValues(Heex::HEEX_VALUE_CONFIGURATION_KEY);
  for (const std::string& vc : valuesConfiguration)
  {
    this->createValueConfiguration(vc);
  }

  // run potential additional commands before finishing configuration change
  this->onConfigurationChangedCallback();

  _agentReady = true;
}

std::string Agent::getTimestampStr()
{
  return HeexUtils::getTimestampStr();
}

bool Agent::isReady()
{
  return _agentReady;
}

void Agent::awaitReady()
{
  while (this->isReady() == false)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void Agent::reportIncident(const std::string msg)
{
  Heex::Incident inc;
  inc.uuid             = _uuid;
  inc.timestamp        = HeexUtils::getTimestampStr();
  inc.component        = HeexUtils::getExecutableName();
  inc.componentVersion = _implementationVersion;
  inc.message          = msg;
  _incidents.push_back(inc);
}

boost::posix_time::ptime Agent::getTimestamp()
{
  return boost::posix_time::microsec_clock::universal_time();
}

std::string Agent::convertTimestampToStr(boost::posix_time::ptime time)
{
  std::stringstream t;
  t << to_iso_extended_string(time);

  return t.str();
}

/// This is to be implemented in Agent implementation !
ValueConfiguration* Agent::handleValueConfiguration(const std::string&, ValueConfHeader&)
{
  return nullptr;
}

void Agent::deleteAllValueConfiguration()
{
  for (std::vector<ValueConfiguration*>::iterator it = _valueConfigurations.begin(); it != _valueConfigurations.end(); ++it)
  {
    delete (*it);
    (*it) = nullptr;
  }
  _valueConfigurations.clear();
}

std::string Agent::encodeIncidents()
{
  std::string encodedIncidents(Heex::Incident::serializeIncidents(_incidents));
  _incidents.clear();
  Heex::Tools::encodeEscapeCharactersWithReplacementCode(encodedIncidents);
  return encodedIncidents;
}

void Agent::getStaticInformationFromMsg(const std::string& msg)
{
  _staticConfigRegistry.clear();

  std::vector<std::vector<std::string>> couplesInMsg = HeexUtils::decodeStringAsVector(msg, Heex::HEEX_KEY_VALUE_SEPARATOR, Heex::HEEX_COUPLE_SEPARATOR);
  for (std::vector<std::string>& couple : couplesInMsg)
  {
    // Extract Agent Configuration value. i.e "HEEX_XXX_KEY:VALUE1:VALUE2" key-values
    std::string key = "";
    // Decode the current couple for both key and value(s)
    for (std::string& item : couple)
    {
      Heex::Tools::decodeReplacementCodeWithEscapeCharacters(item);
      if (item == couple[0])
      {
        key = item;
      }
      else
      {
        _staticConfigRegistry[key].push_back(item);
      }
    }
  }

  // Check for rejection cause. Extract the "HEEX_AGENT_IDENTIFICATION_REJECTION_REASON:<REASON>" key-value
  std::vector<std::string> res = this->retrieveStaticConfigValues(Heex::HEEX_AGENT_IDENTIFICATION_REJECTION_REASON);
  if (res.size() >= 1)
  {
    HEEX_LOG(error) << "This Agent " << this->getUuid() << " has its identification rejected by the SDE due to reason: " << res[0] << std::endl;
    // We don't report incident on identification rejection
  }
  res.clear();
  // Check for SDK version warning. Extract the "HEEX_AGENT_SDK_VERSION_DIFFER:<VERSION>" key-value
  res = this->retrieveStaticConfigValues(Heex::HEEX_AGENT_SDK_VERSION_DIFFER);
  if (res.size() >= 1)
  {
    HEEX_LOG(warning) << "This Agent " << this->getUuid() << " has been built with SDK version " << this->getSdkVersion() << ". Running it with a different HeexCore version (here "
                      << res[0] << ") is at your own risk." << std::endl;
    this->reportIncident(Heex::HEEX_AGENT_SDK_VERSION_DIFFER + " Agent uses " + this->getSdkVersion() + " ,SDE uses " + res[0]);
  }
}
std::vector<std::string> Agent::retrieveStaticConfigValues(const std::string& key)
{
  std::vector<std::string> res;
  if (_staticConfigRegistry.find(key) != std::end(_staticConfigRegistry))
  {
    res = _staticConfigRegistry[key];
  }
  return res;
}
std::vector<std::string> Agent::retrieveDynamicConfigValues(const std::string& key)
{
  std::vector<std::string> res;
  if (_dynamicConfigRegistry.find(key) != std::end(_dynamicConfigRegistry))
  {
    res = _dynamicConfigRegistry[key];
  }
  return res;
}

void Agent::disableLogToFile()
{
  HeexUtils::Log::instance().disableLogToFile();
}
