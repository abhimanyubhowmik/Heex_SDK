#include "getStartedMonitor.h"

#include <iostream>
#include <thread>

Heex::GetStarted::GetStartedMonitor::GetStartedMonitor(const std::string& uuid, const std::string& serverIp, unsigned int& serverPort) : BooleanMonitor(uuid, serverIp, serverPort)
{
  HEEX_LOG(info) << "GetStartedMonitor | " << uuid << " " << serverIp << " " << serverPort << " " << std::endl;
}

int main(int argc, char* argv[])
{
  // TODO: Replace this left Monitor UUID with the one provided (by the Web Platform) for your Trigger
  std::string monitorUuid = "D-d6fa84b8-26cb-44b6-9bde-89ec3ef5b4d9(1.0.0)";
  if (argc > 1)
  {
    monitorUuid = argv[1]; // Parse the UUID as an optional parameter.
  }

  //
  // Configure the Monitor
  //
  const std::string serverIp = "127.0.0.1";
  unsigned int serverPort    = 4242;
  Heex::GetStarted::GetStartedMonitor myMonitor(monitorUuid, serverIp, serverPort);

  //
  // Launch Monitor
  //
  myMonitor.awaitReady();
  myMonitor.updateValue(true);

  return 0;
}
