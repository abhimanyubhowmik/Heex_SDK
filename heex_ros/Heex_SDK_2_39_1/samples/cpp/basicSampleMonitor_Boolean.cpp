///
/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include <fstream>
#include <iostream>
#include <string>
#include <thread> // std::this_thread::sleep_for

#include "BooleanMonitor.h"
#include "HeexUtilsLog.h"

// This basic BooleanMonitor Sample creates a sampleBooleanMonitor instance, and updates it's state to a random boolean state, and if
// the given bool is equal to the monitored state set during the creation of Monitor, it activates the trigger
int main(int argc, char** argv)
{
  HEEX_LOG(info) << "Heex SDK version : " << Heex::HEEX_SYSTEM_SDK_VERSION << std::endl;
  std::srand(static_cast<unsigned int>(std::time(nullptr))); // Initialize random seed

  // TODO: Replace this Monitor UUID with the one provided (by the Web Platform) for your Trigger or give a UUID as an input to the generated executable
  std::string monitorUuid = "M-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)";
  if (argc > 1) // Parse the UUID as an optional parameter.
  {
    monitorUuid = argv[1];
  }
  const std::string serverIp    = "127.0.0.1";
  const unsigned int serverPort = 4242;

  HEEX_LOG(info) << argv[0] << " will be exposed as " << monitorUuid << " tries to connect to " << serverIp << ":" << serverPort << "." << std::endl;

  // configure and launch the Monitor
  BooleanMonitor sampleBooleanMonitor(monitorUuid, serverIp, serverPort);
  sampleBooleanMonitor.awaitReady(); // awaits the connection with the kernel.

  while (true)
  {
    const bool state = std::rand() / (double)RAND_MAX > 0.5 ? false : true; // random state
    HEEX_LOG(info) << "state is " << state << std::endl;
    sampleBooleanMonitor.updateValue(state);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return EXIT_SUCCESS;
}
