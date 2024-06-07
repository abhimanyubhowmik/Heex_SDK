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

#include "HeexUtilsLog.h"
#include "StringMonitor.h"

// This basic StringMonitor Sample creates an sampleStringMonitor instance. It'll then update some string values sent to the monitor, and if
// the given string is equal to the monitored string set during the creation of Monitor, it activates the trigger
int main(int argc, char** argv)
{
  HEEX_LOG(info) << "Heex SDK version : " << Heex::HEEX_SYSTEM_SDK_VERSION << std::endl;

  // TODO: Replace this left Monitor UUID with the one provided (by the Web Platform) for your Trigger or give a UUID as an input to the generated executable
  std::string monitorUuid = "M-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)";
  if (argc > 1) // Parse the UUID as an optional parameter.
  {
    monitorUuid = argv[1];
  }
  const std::string serverIp    = "127.0.0.1";
  const unsigned int serverPort = 4242;

  HEEX_LOG(info) << argv[0] << " will be exposed as " << monitorUuid << " tries to connect to " << serverIp << ":" << serverPort << "." << std::endl;

  // configure and launch the Monitor
  StringMonitor sampleStringMonitor(monitorUuid, serverIp, serverPort);
  sampleStringMonitor.awaitReady(); // awaits the connection with the kernel.

  std::string input_strings[] = {"Roundabout", "Highway", "Animal Crossing", "City center", "Some other random string"};
  int cntr                    = 0;
  while (true)
  {
    HEEX_LOG(info) << "my string input : " << input_strings[cntr % input_strings->length()] << std::endl;
    sampleStringMonitor.updateValue(input_strings[cntr % input_strings->length()]);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    cntr++;
  }

  return EXIT_SUCCESS;
}
