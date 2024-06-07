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
#include "IntervalMonitor.h"

// This basic IntervalMonitor Sample creates a sampleIntervalMonitor instance. It'll then update the instance's humidity percentage it
// receives with random values, and if the percentage is within one of the provided intervals (provided during the creation
// of the monitor on the platform), the monitor activates.
int main(int argc, char** argv)
{
  HEEX_LOG(info) << "Heex SDK version : " << Heex::HEEX_SYSTEM_SDK_VERSION << std::endl;

  std::srand(static_cast<unsigned int>(std::time(nullptr))); // Initialize random seed

  // TODO: Replace the  Monitor UUID with the one provided (by the Web Platform) for your Trigger or give a UUID as an input to the generated executable
  std::string monitorUuid = "M-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)";
  if (argc > 1) // Parse the UUID as an optional parameter.
  {
    monitorUuid = argv[1];
  }
  const std::string serverIp    = "127.0.0.1";
  const unsigned int serverPort = 4242;

  HEEX_LOG(info) << argv[0] << " will be exposed as " << monitorUuid << " tries to connect to " << serverIp << ":" << serverPort << "." << std::endl;

  // configure and launch the Monitor
  IntervalMonitor sampleIntervalMonitor(monitorUuid, serverIp, serverPort);
  sampleIntervalMonitor.setSignalUnit("%"); // Provide the unit of input data
  sampleIntervalMonitor.awaitReady();       // awaits the connection with the kernel.

  while (true)
  {
    const double humidity = (std::rand() % 100000) / 1000.0; // random humidity between 0 and 100 %
    HEEX_LOG(info) << "Humidity is at " << humidity << " " << sampleIntervalMonitor.getSignalUnit() << std::endl;
    sampleIntervalMonitor.updateValue(humidity);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return EXIT_SUCCESS;
}
