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
#include "ZoneMonitor.h"

// This basic ZoneMonitor Sample creates an sampleZoneMonitor instance. It'll then update the GPS values of the instance,
// and if this position gets inside the set zone (provided during the creation of the monitor on the platform), it'll activate the monitor
int main(int argc, char** argv)
{
  HEEX_LOG(info) << "Heex SDK version : " << Heex::HEEX_SYSTEM_SDK_VERSION << std::endl;

  // TODO: Replace this left Monitor UUID with the one provided (by the Web Platform) for your Trigger or give a UUID as an input to the generated executable
  std::string monitor_uuid = "M-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)";
  if (argc > 1) // Parse the UUID as an optional parameter.
  {
    monitor_uuid = argv[1];
  }
  const std::string serverIp    = "127.0.0.1";
  const unsigned int serverPort = 4242;

  HEEX_LOG(info) << argv[0] << " will be exposed as " << monitor_uuid << " tries to connect to " << serverIp << ":" << serverPort << "." << std::endl;

  // configure and launch the Monitor
  ZoneMonitor sampleZoneMonitor(monitor_uuid, serverIp, serverPort);
  sampleZoneMonitor.awaitReady(); // awaits the connection with the kernel.

  HEEX_LOG(info) << "Test Paris 11" << std::endl;
  sampleZoneMonitor.updateValue(48.86292854949911, 2.385822266652272); // Paris 11
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  HEEX_LOG(info) << "Test Dijon" << std::endl;
  sampleZoneMonitor.updateValue(47.59571647649772, 5.763069520668387); // Dijon
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  HEEX_LOG(info) << "Test Chartres" << std::endl;
  sampleZoneMonitor.updateValue(48.42727849702349, 1.4825034618277229); // Chartres
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  HEEX_LOG(info) << "Test Beauvais" << std::endl;
  sampleZoneMonitor.updateValue(49.420258402165906, 2.087570410410857); // Beauvais
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  HEEX_LOG(info) << "Test Versailles" << std::endl;
  sampleZoneMonitor.updateValue(48.79758334100409, 2.1318235573795965); // Versailles
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  return EXIT_SUCCESS;
}
