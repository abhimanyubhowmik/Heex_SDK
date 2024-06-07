///
/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "featureSampleMonitor_ConfChangedCallback.h"

// Sample Monitor's constructor
Heex::Sample::sampleMonitorConfChangeCallback::sampleMonitorConfChangeCallback(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort)
    : BooleanMonitor(uuid, serverIp, serverPort)
{
  HEEX_LOG(info) << "sampleMonitorConfChangeCallback | " << uuid << " " << serverIp << " " << serverPort << " " << std::endl;
}

//! [Tested Feature] Callback function that shall be called during each configuration change
void Heex::Sample::sampleMonitorConfChangeCallback::onConfigurationChangedCallback()
{
  // NOTE: This function allows user to run commands every time the monitor has been reconfigured
  // NOTE: In this sample, we shall get the Monitor's new constant values and valueConfigurations, and print them out, but many other use cases are possible.
  // Let's get the new ValueCOnfigurations and Constant Values
  std::vector<ValueConfiguration*> valueConfs                           = this->getValueConfigurations();
  std::unordered_map<std::string, std::vector<std::string>> constValues = this->getConstantValues();

  // Now we print out all the ValueCOnfigurations if there are any
  if (valueConfs.size() > 0)
  {
    HEEX_LOG(info) << "Monitor's value confs are: " << std::endl;
    for (int i = 0; i < valueConfs.size(); i++)
    {
      HEEX_LOG(info) << "Value Configuration " << i << " :" << std::endl;
      HEEX_LOG(info) << " - Name     : " << valueConfs[i]->getName() << std::endl;
      HEEX_LOG(info) << " - Type     : " << valueConfs[i]->getType() << std::endl;
      HEEX_LOG(info) << " - UUID     : " << valueConfs[i]->getUuid() << std::endl;
      HEEX_LOG(info) << " - IsValid? : " << valueConfs[i]->isValid() << std::endl;
    }
  }

  // And finally we print out the Constant Values if there are any
  if (constValues.size() > 0)
  {
    HEEX_LOG(info) << "Monitor's constant values are: " << std::endl;
    for (const auto& it : constValues)
    {
      std::stringstream keys;
      if (!it.second.empty())
      {
        for (const auto& val : it.second)
        {
          keys << val << "  ,  ";
        }
        // Get the last value without the trailing comma
        keys << it.second.back();
      }
      HEEX_LOG(info) << " - " << it.first << " : " << keys.str() << std::endl;
    }
  }
}

// Main behavior is identical to the get-started example, remember to change the monitor UUID
int main(int argc, char* argv[])
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

  HEEX_LOG(info) << argv[0] << " will be exposed as " << monitorUuid << " try to connect to " << serverIp << ":" << serverPort << "." << std::endl;
  // configure and launch the Monitor
  Heex::Sample::sampleMonitorConfChangeCallback sampleBooleanMonitor(monitorUuid, serverIp, serverPort);
  // The monitor has been created, so the initial configuration will trigger the onConfigurationChangedCallback to be called.
  // Any future re-configuration shall call it as well (ie: OTA)
  sampleBooleanMonitor.awaitReady(); // awaits the connection with the kernel.
  while (true)
  {
    const bool state = std::rand() / (double)RAND_MAX > 0.5 ? false : true; // random state
    HEEX_LOG(info) << "state is " << state << std::endl;
    sampleBooleanMonitor.updateValue(state);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}
