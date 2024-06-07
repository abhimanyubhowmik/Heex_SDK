#!/usr/bin/env python3
#
# Copyright (c) 2024 Heex Technologies
# All Rights Reserved.
#
# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
#
"""
In this sample, every time there is a new reconfiguration (and at initial configuration), we shall print out all the monitor's valueConfigurations and Constant values.
"""
import sys
import time
import random

# Adding the PyHeexCustomerSide Lib to PYTHONPATH. Depending from where you run this sample, you might need to update it to point to the lib
sys.path.insert(1, "../sdk/lib")
sys.path.insert(1, "../../sdk/lib")

from PyHeexCustomerSide import BooleanMonitor  # In this sample we use BooleanMonitor, but any other can also be used
from PyHeexCustomerSide import __version__ as HEEX_SYSTEM_SDK_VERSION


# This sampleMonitorConfChangeCallback class, which inherits from Heex's agent BooleanMonitor class, is created
# to demonstrate how the feature onConfigurationChangedCallback() can be used. The recorder behavior's part is the same
# as the one in the basicSampleMonitor_Boolean
class SampleMonitorConfChangeCallback(BooleanMonitor):
    def __init__(self, uuid, server_ip, server_port, implementation_version="1.0.0"):
        BooleanMonitor.__init__(self, uuid, server_ip, server_port, implementation_version)
        print(f"SampleMonitorConfChangeCallback | instance created: {uuid}, {server_ip}:{server_port}")

    #! [Tested Feature] Callback function that shall be called during each configuration change
    def onConfigurationChangedCallback(self):
        """
        [Sample Key Feature] Callback function that shall be called during each configuration change (called in onConfigurationChanged).
        In this function, we simply fetch all the new Monitor's valueCOnfigurations and Constant Values, and print them out
        """
        const_values = self.getConstantValues()
        value_confs = self.getValueConfigurations()
        if const_values or value_confs:
            print("SampleMonitorConfChangeCallback's reconfiguration :")
            # if there are constant values, let's print them. Other manipulations can be done as well...
            for key in const_values.keys():
                print(f"constant value {key} = {const_values[key]}")

            # if there are value configurations, let's print them. Other manipulations can be done as well...
            for idx, value_conf in enumerate(value_confs):
                print(f"Value Configuration {idx} :")
                print(f" - Name     : {value_conf.getName()}")
                print(f" - Type     : {value_conf.getType()}")
                print(f" - UUID     : {value_conf.getUuid()}")
                print(f" - IsValid? : {value_conf.isValid()}")


if __name__ == '__main__':
    try:
        print(f"Heex SDK version : {HEEX_SYSTEM_SDK_VERSION}")
        # TODO: Replace the Monitor UUID with the one provided (by the Web Platform) for your Trigger or give a UUID as an input to the generated executable
        monitor_uuid = "M-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)"
        if len(sys.argv) > 1:
            monitor_uuid = sys.argv[1]

        server_ip = "127.0.0.1"
        server_port = 4242

        print(f"{sys.argv[0]} will be exposed as {monitor_uuid} tries to connect to {server_ip}:{server_port}.")

        # configure and launch the Monitor
        # NOTE: at the creation of the instance, the initial configuration will trigger the onConfigurationChangedCallback to be called.
        # NOTE: Any future reconfiguration (ie: OTA) will also trigger it.
        sampleInstance = SampleMonitorConfChangeCallback(monitor_uuid, server_ip, server_port)
        sampleInstance.awaitReady()
        while True:
            state = random.choice([True, False])
            print("state is", state)
            sampleInstance.updateValue(state)
            time.sleep(1)

    except Exception as e:
        print("Error", e)
        exit(1)
