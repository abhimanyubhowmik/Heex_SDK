#!/usr/bin/env python3
#
# Copyright (c) 2024 Heex Technologies
# All Rights Reserved.
#
# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
#
import sys
import time

# Adding the PyHeexCustomerSide Lib to PYTHONPATH. Depending from where you run this sample, you might need to update it to point to the lib
sys.path.insert(1, "../sdk/lib")
sys.path.insert(1, "../../sdk/lib")

from PyHeexCustomerSide import StringMonitor
from PyHeexCustomerSide import __version__ as HEEX_SYSTEM_SDK_VERSION


# This basic StringMonitor Sample creates an sample_string_monitor instance. It'll then update some string values sent to the monitor, and if
# the given string is equal to the monitored string set during the creation of Monitor, it activates the trigger
if __name__ == "__main__":
    try:
        print(f"Heex SDK version : {HEEX_SYSTEM_SDK_VERSION}")
        # TODO: Replace the Monitor UUID with the one provided (by the Web Platform) for your Trigger or give a UUID as an input to the generated executable
        monitor_uuid = "M-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)"
        if len(sys.argv) > 1:
            monitor_uuid = sys.argv[1]

        server_ip = "127.0.0.1"
        server_port = 4242

        print(f"{sys.argv[0]} will be exposed as {monitor_uuid} tries to connect to {server_ip}:{server_port}.")

        # configure and launch Monitor
        sample_string_monitor = StringMonitor(monitor_uuid, server_ip, server_port)
        sample_string_monitor.awaitReady()  # awaits the connection with the kernel.
        input_strings = ["Roundabout", "Highway", "Animal Crossing", "City center", "Some other random string"]
        cntr = 0
        while True:
            print(f"input_string : {input_strings[cntr % len(input_strings)]}")
            sample_string_monitor.updateValue(input_strings[cntr % len(input_strings)])
            cntr += 1
            time.sleep(1)
        exit(0)

    except Exception as e:
        print("Error", e)
        exit(1)
