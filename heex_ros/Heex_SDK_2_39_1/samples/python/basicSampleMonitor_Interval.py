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
import random

# Adding the PyHeexCustomerSide Lib to PYTHONPATH. Depending from where you run this sample, you might need to update it to point to the lib
sys.path.insert(1, "../sdk/lib")
sys.path.insert(1, "../../sdk/lib")

from PyHeexCustomerSide import IntervalMonitor
from PyHeexCustomerSide import __version__ as HEEX_SYSTEM_SDK_VERSION


# This basic IntervalMonitor Sample creates a sample_interval_monitor instance. It'll then update the instance's humidity percentage it
# receives with random values, and if the percentage is within one of the provided intervals (provided during the creation
# of the monitor on the platform), the monitor activates.
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

        # configure and launch Monitor
        sample_interval_monitor = IntervalMonitor(monitor_uuid, server_ip, server_port)
        sample_interval_monitor.setSignalUnit("%")  # Provide the unit of input data
        sample_interval_monitor.awaitReady()        # awaits the connection with the kernel.
        while True:
            humidity = random.randint(0, 100000) / 1000  # random humidity between 0 and 100 %
            print(f"Humidity is at {humidity} {sample_interval_monitor.getSignalUnit()}")
            sample_interval_monitor.updateValue(humidity)
            time.sleep(1)

    except Exception as e:
        print("Error", e)
        exit(1)
