#!/usr/bin/env python3
#
# Copyright (c) 2024 Heex Technologies
# All Rights Reserved.
#
# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
#
import sys
from time import sleep

# Adding the PyHeexCustomerSide Lib to PYTHONPATH. Depending from where you run this sample, you might need to update it to point to the lib
sys.path.insert(1, "../sdk/lib")
sys.path.insert(1, "../../sdk/lib")

from PyHeexCustomerSide import ZoneMonitor
from PyHeexCustomerSide import __version__ as HEEX_SYSTEM_SDK_VERSION

# This basic ZoneMonitor Sample creates an sample_zone_monitor instance. It'll then update the GPS values of the instance,
# and if this position gets inside the set zone (provided during the creation of the monitor on the platform), it'll activate the monitor
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
        sample_zone_monitor = ZoneMonitor(monitor_uuid, server_ip, server_port)
        sample_zone_monitor.awaitReady()  # awaits the connection with the kernel.

        print("Test Paris 11")
        sample_zone_monitor.updateValue(48.86292854949911, 2.385822266652272)  # Paris 11
        sleep(1)

        print("Test Dijon")
        sample_zone_monitor.updateValue(47.59571647649772, 5.763069520668387)  # Dijon
        sleep(1)

        print("Test Chartres")
        sample_zone_monitor.updateValue(48.42727849702349, 1.4825034618277229)  # Chartres
        sleep(1)

        print("Test Beauvais")
        sample_zone_monitor.updateValue(49.420258402165906, 2.087570410410857)  # Beauvais
        sleep(1)

        print("Test Versailles")
        sample_zone_monitor.updateValue(48.79758334100409, 2.1318235573795965)  # Versailles

        exit(0)

    except Exception as e:
        print("Error", e)
        exit(1)
