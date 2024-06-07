#!/usr/bin/env python3
import sys
import os

# Adding the PyHeexCustomerSide Lib to PYTHONPATH. Depending on your folder structure, you might need to update this block
sys.path.insert(1, "../sdk/lib")
sys.path.insert(1, "../../sdk/lib")
sys.path.insert(1, "../lib")
from PyHeexCustomerSide import BooleanMonitor


class GetStartedMonitor(BooleanMonitor):
    def __init__(self, uuid, serverIp, serverPort):
        BooleanMonitor.__init__(self, uuid, serverIp, serverPort)


if __name__ == "__main__":
    try:
        # TODO: Replace this left Monitor UUID with the one provided (by the Web Platform) for your Trigger
        monitorUuid = "D-d6fa84b8-26cb-44b6-9bde-89ec3ef5b4d9(1.0.0)"
        if len(sys.argv) >= 2:
            monitorUuid = sys.argv[1]

        #
        # Configure the Monitor
        #
        serverIp = "127.0.0.1"
        serverPort = 4242
        print("Monitor_uuid = ", monitorUuid)
        print("serverIp = ", serverIp)
        print("serverPort = ", serverPort)

        #
        # Launch Monitor
        #
        getStartedMonitor = GetStartedMonitor(monitorUuid, serverIp, serverPort)
        getStartedMonitor.awaitReady()
        getStartedMonitor.updateValue(True, getStartedMonitor.getTimestampStr())

    except Exception as e:
        print("Error", e)
        exit(1)
