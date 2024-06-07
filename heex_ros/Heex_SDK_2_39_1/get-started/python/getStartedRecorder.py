#!/usr/bin/env python3
import sys
import os
import time

# Adding the PyHeexCustomerSide Lib to PYTHONPATH. Depending on your folder structure, you might need to update this block
sys.path.insert(1, "../sdk/lib")
sys.path.insert(1, "../../sdk/lib")
sys.path.insert(1, "../lib")
from PyHeexCustomerSide import Recorder


class getStartedRecorder(Recorder):
    def __init__(self, uuid, serverIp, serverPort):
        Recorder.__init__(self, uuid, serverIp, serverPort)
        print("GetStartedRecorder | ", uuid, " ", serverIp, " ", serverPort)

    # ! Override the pure virtual function of the Recorder class to define the logic of getting data values advertized by the getStartedRecorder.

    def generateRequestedValues(self, query, contextValues):
        # TODO: Fill the variable `contextValues` with all the values for each of the ContextValue keys in the Recorder query. We recommend to use the addContextValue method. The keys for a Recorder are listed in the Recorder implementation panel.
        # Use self.getContextValueKeys(contextValues) to obtain all required keys from the query (optional check).
        # Use self.addContextValue(contextValues, key, value) to assign the value to the key. Values are defaulted to an empty string.
        # Use self.addGNSSContextValue(contextValues, "position", latitude, longitude) to assign separate latitude and longitude to the key "position".
        # Access query info for Recorder uuid (query.uuid), Event uuid (query.eventUuid) and requested timestamp (query.timestamp). See RecorderContextValueArgs structure.

        # Return true when all the context keys have their values added in `contextValues`.
        return (True, contextValues)

    # ! Override the pure virtual function of the Recorder class to define the logic of getting data file paths.

    def generateRequestedFilePaths(self, query, filepath):
        # TODO: Customize the value of the filepath in the `filepath` variable with the event recording part filepath(s) with the desired option.
        # Each of the return options has a specific syntax to match a use case:
        # - 1: Send the file only. Return the direct path to file (no modification required).
        # - 2: Send the folder and all its content. Return the direct path to folder (no modification required, all folder content will be included).
        # - 3: Send only the content of the folder. Return the path to the folder with "/*" as an extra suffix (wildcard at the end to include only files within the parent directory).

        #   Return true when the path to the event recording part have been replaced in `filepath`. File(s) shall have been completely generated.
        return (False, filepath.encode('utf-8', 'surrogateescape'))


if __name__ == "__main__":

    try:

       # Configure the Recorder
        recorderUuid = "R-450dee83-d5f0-4a71-b984-f4699fe93060(1.0.0)"
        serverIp = "127.0.0.1"
        serverPort = 4243

        # launch monitor
        getStartedRecorder = getStartedRecorder(recorderUuid, serverIp, serverPort)

        timeToQuit = False
        hasBeenConnected = False

        # Keep Recorder active while connection with the SDE is active

        while not timeToQuit:
            time.sleep(0.1)
            if not getStartedRecorder.isConnected():
                print("has been disconnected")
                if hasBeenConnected:
                    timeToQuit = True
            else:
                hasBeenConnected = True

    except Exception as e:
        print("Error", e)
        exit(1)
