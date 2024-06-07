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
from pathlib import Path

# Adding the PyHeexCustomerSide Lib to PYTHONPATH. Depending from where you run this sample, you might need to update it to point to the lib
sys.path.insert(1, "../sdk/lib")
sys.path.insert(1, "../../sdk/lib")

from PyHeexCustomerSide import Recorder
from PyHeexCustomerSide import __version__ as HEEX_SYSTEM_SDK_VERSION

# This SampleRecorder class, which inherits from Heex's agent Recorder class, is created to demonstrate how a
# basic example of a recorder can be implemented and used


class sampleRecorder(Recorder):
    def __init__(self, uuid, server_ip, server_port, implementation_version="1.0.0"):
        Recorder.__init__(self, uuid, server_ip, server_port, implementation_version)
        print(f"sampleRecorder | instance created: {uuid}, {server_ip}:{server_port}")

    # ! Override the virtual function of the Recorder class to define the logic of getting data values advertized by the SampleRecorder.
    def generateRequestedValues(self, query, contextValues):
        # NOTE: this function has the same behavior as what was done during the onboarding's 'get-started' example.
        #! this will add the given position as a context values.
        latitude = 37.795739190321555
        longitude = -122.42351359240077
        self.addGNSSContextValue(contextValues, "position", latitude, longitude)

        # Paste your custom key and value pair below
        self.addContextValue(contextValues, "my_custom_key", "my_custom_value")

        return (True, contextValues)

    # ! Override the virtual function of the Recorder class to define the logic of getting data file paths.
    def generateRequestedFilePaths(self, query, filepath):
        # NOTE: In this basic example, we shall create a simple text file that contains "This is Smart Data" and a timestamp in it.
        # NOTE: We will then send the whole folder, 'RecordingCache', containing this text file to the cloud as our recording sample
        # NOTE: we are handeling all paths to comply with UTF-8 chars, but if you are only using ASCII characters it is not mandatory

        # Step 1: First we create the directory if it doesn't exist
        relativePath = Path(__file__).resolve().parent / "RecorderCache"
        relativePath.mkdir(parents=True, exist_ok=True)
        print(f"SampleRecorder | Using operating directory: {str(relativePath).encode(sys.stdout.encoding, 'replace')}")

        # Step 2: Define the dedicated file name for this recorder.
        # Generate a unique name to avoid file name collision when multiple recorders write in same folder
        recordingEventPartFilename = "file-" + query.uuid + query.eventUuid + ".txt"
        print("SampleRecorder | recordingEventPartFilename = ", str(recordingEventPartFilename).encode('utf-8', 'surrogateescape'))
        recordingPath = relativePath / recordingEventPartFilename

        # Step 3: Write data in the recording's text file
        print("SampleRecorder | Creating file at     : ", str(recordingPath.resolve()).encode('utf-8', 'surrogateescape'))
        with open(recordingPath, "w") as f_out:
            f_out.write(f"Empty recording mock data for timestamp: {query.timestamp}")

        # Step 4: Return the path to the event recording folder.
        # NOTE: the user has 3 options during this step. Instead of sending the file only, you can chose to point to a whole folder,
        # NOTE: or you can ask for the content of the whole folder to be uploaded without the folder itself, by adding a /* at the end of the path.
        filepath = str(recordingPath.resolve()).encode('utf-8', 'surrogateescape')
        print(f"SampleRecorder | Recording folder: {filepath}")

        # The filepath been successfully generated and assigned to the `filepath` variable.
        # WARNING: filepath is a string, not a pathlib.Path
        return (True, filepath)


if __name__ == "__main__":
    try:
        print(f"Heex SDK version : {HEEX_SYSTEM_SDK_VERSION}")
        # TODO: Replace the Recorder UUID with the one provided (by the Web Platform) for your Trigger or give a UUID as an input to the generated executable
        recorder_uuid = "R-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)"
        if len(sys.argv) > 1:
            recorder_uuid = sys.argv[1]

        server_ip = "127.0.0.1"
        server_port = 4243

        print(f"{sys.argv[0]} will be exposed as {recorder_uuid} tries to connect to {server_ip}:{server_port}.")

        # configure and launch the Recorder
        sampleRecorderInstance = sampleRecorder(recorder_uuid, server_ip, server_port)
        sampleRecorderInstance.awaitReady()  # awaits the connection with the kernel.

        while True:  # wait for request indefinitly, until user break or kernel break
            time.sleep(1)

    except Exception as e:
        print("Error", e)
        exit(1)
