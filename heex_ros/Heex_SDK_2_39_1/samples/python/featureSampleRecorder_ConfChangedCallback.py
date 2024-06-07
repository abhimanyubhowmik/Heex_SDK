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


# This SampleRecorderConfChangedCallback class, which inherits from Heex's agent Recorder class, is created to
# demonstrate how the feature onConfigurationChangedCallback() can be used. The recorder behavior's part is the same as
# the one in the basicSampleRecorder
class sampleRecorderConfChangedCallback(Recorder):
    def __init__(self, uuid, server_ip, server_port, implementation_version="1.0.0"):
        Recorder.__init__(self, uuid, server_ip, server_port, implementation_version)
        print(f"sampleRecorderConfChangedCallback | instance created: {uuid}, {server_ip}:{server_port}")

    #! [Tested Feature] Callback function that shall be called during each configuration change
    def onConfigurationChangedCallback(self):
        """
        [Sample Key Feature] Callback function that shall be called during each configuration change (called in onConfigurationChanged).
        In this function, we simply fetch all the new Recorders's valueCOnfigurations and Constant Values, and print them out
        """
        const_values = self.getConstantValues()
        value_confs = self.getValueConfigurations()
        if const_values or value_confs:
            print("\\sampleRecorderConfChangedCallback's reconfiguration :")
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
        # NOTE: In this example, we shall create and upload a simple text file that contains "This is Smart Data" and a timestamp in it.
        # NOTE: we are handeling all paths to comply with UTF-8 chars, but if you are only using ASCII characters it is not mandatory

        # Step 1: Define the dedicated file name for this recorder.
        recordingFilename = Path(__file__).resolve().parent / ("file-" + query.uuid + query.eventUuid + ".txt")

        # Step 2: Write data in the recording's text file
        print("sampleRecorderConfChangedCallback | Creating file at: ", str(recordingFilename).encode('utf-8', 'surrogateescape'))
        with open(recordingFilename, "w") as f_out:
            f_out.write(f"Empty recording mock data for timestamp: {query.timestamp}")

        # Step 3: Return the path to the event recording file.
        # NOTE: the user has 3 options during this step. Instead of pointing to the given file, you can chose to point to a whole folder,
        # NOTE: or you can ask for the content of a whole folder to be uploaded without the folder itself, by adding a /* at the end of the path.
        filepath = str(recordingFilename.resolve()).encode('utf-8', 'surrogateescape')
        print(f"sampleRecorderConfChangedCallback | Recording folder: {filepath}")

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
        # NOTE: at the creation of the instance, the initial configuration will trigger the onConfigurationChangedCallback to be called.
        # NOTE: Any future reconfiguration (ie: OTA) will also trigger it.
        sampleRecorderInstance = sampleRecorderConfChangedCallback(recorder_uuid, server_ip, server_port)
        sampleRecorderInstance.awaitReady()  # awaits the connection with the kernel.

        while True:  # wait for request indefinitly, until user break or kernel break
            time.sleep(1)

    except Exception as e:
        print("Error", e)
        exit(1)
