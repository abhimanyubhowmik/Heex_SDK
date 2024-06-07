# apex_ai_heex_demo_pkg

This is the documentation for the `apex_ai_heex_demo_pkg` package.

## Purpose

This project is the example of SDK usage for ROS2 in the APEX AI environment that implements Monitors and Recorders for the edge use case.  
It implements logics of detection and recording that interface with ROS 2. This example subscribes to the ROS topics from a rosbag and perform the detection and recording activities based on the ROS message callbacks.

## Project content
files of interest are:
- `monitor_node`: source and header for the monitor node that monitors the disengagement condition using ROS 2 message subscription and callback.
  - The Monitor's logic is implemented within the reception of new `Bool` messages.
  - It sends the signal value to the Heex SDE everytime using       `updateValue`.
- `recorder_node`: A generic class for Recorder interface where nodes are created to subscribe to the necessary topics.
  In this example, it performs the extraction of the ContextValue `position` and the generation of the Event Recording Part as a smaller rosbag.
- `main`:
  - It instanciates the Monitor.
  - It instanciates the Recorder. More importantly, it adds a subscriber for each message type that is needed. The topics are specified also in this file.

## Usage
This sections details how to deploy, build, source and run the **apex_ai_heex_demo_pkg** sample.
### Prerequisite

It is assumed that you have an ApexAI environment setup.
Otherwise, follow this guide [Getting Started](https://apexai.pages-customer.apex.ai/apexos-docs/latest/guides/getting-started-with-apex-os/)


### Deployment
First, the main folder `apex_ai_heex_demo_pkg` comes structured as an ApexAI workspace. Thus, there is no need to copy this folder in any workspace except if you want to use it in your own.

Second, this example requires the Kernel to have a trigger dedicated to operate the demo. For this purpose, you need to use the web platform to:
- Create a trigger and name it HeexDemoEdgeTrigger.
- Create a HeexDemoEdgeMonitor Monitor related to a boolean signal.
- Create a HeexDemoEdgeRecorder Recorder. You may select the Context Values `"position"` and `"bag_timestamp"` if they are available. Make sure you select a recording range of less than 10s.
  - Note that the buffer size is actually set to 15s in the implemention. If you want to modify this value, please refer to the Implementation notes section.
- Export your system configuration as a systemConf.json file and copy it into the *Kernel/systemConfs* folder.

Finally, as your newly created HeexDemoEdgeMonitor and HeexDemoEdgeRecorder have different UUIDs than the one shipped in the code, you will need to modify their value.
- For Monitor, edit it in `monitor_node.cpp` in `BooleanMonitor("M-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)", serverIp, serverPort)`
- for Recorder, edit it in `recorder_node.cpp` in `Recorder("R-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)", serverIp, serverPort)`


### Building
#### SDK build
Before building this example ROS package for edge applications, make sure the Heex SDK has been built within your machine with flag `--with-python`.

#### Rosbags
The demo rosbag can be retrieved using this [rosbag](https://heex-public-sample-datasets.s3.eu-west-1.amazonaws.com/example_offline_extraction.db3)

#### ROS pkg build
You need to add an extra argument to set the HEEX_SDK_DIR when building.
  ```bash
    cd ~/Heex_SDK_X_XX_X/samples/ApexAI
    source /opt/ApexOS/setup.bash
    colcon build --merge-install --packages-select apex_ai_heex_demo_pkg --cmake-clean-cache --cmake-args 
    -DCMAKE_TOOLCHAIN_FILE=/opt/ApexTools/share/apex_cmake/cmake/apex_toolchainfile.cmake -DAPEX_PRINT_LOGS_TO_TERMINAL=ON -DHEEX_SDK_DIR=~/Heex_SDK_X_XX_X/    
  ```

### Running
It is expected that you have all well sourced environment.
Otherwise,
For each Terminal,
  ```bash
  source /opt/ApexOS/setup.bash
  source ~/Heex_SDK_X_XX_X/samples/ROS2/ApexAI/install/setup.bash
  ```
Terminal 1: Run the heex SDE
  ```bash
  cd ~/Heex_SDK_X_XX_X/
  ./runHeexKernel.sh
  ```
Terminal 2: Run the nodes
  ```bash
  cd ~/Heex_SDK_X_XX_X/samples/ROS2/ApexAI
  ros2 run apex_ai_heex_demo_pkg apex_ai_heex_demo_main_exe
  ```
Terminal 3: Start the rosbag replay (mimic a live ROS environment)
  ```bash
  ros2 bag play example_offline_extraction.db3
  ```

If something went wrong, have you:
- Made sure each terminal were sourced with the Sourcing command above.
- Noted that the heex Kernel command required to be run in a specific folder.

## Implementation notes
The `monitor_node` subscribes to the `/demo/bool` topic and performs the detection logic every callback. This topic contains only a boolean to mimic a status dataflow that an AD system in operation would produce. Each callback updates the internal states to determine if the dataflows when from `True` to `False`, such as when a Disengagement happens.

The `recorder_node` subscribes `/gps/fix`, `/imu` and `/demo/bool` using Apex AI Polling subscriptions. Apex Ai Middleware is used to buffer messages recevied for each topic.
- The QOS policy is set to `Keep Last` with a queue size of 100. You can adapt this value with respect to the capacity of your system.

## Adaptation notes
This example provides a starting point on how to combine the use of HeexSDK and ApexAI environment in ROS2.
A few extensions have been identified:
- Add new topics with the same topics types as the sample, update the `main.cpp` with your topcs.
  - In file `main.cpp`, function `main`, lines:
    ```bash
    std::vector<apex::string_strict256_t> topics = {"/gps/fix", "/demo/bool", "/imu"};
    ```
- Extension with new topics types, in addition to the update to `main.cpp` as the previous point, update `recorder_node.cpp`, function `RecorderNode::generateRequestedFilePaths` with the recommendation that are present at the beginning of the function
- A well sourced environment is required when custom messages are used.
- Modify the list of buffered topics for latest message `topics_once_at_bagstart_` to buffer to include your static topics:
  - In file `main.cpp`, function `main`, line `topics_once_at_bagstart{{"/tf_static", "tf2_msgs/msg/TFMessage"}};`

## Additional notes
- Our sample will generate recorded data into a folder that will have following structure:
  `~/Heex_SDK_X_XX_X/samples/ApexAI/recording_<query.eventUuid>_<query.uuid>/`
                      `metadata.yaml`
                      `recording_<query.eventUuid>_<query.uuid>.db3`
- This folder will then be zipped and uploaded onto the webplatform. 
- User may decide to change the uploaded folder formatting: the web platform will accept 4 different cases:
  - folder containing `metadata.yaml` + one or several `.db3` files
  - directly `metadata.yaml` + one or several `.db3` files
  - folder containing one or several `.db3` files (no `metadata.yaml`)
  - directly one or several `.db3` files (no `metadata.yaml`)
- Please note that if user decides not to upload the `metadata.yaml` file, the bags will not be able to be visualized in Foxglove eenvironment, and any further conversions or usages requiering the `metadata.yaml` will not be possible without it.

## Assumptions / Known limits
- Setting the buffer size can be tricky. For example, when set, if one replays at a higher speed, some messages needed to extract might be missing
- The ideal solution would be to a buffer with time instead of number of messages, it is not yet supported

## Rosbags
The example expects a bag in ROS2 format. If you have a rosbag in `bag` format, use rosbags-convert.  
convert `your_bag.bag` in a format supported by ROS2
  ```bash
  rosbags-convert your_bag.bag
  ```
You can also use `rosbags-convert` to convert ros2 bags into ros1 bags format, and use currently availaible tools for vizualization (E.g, Heex Studio).
## Contact ###
If you have any questions, feedback, or suggestions, feel free to contact us at contact@heex.io.