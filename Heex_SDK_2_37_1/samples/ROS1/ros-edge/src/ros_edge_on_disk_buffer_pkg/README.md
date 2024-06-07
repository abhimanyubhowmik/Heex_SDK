# Heex SDK Example Ros Edge
This project is the example of the recommended SDK usage for a ROS1 package that implements Monitors and Recorders for the edge use case.  
It implements logics of monitoring and recording that interface with ROS. This example subscribes to the ROS topics from the example bag and perform the detection and recording activities based on the ROS message callbacks.
To mimic ROS1 edge operations, you need to replay the bag using the `rosbag play example_offline_extraction.bag` command to obtain a similar environment to edge. You may want to accelerate the replay speed using the ` --rate 8` option to the previous command. Make sure that the rosmaster is also running using `roscore`.

## Project content
Main directories and files of interest are:
* `ros-edge`: This main folder of the sample comes structured as a ROS1 workspace. It will be used as such in the Deployment and Building steps.
* `src/ros_edge_on_disk_buffer_pkg`: ROS package that imports and uses of the HeexCustomerSide library (HeexSDK).
  * Building this package will generate a ROS node named `rosEdgeOnDiskBuffer` that will perform the monitoring or recording logics on all messages of interest.
  * It instanciates the Monitor and Recorder as one ROS node and perform the monitoring or recording logic on all messages of interest.
  * This node is implemented in the file `main.cpp` and operates one `MonitorADTransitionDisengagement` and one `RosBagRecorder`.
* `...ros_edge_pkg/src/MonitorADTransitionDisengagement`: Contains header and source for the Monitor interface that monitors the ADTransitionDisengagement condition using ROS message subscription and callback.
    * The Monitor's logic is implemented within the reception of new `Bool` messages.
    * It sends the signal value to the Heex SDE everytime using `updateValue`.
* `...ros_edge_pkg/src/RosBagRecorder`: Contains header and source for Recorder interface. It exposes and performs the extraction of the ContextValue `position` and the generation of the Event Recording Part as a smaller rosbag using ROS message subscription and type-agnostic callbacks.
    * The Recorder's rosbag generation logic is implemented and performed within the `BagRecorder` utility class.
    * It uses an internal heex class that writes overlapping bags on disk.

## Usage
This sections details how to deploy, build, source and run the ROS1 ros-edge sample.
This ROS1 sample has been tested with the following ROS1 distributions: Noetic(Ubuntu 20.04), Melodic(Ubuntu 18.04)
### Deployment
First, the main folder `ros-edge` comes structured as a ROS1 workspace. Thus, there is no need to copy this folder in any workspace except if you want to use it in your own.


Second, this example requires the Kernel to have a trigger dedicated to operate the demo. For this purpose, you need to use the web platform to:
- Create a trigger and name it HeexDemoEdgeTrigger.
- Create a HeexDemoEdgeMonitor Monitor related to a boolean signal.
- Create a HeexDemoEdgeRecorder Recorder. You may select the Context Values `"position"` and `"bag_timestamp"` if they are available. Make sure you select a recording range of less than 10s.
  - Note that the buffer size is actually set to 15s in the implemention. If you want to modify this value, please refer to the Implementation notes section.
- Export your system configuration as a systemConf.json file and copy it into the Kernel folder.


Finally, as your newly created HeexDemoEdgeMonitor and HeexDemoEdgeRecorder have different UUIDs than the one shipped in the code, you will need to modify their value.
- For Monitor, edit it in `MonitorADTransitionDisengagement.cpp` in `BooleanMonitor("M-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)", serverIp, serverPort)`
- for Recorder, edit it in `main.cpp` in `rosbagRecorder("R-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)", serverIp, serverPort)`


### Building
#### SDK build
Before building this example ROS package for edge applications, make sure the Heex SDK has been built within your machine.

#### Rosbags
The demo rosbag can be retrieved using this [rosbag](https://heex-public-sample-datasets.s3.eu-west-1.amazonaws.com/example_offline_extraction.bag)  

#### Example build for Linux 20.04 and above  
The following buildings steps apply to Linux 20.04 and above.
Legacy platforms currently covered are Ubuntu 18.04. Their build varies and the alternative steps are detailed in the next sections.

The `catkin_make` command is a convenience tool in ROS for initializing and building catkin workspaces. Running it the first time in your `ros-edge` workspace will create a .catkin_workspace in ros-edge and a CMakeLists.txt file in your 'src' folder. You can have more details on ROS1 workspaces in this [ROS1 workspace tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
In addition, you need to add an extra argument to set the HEEX_SDK_DIR when perform the usual 'catkin_make' command.
- Term 1: Run your catkin make command
  ```bash
  cd ~/Heex_SDK_X_XX_X/samples/ROS1/ros-edge/
  catkin_make -DHEEX_SDK_DIR=~/Heex_SDK_X_XX_X
  ```

#### Legacy platforms
##### 18.04
If your platform OS is 18.04 and you plan to use the Heex SDK in a ROS melodic environment (requires Boost 1.65.1), you will need to follow these steps modifications.
First, ensure to have your `libraries` and `sdk` packages built for 18.04.
Second, add the `legacy1804HeexSdk` flag to build for 18.04 in addition to the HEEX_SDK_DIR path:
  ```bash
  cd ~/Heex_SDK_X_XX_X/samples/ROS1/ros-edge/
  catkin_make -DHEEX_SDK_DIR=~/Heex_SDK_X_XX_X -Dlegacy1804HeexSdk:BOOL=TRUE
  ```

### Sourcing
Since we have initialized our new ros-edge workspace, we will need to source it to overlay it on top of your environment. To understand more about this, see the general [catkin_make  documentation](http://wiki.ros.org/catkin/commands/catkin_make). Before continuing rememver to source your new setup.*sh file in every future terminal:
  ```bash
  source ~/Heex_SDK_X_XX_X/samples/ROS1/ros-edge/devel/setup.bash
  ```

### Running
You can run the example with the followings commands in a well sourced and operational ROS environment.

- Term 1: Run the ROS core node
  ```bash
  roscore
  ```
- Term 2: Run the rosEdgeOnDiskBuffer node that run the MonitorADTransitionDisengagement and RosbagRecorder
  ```bash
  rosrun ros_edge_on_disk_buffer_pkg rosEdgeOnDiskBuffer
  ```
- Term 3: Run the heex Kernel
  ```bash
  cd ~/Heex_SDK_X_XX_X/
  ./runHeexKernel.sh
  ```
- Term 4: Start the rosbag replay (mimic a live ROS environment)
  ```bash
  rosbag play -s 190 ~/example_offline_extraction.bag
  ```

If something went wrong, have you:
- Made sure each terminal were sourced with the Sourcing command above.
- Noted that the heex Kernel command required to be run in a specific folder.


## Implementation notes
**In this sample, time is managed using message reception time and not the timestamp present in the ros message. As not all custom messages contain a timestamp in their definition, we went with *reception time*. This also allows us to handle serialized messages directly. Only the Monitor or Recorder Context Values require their deserialization to inspect and use their content.**

The `MonitorADTransitionDisengagement` subscribes to the `/demo/bool` topic and performs the monitoring logic every callback. This topic contains only a boolean to mimic a status dataflow that an AD system in operation would produce. Each callback updates the internal states to determine if the dataflows when from `True` to `False`, such as when a Disengagement happens.

The `RosBagRecorder` continously creates bags that contains messages of a fixed duration. Only two bags are on disk.
It covers the expected recording duration. 

- There is a memory check that is done to ensure one can write the bags on the disk. The current minimun size is 1Gb.
- There is a limit of 10000 messages in the in-memory queue.

## Adaptation notes
The structure of this example shows a possibility to structure your ROS collection of Monitors and Recorders.

To adapt this example project for your dataflows and rosbag, you will need to configure the following:

- Your Monitor will probably need some specific custom states. They can be included in the custom monitor class as demonstrated in the `MonitorADTransitionDisengagement`.
- Modify the list of buffered topics `_topics` to buffer to include your selection of topics to record:
  - In file `RosbagRecorder.cpp`, function `RosBagRecorder::RosBagRecorder`, line `_topics = {"/gps/fix", "/demo/bool", "/imu"};`
- If you prefer to register all available topics instead, set `_topics` to `{}`
- To record a new context value, 
  - create a struct for your message (i.e PositionMessage)
  - create a queue and a callback for your message (i.e contextValueCallback() , std::queue<...>)
  - Implement extractContextValue following the example bool RosBagRecorder::extractContextValue(
    const Heex::RecorderArgs::RecorderContextValueArgs &query,
    sensor_msgs::NavSatFix::ConstPtr &msg, ros::Time &time)
