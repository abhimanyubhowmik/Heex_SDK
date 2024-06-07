# Heex SDK Example Ros2 Edge
This project is the example of the recommended SDK usage for a ROS2 package that implements Monitors and Recorders for the edge use case.  
It implements logics of monitoring and recording that interfaces with ROS2. This example subscribes to the ROS2 topics from the example bag and perform the detection and recording activities based on the ROS2 message callbacks.
To mimic ROS2 edge operations, you need to replay the bag using the `ros2 bag play example_offline_extraction.db3` command to obtain a similar environment to edge (if you want to run it at a higher rate, you can with `-r 2`, but be aware that higher rates can have unwated behaviors)

## Project content
Main directories and files of interest are:
* `ros2-edge`: This main folder of the sample comes structured as a ROS1 workspace. It will be used as such in the Deployment and Building steps.
* `src/ros2_edge_in_memory_buffer_pkg`: ROS2 package that imports and uses of the HeexCustomerSide library (HeexSDK).
  * Building this package will generate a ROS2 node named `ros2EdgeInMemoryBuffer` that will perform the monitoring or recording logics on all messages of interest.
  * It instanciates the Monitor and Recorder as two different ROS2 nodes (`ros2_edge_in_memory_buffer_monitorNode` and `ros2_edge_in_memory_buffer_recorderNode`) and perform the monitoring or recording logic on all messages of interest.
  * Additionnaly, a third node is instantiated, `singleMessageCaches_node`, that will subscribe to any instant messages if user requested for the extraction of some *static* data 
  * These nodes are implemented in the files `Ros2MonitorADTransitionDisengagement.cpp`, `Rosbag2RecorderSnapshotter.cpp` and `Snapshotter.cpp` and are operated from the `Ros2EdgeInMemoryBuffer.cpp` file.
* `...ros_edge_pkg/src/Ros2MonitorADTransitionDisengagement`: Contains the source for the Monitor interface that monitors the ADTransitionDisengagement condition using ROS2 message subscription and callback.
    * The Monitor's logic is implemented within the reception of new `Bool` messages.
    * It sends the signal value to the Heex Kernel everytime using `updateValue`.
* `...ros_edge_pkg/src/Rosbag2RecorderSnapshotter`: Contains the source for Recorder interface. It exposes and performs the extraction of the ContextValue `position` and the generation of the Event Recording Part as a smaller rosbag using ROS2 message subscription and type-agnostic callbacks.
    * The Recorder's rosbag generation logic is implemented and performed within the `Snapshotter` utility class.
    * In this edge extraction use case, the `Snapshotter` bufferizes the messages by timestamp and export the pre and post-event data using the timestamps and time range provided by the kernel.
    * It also handles the extraction of some *static* data that can be published outside of this time range like `tf_static` messages or calibration data (eg. LIDAR, camera). The `Snapshotter` offers the option to bufferize and write the latest message per topic at the start of each recorded bag.

## Usage
This sections details how to deploy, build, source and run the ROS2 ros2-edge sample.
This ROS2 sample has been tested with the following ROS2 distributions: Galactic(Ubuntu 20.04), Humble(Ubuntu 22.04)
### Deployment
First, the main folder `ros2-edge` comes structured as a ROS2 workspace. Thus, there is no need to copy this folder in any workspace except if you want to use it in your own.

Second, this example requires the Kernel to have a trigger dedicated to operate the demo. For this purpose, you need to use the web platform to:
- Create a trigger and name it HeexDemoEdgeTrigger.
- Create a HeexDemoEdgeMonitor Monitor related to a boolean signal.
- Create a HeexDemoEdgeRecorder Recorder. You may select the Context Values `"position"` and `"bag_timestamp"` if they are available. Make sure you select a recording range of less than 10s.
  - Note that the buffer size is actually set to 15s in the implemention. If you want to modify this value, please refer to the Implementation notes section.
- Export your system configuration as a systemConf.json file and copy it into the Kernel folder.


Finally, as your newly created HeexDemoEdgeMonitor and HeexDemoEdgeRecorder have different UUIDs than the one shipped in the code, you will need to modify their value.
- For Monitor, edit it in `Ros2MonitorADTransitionDisengagement.cpp` in `BooleanMonitor("M-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)", serverIp, serverPort)`
- for Recorder, edit it in `Rosbag2RecorderSnapshotter.cpp` in `Recorder("R-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)", serverIp, serverPort)`


### Building
#### SDK build
Before building this example ROS2 package for edge applications, make sure the Heex SDK has been built within your machine.

#### Rosbags
The demo rosbag can be retrieved using this [rosbag](https://heex-public-sample-datasets.s3.eu-west-1.amazonaws.com/example_offline_extraction_withinstant/example_offline_extraction_withinstant.db3)  

#### Example build for Ubuntu 22.04 (Humble Hawksbill)
This example has been implemented and tested with ROS2 Humble Hawskbill and Ubuntu 22.04. For information, Humble LTS is only available on 22.04.
- Are you using another ROS2 version ? We recommend the use of a Docker Ubuntu 22.04 environment to build and try this example
- Are you on another OS? You may experience possible issues at build time 

Please make sure you specify the path to the HEEX_SDK_DIR by using `--ament-cmake-args -DHEEX_SDK_DIR=/path/to/heex-sdk-dir` after the `colcon build` command or follow the ADAPTATION NOTES in the ros2-edges `CMakeLists.txt` file
  ```bash
    cd ~/Heex_SDK_X_XX_X/samples/ROS2/ros2-edge
    source /opt/ros/humble/setup.bash
    colcon build --ament-cmake-args -DHEEX_SDK_DIR=/path/to/heex-sdk-dir
  ```
You can specify the number of processes that run in parallel by adding following argument and modifying the NUM_OF_PROCESSES `--parallel-workers NUM_OF_PROCESSES`
Once build is finished, source the installed setup.bash by running `source install/setup.bash`. Feel free to run `echo 'source ~/Heex_SDK_X_XX_X/samples/ROS2/ros2-edge/install/setup.bash' > ~/.bashrc` to add it to your .bashrc


### Running
You can run the example with the followings commands in a well sourced and operational ROS environment.

- Term 1: Run the ROS2 package node
  ```bash
  ros2 run ros2_edge_in_memory_buffer_pkg ros2EdgeInMemoryBuffer
  ```
- Term 2: Run the heex Kernel
  ```bash
  cd ~/Heex_SDK_X_XX_X/
  ./runHeexKernel.sh
  ```
- Term 3: Start the rosbag replay (mimic a live ROS2 environment)
  ```bash
  ros2 bag play example_offline_extraction.db3
  ```

If something went wrong, have you:
- Made sure each terminal were sourced with the Sourcing command above.
- Noted that the heex Kernel command required to be run in a specific folder.


## Implementation notes
**In this sample, time is managed using message reception time and not the timestamp present in the ros message. As not all custom messages contain a timestamp in their definition, we went with *reception time*. This also allows us to handle serialized messages directly. Only the Monitor or Recorder Context Values require their deserialization to inspect and use their content.**


The `Ros2MonitorADTransitionDisengagement` subscribes to the `/demo/bool` topic and performs the monitoring logic every callback. This topic contains only a boolean to mimic a status dataflow that an AD system in operation would produce. Each callback updates the internal states to determine if the dataflows when from `True` to `False`, such as when a Disengagement happens.

The `Rosbag2RecorderSnapshotter` uses an adaptated version of the `Snapshotter` class and dependencies from the `rosbag2_snapshot` package. [This package](https://github.com/gaia-platform/rosbag2_snapshot), under BSD license, provides structures and logic implementations of proven extraction methods in ROS2, and acts similarly to the ROS1's rosbag_snaphot package. Heex has adapted it to keep its core functionnalities while extending its controlability through the Heex SDK using Recorders:

- A Message structure to store ROS message agnostic to its type (`SnapshotMessage` struct).
- A Buffer structure to store Messages and maintain a cyclic buffer for a dedicated topic (`MessageQueue` class) with options (`SnapshotterTopicOptions` struct).
- A Manager class to initialize and operate Buffers (`Snapshotter` class). It also runs data extraction from the buffers for a given request.

This provided adaptation inherits some limitations of the inital Snapshotter on top of new ones that are:
  - the extraction can only happen one at a time: parallel extraction is not supported
  - the addition of new message in the cyclic buffer is prevented during the bag snapshot and all buffers get empty once snapshot is completed. Thus, any extraction close to each other in time (ie. less than buffer size) may contain only part of the data.
  - If you use the extraction of some *static* data from topic like `tf_static`, the node shall be run prior to the initial message to bufferize it. If no message are found, a warning will be issued at extraction stating that no message have been bufferized for the topic.

## Adaptation notes
The structure of this example shows a possibility to structure your ROS2 collection of Monitors and Recorders. We have purposely stored our Monitor and Recorder original code outside of the ROS2 `ros2_edge_in_memory_buffer_pkg` package. This way, they can be single code repository and reused in other ROS packages using symlinks.

To adapt this example project for your dataflows and rosbag, you will need to configure the following:

- Your Monitor will probably need some specific custom states. They can be included in the custom monitor class as demonstrated in the `Ros2MonitorADTransitionDisengagement`.
  - If the signal monitored has a specific Quality Of Services (QoS) that are not compatible with the default one, you will need to specify the profile and replace the `topicQosProfile` given in the sample. 
- Modify the list of buffered topics `_topics` to buffer to include your selection of topics to record:
  - In file `Rosbag2RecorderSnapshotter.cpp`, function `Rosbag2RecorderSnapshotter::Rosbag2RecorderSnapshotter`, lines:
  ```cpp
    _topics = {
        heex_rosbag2_snapshot::TopicDetails("/gps/fix", "sensor_msgs/msg/NavSatFix"),
        heex_rosbag2_snapshot::TopicDetails("/demo/bool", "std_msgs/Bool", specific_qos_profile),
        heex_rosbag2_snapshot::TopicDetails("/imu", "sensor_msgs/msg/Imu")};
  ```
  make sure you add the proper topic types associated to the topic names
  - If your use case uses signals that have specific Quality Of Services (QoS) that are not compatible with the default ones, you will need to specify the profile for each topic that needs a different QoS than the default `QoS{10}` (as shown on the second line above)
- If you prefer to register all available topics instead, set the SnapshotterOptions option `all_topics` to `true`.
  - In file `Rosbag2RecorderSnapshotter.cpp`, function `Rosbag2RecorderSnapshotter::Rosbag2RecorderSnapshotter`, line `_all_topics = false;`
  - In this case, you don't need to worry about QoS profiles, as each QoS profile will be read from the published signal before applying it to the subscriptions.
- In both cases, make sure to update how Context Values like GNSS position are extracted from your topics list.
  - You can update the name of the topic that contains the GPS data in the hardcoded value line `_topicNameGps = "/gps/fix"`
  - The topic is set using the index in `Rosbag2RecorderSnapshotter::generateRequestedValues`, lines following `auto req = std::make_shared<heex_rosbag2_snapshot::SnapshotInstantRequest>(`
- Ensure that the buffer size is enough for the Recorder definition (default options get to 30s but we set it to 15s)
  - In file `Rosbag2RecorderSnapshotter.cpp`, function `Rosbag2RecorderSnapshotter::Rosbag2RecorderSnapshotter`, line `heex_rosbag2_snapshot::SnapshotterOptions options(rclcpp::Duration(15, 0));`.
- Make sure that this buffer size also cope with your system resources (memory).
- A well sourced environment is required when custom messages are used.
- Modify the list of buffered topics for latest message `topics_once_at_bagstart_` to buffer to include your static topics:
  - In file `Rosbag2RecorderSnapshotter.cpp`, function `Rosbag2RecorderSnapshotter::RosbagRecorderSnapshotter`, line `topics_once_at_bagstart_ = {"/tf_static"};`
- Depending on how the `Snapshotter` reconfigure after an OTA (live reconfiguration), you may need to adapt the code so that cleaning of the bufferized messages is prevented.

## Additional notes
- Our sample will generate recorded data into a folder that will have following structure:
  `/tmp/recording_<query.eventUuid>_<query.uuid>/`
                      `metadata.yaml`
                      `recording_<query.eventUuid>_<query.uuid>.db3`
- This folder will then be zipped and uploaded onto the webplatform. 
- User may decide to change the uploaded folder formatting: the web platform will accept 4 different cases:
  - folder containing `metadata.yaml` + one or several `.db3` files
  - directly `metadata.yaml` + one or several `.db3` files
  - folder containing one or several `.db3` files (no `metadata.yaml`)
  - directly one or several `.db3` files (no `metadata.yaml`)
- Please note that if user decides not to upload the `metadata.yaml` file, the bags will not be able to be visualized in Foxglove eenvironment, and any further conversions or usages requiering the `metadata.yaml` will not be possible without it.
