# Heex SDK Example Ros2 Edge In Memory V2 Buffer
This project is the example of the recommended SDK usage for a ROS2 package that implements Monitors and Recorders for the edge use case.  
It implements logics of monitoring and recording that interfaces with ROS2. This example subscribes to the ROS2 topics from the example bag and perform the detection and recording activities based on the ROS2 message callbacks.
Note that this sample is a new version of the `ros2_edge_in_memory_buffer_pkg`. This new version mainly adds the possibility to generate overlapping recordings with any custom interval between them, allows any recording range and has a refactored and simplified way of recording data using the BagRecorder.
To mimic ROS2 edge operations, you need to replay the bag using the `ros2 bag play example_offline_extraction.db3` command to obtain a similar environment to edge (if you want to run it at a higher rate, you can with `-r 2`, but be aware that higher rates can have unwated behaviors)

## Project content
Main directories and files of interest are:
* `ros2-edge`: This main folder of the sample comes structured as a ROS workspace. It will be used as such in the Deployment and Building steps.
* `src/ros2_edge_in_memory_buffer_v2_pkg`: ROS2 package that imports and uses of the HeexCustomerSide library (HeexSDK).
  * Building this package will generate an executable named `ros2EdgeInMemoryBufferV2` that will perform the monitoring or recording logics on all messages of interest.
  * It instanciates the Monitor and Recorder as two different ROS2 nodes (`ros2_edge_in_memory_buffer_v2_monitorNode` and `ros2_edge_in_memory_buffer_v2_bagRecorderNode`) and perform the monitoring or recording logic on all messages of interest.
  * These nodes are implemented in the files `SampleRos2BooleanMonitor.cpp` and `SampleRos2BagRecorder.cpp`, and are operated from the `Ros2EdgeInMemoryBufferV2.cpp` file.
* `...ros_edge_pkg/src/SampleRos2BooleanMonitor`: Contains the source for the Monitor interface that monitors the trigger condition using ROS2 message subscription and callback.
    * The Monitor's logic is implemented within the reception of new `Bool` (of name `/demo/bool`) messages.
    * It sends the signal value to the Heex Kernel everytime using `updateValue`.
* `...ros_edge_pkg/src/SampleRos2BagRecorder`: Contains the source for Recorder interface. It exposes and performs the extraction of the ContextValue `position` and the generation of the Event Recording Part as a smaller rosbag using ROS2 message subscription and type-agnostic callbacks.
    * The Recorder's rosbag generation logic is implemented and performed within the `BagRecorder` utility class.
    * In this edge extraction use case, the `BagRecorder` bufferizes the messages by timestamp and export the pre and post-event data using the timestamps and time range provided by the kernel.

## Usage
This sections details how to deploy, build, source and run the ROS2 ros2-edge sample.
This ROS2 sample has been tested with the following ROS2 distributions: Galactic(Ubuntu 20.04), Humble(Ubuntu 22.04)
### Deployment
First, the main folder `ros2-edge` comes structured as a ROS2 workspace. Thus, there is no need to copy this folder in any workspace except if you want to use it in your own.

Second, this example requires the Kernel to have a trigger dedicated to operate the demo. For this purpose, you need to use the web platform to:
- Create a trigger and name it HeexDemoEdgeTrigger.
- Create a HeexDemoEdgeMonitor Monitor related to a boolean signal.
- Create a HeexDemoEdgeRecorder Recorder. You may select the Context Values `"position"` and `"bag_timestamp"` if they are available. You may select the recording range you please.
- Export your system configuration as a systemConf.json file and copy it into the Kernel folder.


Finally, as your newly created HeexDemoEdgeMonitor and HeexDemoEdgeRecorder have different UUIDs than the one shipped in the code, you will need to modify their value in the `Ros2EdgeInMemoryBufferV2.cpp` by replacing:
- For the Monitor, edit it in `sampleRos2BooleanMonitor("M-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)", serverIp, monitorServerPort)`
- for the Recorder, edit it in `sampleRos2BagRecorder("R-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)", serverIp, recorderServerPort)`


### Building
#### SDK build
Before building this example ROS2 package for edge applications, make sure the Heex SDK has been built within your machine.

#### Rosbags
The demo rosbag can be retrieved using this [rosbag](https://heex-public-sample-datasets.s3.eu-west-1.amazonaws.com/example_offline_extraction_withinstant/example_offline_extraction_withinstant.db3)  

#### Example build for Ubuntu 22.04 (Humble Hawksbill) or Ubuntu 20.04 (Galactic GeoChelone)
This example has been implemented and tested with ROS2 Humble Hawskbill on Ubuntu 22.04 and with Galactic GeoChelone on Ubuntu 20.04.
- Are you using another ROS2 version ? We recommend the use of an appropriate Docker environment to build and try this example
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
  ros2 run ros2_edge_in_memory_buffer_v2_pkg ros2EdgeInMemoryBufferV2
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


The `SampleRos2BooleanMonitor` subscribes to the `/demo/bool` topic and performs the monitoring logic every callback. This topic contains only a boolean to mimic any binary dataflow a system might have. Each callback updates the internal states to determine if the dataflows when from `True` to `False`, such as when a Disengagement happens.

## Adaptation notes
The structure of this example shows a possibility to structure your ROS2 collection of Monitors and Recorders. We have purposely stored our Monitor and Recorder original code outside of the ROS2 `ros2_edge_in_memory_buffer_v2_pkg` package. This way, they can be single code repository and reused in other ROS2 packages using symlinks.

To adapt this example project for your dataflows and rosbag, you will need to configure the following:

### Monitor
- Your Monitor will probably need some specific custom states. They can be included in the custom monitor class as demonstrated in the `SampleRos2BooleanMonitor`.
  - If the signal monitored has a specific Quality Of Services (QoS) that are not compatible with the default one, you will need to specify the profile and replace the `topicQosProfile` given in the sample. 

### Recorder
- Modify the list of buffered topics `_topicDetails`, to include your selection of topics to record:
  - In file `SampleRos2BagRecorder.cpp`, function `SampleRos2BagRecorder::SampleRos2BagRecorder`, lines:
  ```cpp
    _topicDetails = {
        heex_rosbag2_snapshot::TopicDetails_t("/gps/fix", "sensor_msgs/msg/NavSatFix"),
        heex_rosbag2_snapshot::TopicDetails_t("/demo/bool", "std_msgs/Bool", specific_qos_profile),
        heex_rosbag2_snapshot::TopicDetails_t("/imu", "sensor_msgs/msg/Imu")};
  ```
  make sure you add the proper topic types associated to the topic names
  - If your use case uses signals that have specific Quality Of Services (QoS) that are not compatible with the default ones, you will need to specify the profile for each topic that needs a different QoS than the default `QoS{10}` (as shown on the second line above with the `specific_qos_profile`)
- If you prefer to register to **all available topics** instead, you can leave the `_topicDetails` empty, and the recorder shall then subscribe to any topic that comes from a publisher.
  - In this case, you don't need to worry about QoS profiles, as each QoS profile will be read from the published signal before applying it to the subscriptions.
- You won't need to bother about the size of the buffer that is set for the messages, as it is directly set based on the recorder's intervals
- A well sourced environment is required when custom messages are used.
- In both cases, make sure to update how Context Values like GNSS position are extracted from your topics list.
  - You can update the name of the topic that contains the GPS data in the `_positionTopicName` var

## Additional notes
- Our sample will generate recorded data into a folder that will have following structure:
  `/tmp/RosBags/rosbag_<query.eventTimestamp>_<query.eventUuid>/`
                      `metadata.yaml`
                      `recording_<query.eventTimestamp>_<query.eventUuid>.db3`
- This folder will then be zipped and uploaded onto the webplatform.
