# Heex SDK Example Ros Datalake
This project is the example of the recommended SDK usage for ROS that implements Monitors and Recorders for a datalake rosbag extraction (offline) use case.  
This use case for offline extraction on rosbag uses the rosbag as a database (collection of timestamped messages) and use the Rosbag C++ API to access the messages without replaying it. Hence, no roscore command is necessary to run the extraction. However, a well sourced environement is necessary when custom messages are used.

## Project content

- `CMakeLists.txt`: Contains all the import requirements for the HeexSDK and the Rosbag C++ API
- `MonitorADTransitionDisengagement`: Contains header and source for Monitor interface that monitors the ADTransitionDisengagement condition.
- `RecorderRosbagDataLakeExtractor`: Contains header and source for Recorder interface that exposes and performs the extraction of the ContextValue `position` and generation of the Event Recording Part as a smaller rosbag from a source rosbag.
- `main.cpp`: Main process for offline extraction that instanciates the Monitors and Recorders and iterate on all of the bag messages.
  - The Monitor(s) logic of detection is implemented within the iteration over the messages contains in the rosbag. Custom states can be either included in the custom Monitor class or in the loop as demonstrated here. 
  - The Recorder(s) logic is implemented and performed within its own class. In this offline extraction use case, we do not buffer the messages by timestamps as in the `ros-edge` online implementation.

## Usage
### Deployment
This example requires the SDE to have a trigger dedicated to operate the demo. For this purpose, you need to use the web platform to:
- Create a trigger and name it HeexDemoDatalakeTrigger.
- Create a HeexDemoDatalakeMonitor Monitor (MonitorADTransitionDisengagement) related to a boolean signal.
- Create a HeexDemoDatalakeRecorder Recorder (RecorderRosbagDataLakeExtractor). You may select the context value "position" and select a recording range of your choosing.
- Export your configuration as a systemConf.json file and copy it into the SDE folder.

### Running
You can run the example as follows:
- Term 1: Run the Heex kernel
  ```bash
  cd ~/Heex_SDK_X_XX_X/
  ./runHeexKernel.sh
  ```
- Term 2: Run the heexExampleRosDataLakeExtraction
  - Replace with your monitor and recorder UUIDs
  ```bash
  cd ~/Heex_SDK_X_XX_X/samples/ROS1/ros-datalake/build/
  ./heexExampleRosDataLakeExtraction --monitor_uuid M-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX --recorder_uuid R-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX -i ~/example_offline_extraction.bag
  ```

## Implementation notes

Be sure to extract all relevant messages in the time interval around the timestamp specified in the query. Some extraction may also require the extraction of configuration data published outside of this time range like "tf_static" messages or calibration data.  

The logic of detection is based on the fact that all messages are stored and read sorted by timestamp. Please ensure that is the case for any adaptation on your bag.
