# Changelog

All notable changes to the Heex SDK will be documented in this file.

## [Unreleased]
- Add the release command to the HRE CLI, allowing the upload of an HRE installer for a set of vehicles.

## 2.39.1 - May 2024
- Fix memory leak on ros 1 sample and update documentation accordingly

## 2.39.0 - May 2024
- Add heex installer that will be able to deploy heex environment
- Add support for a new trigger: persistent state trigger
- Add ROS2 Edge Sample using On disk (equivalent of ROS1)
- Add support to handle successive extraction in the ROS2 edge in-memory buffer sample
- Add the implementation of the generic sample of datalake extraction for ROS2
- Add a robustness test
- Add DBManager SObjectizer agent in the HeexKernelAgent
- Add a separate upload channel for metadata
- Change the PR template to add the linter check
- Change value of uploadNoProgressTimeout from 5 min to 30 sec
- Fix issue with curl when the SSL certificate revocation check fails with error code 35
- Fix concurrency error in some unittest

## 2.37.1 - May 2024
- Fix issue with Zone Monitor that was not handeling correctly new conditions (in, out, enter, exit)
- Fix some tooling issues (internal usage)

## 2.37.0 - April 2024
- Add DB sqlite3 to handle job processing instead of using a file
- Add a script to parse the clang-tidy report and display clear warning and error issues
- Add new images related to the new WP in public and private doc 
- Add support to merge public and private documentation
- Add a security check on downloaded HRE to make sure it is unaltered
- Add a recovery/failsafe for HRE OTA update 
- Add usage of a new API for HRE OTA to provide some data to WP after every release
- Add/create a pipeline to uploadHres automatically
- Change geo-fence conditions and handle new conditions (in, out, enter, exit) 
- Fix issue of uploading a file that doesn’t exist, set maximum retry number at 10
- Fix CLIs environment flags
- Fix issue with build sdk failing in Windows 11 due to long paths

## 2.35.0 - March 2024
- Add feature "Heex Agent Installer" (previously called HRE) OTA
- Add unit tests and integration tests for the "Heex Agent Installer" OTA
- Add code quality check using clang tidy
- Change build of sdk by default: set flag j to 1 by default and not take all resources.
- Change CLI: Retrieve company env from client environment and set the flag -e <env> by default, so it's not required to set it all the time.
- Change some debug messages into info messages in heex logs
- Fix issue with IntervalMonitor for floating point values. Remove hash from the systemConf and get value from the header now.
- Fix issue with sample ROS2 when list of topics is empty
- Fix issue with ROS2 samples when ros version is not detected correctly and searches for ROS1 package even though it is not ROS1 environment. 

## 2.33.1 - March 2024
- Fix issue with the ZoneMonitor to make it case insensitive.

## 2.33.0 - February 2024
- Add feature of sending HRE hash along with events to the cloud
- Add coding guidelines with linter
- Add check if the values of systemUuid and systemConfHash are empty and remove from query parameters in the API to send recordings to the cloud
- Add/update blueprints for the new samples
- Add warning to user if Linux system is out dated
- Change code to refactor the samples (python & c++) into basic and advanced samples
- Change integration tests to be complient with periodic monitor
- Change sdk-docs to reflect the samples code refactoring
- Change in sdk-docs disableSystemConfigurationOTAUpdate into enableSystemConfigurationOTAUpdate to avoid double negation and be aligned in the new added parameters
- Fix OTA integration having an extra data error when uploading jobs in json file
- Fix some issues with python samples lib import in HRE
- Fix Event uuid not found error in heexCore after receiving Recorder message in OTA integration test

## 2.31.0 - January 2024
- Fix issue of integration test failing when building in debug on Windows.
- Fix issue with HRE installer running in the background to properly uninstall all processes that have more than 1 pid and added some post install checks.git
- Fix ROS2 sample from using topic index to topic name for better supporting all topics
- Change default user from root to user for service when installed using the HRE
- Add QoS support to listed ROS2 topic
- Fix job respawn causing unnecessary logging errors but no data loss.

## 2.29.2 - January 2024
- Fix issue with HRE installer running in the background to properly uninstall all processes that have more than 1 pid

## 2.29.1 - January 2024
- Fix ROS2 sample from using topic index to topic name for better supporting all topics
- Change default user from root to user for service when installed using the HRE
- Add QoS support to listed ROS2 topic
- Fix job respawn causing unnecessary logging errors but no data loss.
## 2.29.0 - December 2023
- Change the default location for SystemConfs from kernel/ to kernel/SystemConfs folder. SystemConf in the previous location will be moved when loaded by heexCore
- Change the Apex.AI sample to be compatible with the newest Apex OS 3.x releases (previously 2.2.0)
- Change in documentation for smoother experience with Python agents, running on Windows, and update the list of SDK requirements and dependencies for customers.
- Add an optional parameter to the HRE installer to force the installation type: service (default) or background process.
## 2.27.2 - December 2023
- Fix high CPU usage of agents and kernel while waiting for connection to Kernel Core
## 2.27.1 - December 2023
- Fix agents built in debug throwing exception when connecting to Kernel
## 2.27.0 - November 2023
- Add a 40MB and 4 file number limits for the CLI logs
- Add telemetry of the current system pressure at event creation in the metadata (metadata, recording) and awaiting jobs to be completed
- Change some key functions which throws to return boolean instead when the control flow was affected
- Fix autocomplete for long arguments on Linux
- Fix computation and communication of the systemConf.json hash with upload APIs
## 2.25.2 - November 2023
- Fix HRE default and custom folder names. Add HRE package information into agent.json file
## 2.25.1 - November 2023
- Fix HRE handling of Python agents
## 2.25.0 - October 2023
- Add **heex** CLI as one tool to replace the multiple bootstrap and packageHRE scripts. Add extra features for cloud operations and autocomplete integration.
- Add Ubuntu 22.04 support
- Change initial ROS1 ros-edge package to ros_edge_in_memory_buffer_pkg
- Add ROS1 ros_edge_on_disk_buffer_pkg sample as an alternative approach to in-memory 
- Add ROS2 support (Humble for Ubuntu 22.04) with ROS2 sample ros2_edge_in_memory_buffer_pkg
- Fix Kernel not correctly linking libcurl library in static
- Fix missing function addGNSSContextValue for Recorders in Python
- Change Get Started experience to use the CLI
- Fix folder and file rights too permissive on Ubuntu SDK releases
## 2.23.1 - September 2023
- Fix file and folder permissions in SDK release folder
## 2.23.0 - September 2023
- Add a wait job optional parameter to runHeexKernel script to kill the kernel only after all smart data got collected and uploaded
- Change OTA process to a few independent steps and make it independent for each kernel executable. OTA is now performed whenever the systemConf.json file gets updated.
- Fix potential kernel lock when DataCollector fails to perform the OTA.
- Add a toggleable all_topics option in ROS edge sample
- Add an option to extract data from static topics (e.g. tf_static) and add it start of each recorded bag
- Add waiting time optimization when multiple events are created over a short period
## 2.21.2 - September 2023
- Fix units import of kernel from a system configuration file.
- Fix ros-edge build when HEEX_SDK_DIR is set with initial ~ or ending / character (and on all other cmakelists where issue was present)
- Fix unit to be loaded from systemConf
## 2.21.1 - August 2023
- Fix OTA sync race condition
## 2.21.0 - August 2023
- Several minor internal enhancements
## 2.19.0 - July 2023
- SDK Build handles corrupted dependencies downloads
- Monitor and Recorders can receive extra configurations from the Web using the **constant feature**
- Rework of the ros-edge sample
- Helper function for Geolocation Context Value
## 2.17.0 - June 2023
 - Add extra configurations per Agent
## 2.15.1 - Mai 2023
 - Fix HRE packaging on Ubuntu 18
## 2.15.0 - Mai 2023
 - Resistant data collection
 - Updated ROS Samples
## 2.13.1 - Mai 2023
 - fix HRE Python lib installation
 - fix installation in folder containing space characters
## 2.13.0 - April 2023
 - HRE (Heex Runtime Environment) generation
 - Windows build: fixed most warnings
## 2.11.1 - March 2023
- Fixed session expiration
## 2.11.0 - March 2023
- SDK Folder name simplification
- Agent version is now part of the UUID
- Agents & Kernel now also store logs into log files
## 2.9.0 - February 2023
- Python support on Windows
- generic base Recorder for recording in the past
## 2.7.1 - February 2023
- Fix SDK build
## 2.7.0 - Janauary 2023
- Trigger's default Tags are now exported in the systemConf.json
- Windows: python is now fully supported
## 2.5.3 - Janauary 2023
- Fix CentOS 7.8 build for Linux Kernel 3.10.0-514 
## 2.5.2 - Janauary 2023
- Fix multifile recordings
- Fix OTA Detectors V2 update
## 2.5.1 - December 2022
- Fix for CentOS 7
- From 2.3.2: fixes for typedef
- From 2.3.2: info log for ValueConfiguration sending
## 2.5.0 - December 2022
- portable 7zip is now shipped with the sdk (both linux and windows)
- default recordings folder moved from /tmp to ./outbox
- added runHeexKernel flavors 
## 2.3.1 - November 2022
- Various fixes
## 2.3.0 - November 2022
- Added new metadata-only upload for systems with limited connectivity
- Detectors and Recorders can now notify their implementation version
- Various fixes and performance improvements

## 2.2.0 - Not released
## 2.1.0 - October 2022
- Various fixes and performance improvements

## 2.0.0 - October 2022
- Release of the Detectors V2
- Support for ARM version of Ubuntu 18 and Ubuntu 20
- Enriched log messages

## 1.9.0 - August 2022
- Preliminary work for Detectors V2 (cont.)
- Handling of Over-The-Air reconfiguration
- Introduction of a built-in periodic detector
- logs: new upload progress bar 

## 1.8.0 - August 2022
- Preliminary work for Detectors V2

## 1.7.3 - August 2022
- Fix build on Ubuntu 18.04

## 1.7.2 - July 2022
- Fix Python samples not working properly.

## 1.7.0 - July 2022
- Add Bootstrap.sh script to build all Heex SDK.
- SDK ContextValue handling changed from string to Heex::RecorderArgs::ContextValue.

## 1.6.2 - July 2022
- Fix crash on recorder reconection to heexCore

## 1.6.0 - July 2022
- Improve sdk unit testing regarding HeexCom and context values
- SDK Support for adding extra ContextValues to an event even if no ContextValues have been set in the Trigger Definition
- Provide examples on importing SDK and demonstrating ROS use cases

## v1.5.0 - June 2022
- Display version on sde binaries and sdk samples
- Improve CMakeLists for modern and easier import

## v1.4.0 - June 2022
- Add granularity to event recording part deletion.
- heexDataCollector and heexDataUploader got an option --removeJobs to clean unwanted remaining jobs from jobs list.
- Add isPerformingCallback() to Recorder to be able to know whether or not the recorder is performing a job.

## v1.3.0 - April 2022
- Unify Detector and Recorder classes as Agents
- Add Configuration-by-value feature to Detector and Recorder Agents
- Add Python Wrapper and document its deployment
- Make Recorders support multiple Context Values
- Separate the DataSender into two entities: DataCollector (collect and prepare) and DataUploader (auth, send and remove)
Make SDE import the services configuration from the system configuration json file

## v1.2.0 - February 2022
- Split the SDK into sdk (CustomerSide) and sde (Core & DataSender) packages
- Upgrade thread management in Core, Detectors and Recorder and the interprocess TCP stack
- Fix binaries missing shared libraries on launch
- Add Samples and GetStarted elements for onboarding purposes
- Make SDE import the trigger configuration from the system configuration json file
- Add support for Windows with VS2019 MSVC v142 compiler

## v1.1.0 - November 2021
- Add buffer for detections to Core
- Add compatibility on event recording filepath with absolute and relative path
- Add encoding and decoding of illegal character in filepath and data values
- Add Deployment and Usage documentation and simple example.
- Add error management with the Heex upload APIS (routing services)

## v1.0.0 - June 2021
- Initial release.
