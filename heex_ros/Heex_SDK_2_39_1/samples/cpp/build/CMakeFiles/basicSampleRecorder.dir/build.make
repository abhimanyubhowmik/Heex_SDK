# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/abhimanyu/Heex_SDK_2_39_1/samples/cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/abhimanyu/Heex_SDK_2_39_1/samples/cpp/build

# Include any dependencies generated for this target.
include CMakeFiles/basicSampleRecorder.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/basicSampleRecorder.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/basicSampleRecorder.dir/flags.make

CMakeFiles/basicSampleRecorder.dir/basicSampleRecorder.cpp.o: CMakeFiles/basicSampleRecorder.dir/flags.make
CMakeFiles/basicSampleRecorder.dir/basicSampleRecorder.cpp.o: ../basicSampleRecorder.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abhimanyu/Heex_SDK_2_39_1/samples/cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/basicSampleRecorder.dir/basicSampleRecorder.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/basicSampleRecorder.dir/basicSampleRecorder.cpp.o -c /home/abhimanyu/Heex_SDK_2_39_1/samples/cpp/basicSampleRecorder.cpp

CMakeFiles/basicSampleRecorder.dir/basicSampleRecorder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/basicSampleRecorder.dir/basicSampleRecorder.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/abhimanyu/Heex_SDK_2_39_1/samples/cpp/basicSampleRecorder.cpp > CMakeFiles/basicSampleRecorder.dir/basicSampleRecorder.cpp.i

CMakeFiles/basicSampleRecorder.dir/basicSampleRecorder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/basicSampleRecorder.dir/basicSampleRecorder.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/abhimanyu/Heex_SDK_2_39_1/samples/cpp/basicSampleRecorder.cpp -o CMakeFiles/basicSampleRecorder.dir/basicSampleRecorder.cpp.s

# Object files for target basicSampleRecorder
basicSampleRecorder_OBJECTS = \
"CMakeFiles/basicSampleRecorder.dir/basicSampleRecorder.cpp.o"

# External object files for target basicSampleRecorder
basicSampleRecorder_EXTERNAL_OBJECTS =

../basicSampleRecorder: CMakeFiles/basicSampleRecorder.dir/basicSampleRecorder.cpp.o
../basicSampleRecorder: CMakeFiles/basicSampleRecorder.dir/build.make
../basicSampleRecorder: /home/abhimanyu/Heex_SDK_2_39_1/sdk/lib/libHeexCustomerSide.a
../basicSampleRecorder: /home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_filesystem.a
../basicSampleRecorder: /home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_program_options.a
../basicSampleRecorder: /home/abhimanyu/Heex_SDK_2_39_1/3rdparty/units/build/install/lib/libunits.a
../basicSampleRecorder: /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages/lib/libHeexMessages.a
../basicSampleRecorder: /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/libTcpClient.a
../basicSampleRecorder: /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexConfig/libHeexConfig.a
../basicSampleRecorder: /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexUtils/libHeexUtils.a
../basicSampleRecorder: /home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_log_setup.a
../basicSampleRecorder: /home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_log.a
../basicSampleRecorder: /home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_filesystem.a
../basicSampleRecorder: /home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_thread.a
../basicSampleRecorder: /home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_date_time.a
../basicSampleRecorder: /home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_regex.a
../basicSampleRecorder: /home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_chrono.a
../basicSampleRecorder: /home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_atomic.a
../basicSampleRecorder: CMakeFiles/basicSampleRecorder.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/abhimanyu/Heex_SDK_2_39_1/samples/cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../basicSampleRecorder"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/basicSampleRecorder.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/basicSampleRecorder.dir/build: ../basicSampleRecorder

.PHONY : CMakeFiles/basicSampleRecorder.dir/build

CMakeFiles/basicSampleRecorder.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/basicSampleRecorder.dir/cmake_clean.cmake
.PHONY : CMakeFiles/basicSampleRecorder.dir/clean

CMakeFiles/basicSampleRecorder.dir/depend:
	cd /home/abhimanyu/Heex_SDK_2_39_1/samples/cpp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/abhimanyu/Heex_SDK_2_39_1/samples/cpp /home/abhimanyu/Heex_SDK_2_39_1/samples/cpp /home/abhimanyu/Heex_SDK_2_39_1/samples/cpp/build /home/abhimanyu/Heex_SDK_2_39_1/samples/cpp/build /home/abhimanyu/Heex_SDK_2_39_1/samples/cpp/build/CMakeFiles/basicSampleRecorder.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/basicSampleRecorder.dir/depend

