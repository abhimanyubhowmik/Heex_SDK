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
CMAKE_SOURCE_DIR = /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages/build

# Include any dependencies generated for this target.
include CMakeFiles/HeexMessages.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/HeexMessages.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/HeexMessages.dir/flags.make

CMakeFiles/HeexMessages.dir/Common/Tools.cpp.o: CMakeFiles/HeexMessages.dir/flags.make
CMakeFiles/HeexMessages.dir/Common/Tools.cpp.o: ../Common/Tools.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/HeexMessages.dir/Common/Tools.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/HeexMessages.dir/Common/Tools.cpp.o -c /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages/Common/Tools.cpp

CMakeFiles/HeexMessages.dir/Common/Tools.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HeexMessages.dir/Common/Tools.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages/Common/Tools.cpp > CMakeFiles/HeexMessages.dir/Common/Tools.cpp.i

CMakeFiles/HeexMessages.dir/Common/Tools.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HeexMessages.dir/Common/Tools.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages/Common/Tools.cpp -o CMakeFiles/HeexMessages.dir/Common/Tools.cpp.s

CMakeFiles/HeexMessages.dir/RecorderMessages/RecorderTools.cpp.o: CMakeFiles/HeexMessages.dir/flags.make
CMakeFiles/HeexMessages.dir/RecorderMessages/RecorderTools.cpp.o: ../RecorderMessages/RecorderTools.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/HeexMessages.dir/RecorderMessages/RecorderTools.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/HeexMessages.dir/RecorderMessages/RecorderTools.cpp.o -c /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages/RecorderMessages/RecorderTools.cpp

CMakeFiles/HeexMessages.dir/RecorderMessages/RecorderTools.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HeexMessages.dir/RecorderMessages/RecorderTools.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages/RecorderMessages/RecorderTools.cpp > CMakeFiles/HeexMessages.dir/RecorderMessages/RecorderTools.cpp.i

CMakeFiles/HeexMessages.dir/RecorderMessages/RecorderTools.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HeexMessages.dir/RecorderMessages/RecorderTools.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages/RecorderMessages/RecorderTools.cpp -o CMakeFiles/HeexMessages.dir/RecorderMessages/RecorderTools.cpp.s

# Object files for target HeexMessages
HeexMessages_OBJECTS = \
"CMakeFiles/HeexMessages.dir/Common/Tools.cpp.o" \
"CMakeFiles/HeexMessages.dir/RecorderMessages/RecorderTools.cpp.o"

# External object files for target HeexMessages
HeexMessages_EXTERNAL_OBJECTS =

../lib/libHeexMessages.a: CMakeFiles/HeexMessages.dir/Common/Tools.cpp.o
../lib/libHeexMessages.a: CMakeFiles/HeexMessages.dir/RecorderMessages/RecorderTools.cpp.o
../lib/libHeexMessages.a: CMakeFiles/HeexMessages.dir/build.make
../lib/libHeexMessages.a: CMakeFiles/HeexMessages.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library ../lib/libHeexMessages.a"
	$(CMAKE_COMMAND) -P CMakeFiles/HeexMessages.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/HeexMessages.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/HeexMessages.dir/build: ../lib/libHeexMessages.a

.PHONY : CMakeFiles/HeexMessages.dir/build

CMakeFiles/HeexMessages.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/HeexMessages.dir/cmake_clean.cmake
.PHONY : CMakeFiles/HeexMessages.dir/clean

CMakeFiles/HeexMessages.dir/depend:
	cd /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages/build /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages/build /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexMessages/build/CMakeFiles/HeexMessages.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/HeexMessages.dir/depend
