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
CMAKE_SOURCE_DIR = /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/build

# Include any dependencies generated for this target.
include CMakeFiles/TcpServer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/TcpServer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/TcpServer.dir/flags.make

CMakeFiles/TcpServer.dir/TcpHandler.cpp.o: CMakeFiles/TcpServer.dir/flags.make
CMakeFiles/TcpServer.dir/TcpHandler.cpp.o: ../TcpHandler.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/TcpServer.dir/TcpHandler.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TcpServer.dir/TcpHandler.cpp.o -c /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/TcpHandler.cpp

CMakeFiles/TcpServer.dir/TcpHandler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TcpServer.dir/TcpHandler.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/TcpHandler.cpp > CMakeFiles/TcpServer.dir/TcpHandler.cpp.i

CMakeFiles/TcpServer.dir/TcpHandler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TcpServer.dir/TcpHandler.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/TcpHandler.cpp -o CMakeFiles/TcpServer.dir/TcpHandler.cpp.s

CMakeFiles/TcpServer.dir/TcpSession.cpp.o: CMakeFiles/TcpServer.dir/flags.make
CMakeFiles/TcpServer.dir/TcpSession.cpp.o: ../TcpSession.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/TcpServer.dir/TcpSession.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TcpServer.dir/TcpSession.cpp.o -c /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/TcpSession.cpp

CMakeFiles/TcpServer.dir/TcpSession.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TcpServer.dir/TcpSession.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/TcpSession.cpp > CMakeFiles/TcpServer.dir/TcpSession.cpp.i

CMakeFiles/TcpServer.dir/TcpSession.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TcpServer.dir/TcpSession.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/TcpSession.cpp -o CMakeFiles/TcpServer.dir/TcpSession.cpp.s

CMakeFiles/TcpServer.dir/TcpServer.cpp.o: CMakeFiles/TcpServer.dir/flags.make
CMakeFiles/TcpServer.dir/TcpServer.cpp.o: ../TcpServer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/TcpServer.dir/TcpServer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TcpServer.dir/TcpServer.cpp.o -c /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/TcpServer.cpp

CMakeFiles/TcpServer.dir/TcpServer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TcpServer.dir/TcpServer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/TcpServer.cpp > CMakeFiles/TcpServer.dir/TcpServer.cpp.i

CMakeFiles/TcpServer.dir/TcpServer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TcpServer.dir/TcpServer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/TcpServer.cpp -o CMakeFiles/TcpServer.dir/TcpServer.cpp.s

# Object files for target TcpServer
TcpServer_OBJECTS = \
"CMakeFiles/TcpServer.dir/TcpHandler.cpp.o" \
"CMakeFiles/TcpServer.dir/TcpSession.cpp.o" \
"CMakeFiles/TcpServer.dir/TcpServer.cpp.o"

# External object files for target TcpServer
TcpServer_EXTERNAL_OBJECTS =

../libTcpServer.a: CMakeFiles/TcpServer.dir/TcpHandler.cpp.o
../libTcpServer.a: CMakeFiles/TcpServer.dir/TcpSession.cpp.o
../libTcpServer.a: CMakeFiles/TcpServer.dir/TcpServer.cpp.o
../libTcpServer.a: CMakeFiles/TcpServer.dir/build.make
../libTcpServer.a: CMakeFiles/TcpServer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library ../libTcpServer.a"
	$(CMAKE_COMMAND) -P CMakeFiles/TcpServer.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TcpServer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/TcpServer.dir/build: ../libTcpServer.a

.PHONY : CMakeFiles/TcpServer.dir/build

CMakeFiles/TcpServer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/TcpServer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/TcpServer.dir/clean

CMakeFiles/TcpServer.dir/depend:
	cd /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/build /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/build /home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/build/CMakeFiles/TcpServer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/TcpServer.dir/depend

