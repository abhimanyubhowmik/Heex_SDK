cmake_minimum_required(VERSION 3.10.2) # Preferred cmake-3.16.3

# #############################################
# Path works for build. Mostly relative to Heex.

# Set relatively the Heex librairies and 3rd party dirs
if(NOT DEFINED HEEX_SDK_DIR)
  get_filename_component(HEEX_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." REALPATH)
  message(STATUS "Auto Setting Heex SDK path to HEEX_SDK_DIR=${HEEX_SDK_DIR}")
else()
  message(STATUS "Heex SDK path set to: ${HEEX_SDK_DIR}")
endif()

if(NOT DEFINED HEEX_LIBRARIES_DIR)
  set(HEEX_LIBRARIES_DIR ${HEEX_SDK_DIR}/libraries)
  message(STATUS "Auto Setting HEEX_LIBRARIES_DIR=${HEEX_LIBRARIES_DIR}")
endif()

if(NOT DEFINED HEEX_3RDPARTY_DIR)
  set(HEEX_3RDPARTY_DIR ${HEEX_SDK_DIR}/3rdparty)
  message(STATUS "Auto Setting HEEX_3RDPARTY_DIR=${HEEX_3RDPARTY_DIR}")
endif()

if(NOT DEFINED HEEX_CMAKE_DIR)
  set(HEEX_CMAKE_DIR ${HEEX_SDK_DIR}/cmake)
  message(STATUS "Auto Setting HEEX_CMAKE_DIR=${HEEX_CMAKE_DIR}")
endif()

# #############################################
# Set version and project info

file (STRINGS "${HEEX_SDK_DIR}/version" HEEX_BUILD_VERSION)
project("HeexConfig"
  VERSION ${HEEX_BUILD_VERSION}
  LANGUAGES CXX
  DESCRIPTION "C++ toolbox parses Heex-custom format configuration file to store Heex-specific system configurations."
)
message(STATUS "${CMAKE_PROJECT_NAME} version: ${HEEX_BUILD_VERSION}")

# Set output for library and execs to main project folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Debug/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Debug/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Debug/)

set(HEEX_BUILD_DIR "build/")
if(CMAKE_BUILD_TYPE MATCHES "Debug")
  set(HEEX_BUILD_DIR "debugBuild/")
endif()

# Configure the flags per supported compiler
set (CMAKE_C_COMPILER g++)

# #############################################
# This command generates FindHeexConfigDeps.cmake from a template file. The template file,
# FindHeexConfigDeps.cmake.in, contains placeholders of the form @VAR@ or ${VAR}.
# With @ONLY, only @VAR@ variables are replaced during the template generation
# HEEX_BUILD_VERSION and HEEX_BUILD_DIR are replaced
# When there are new dependencies to be included in the project, you should add them to
# the template file. This ensures that subsequent configurations will incorporate the new
# libraries and their find_package calls or necessary variables into the build process.
configure_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/FindHeexConfigDeps.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/FindHeexConfigDeps.cmake
  @ONLY
)
include(${CMAKE_CURRENT_BINARY_DIR}/FindHeexConfigDeps.cmake)
include(${HEEX_CMAKE_DIR}/HeexCMake.cmake)
##############################################
# Create target and set properties

# Create HeexConfig static library
add_library(HeexConfig STATIC HeexConfig.cpp)

# Set target properties
target_include_directories(HeexConfig
  PUBLIC
    $<INSTALL_INTERFACE:>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(HeexConfig
  PUBLIC
    HeexUtils::HeexUtils
)

set(HEEX_CONFIG_COMPILE_OPTIONS
  # Add default compile flags for GNU gcc for All builds
  "$<$<CXX_COMPILER_ID:GNU>:-Wextra;-Wall;-W;-pedantic;-std=c++14;>"
  # Add default compile flags for GNU gcc for Release build
  "$<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:RELEASE>>:-O3;>"
  # Add default compile flags for GNU gcc for Debug build
  "$<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:DEBUG>>:-g3;-O0;>"
  # Disable warning C4127 causes by HEEX_LOG. if constexpr is not available in c++14
  "$<$<CXX_COMPILER_ID:MSVC>:/wd4127;>"
  # Set default compile flags for Windows MSVC for Release build
  "$<$<AND:$<CXX_COMPILER_ID:MSVC>,$<CONFIG:RELEASE>>:/MD;/O2;/Ob2;/DNDEBUG;/W4;>"
  # Set default compile flags for Windows MSVC for Debug build
  "$<$<AND:$<CXX_COMPILER_ID:MSVC>,$<CONFIG:DEBUG>>:/MDd;/Od;/Ob0;/Zi;/RTC1;/DDEBUG;/D_DEBUG;>"
)
target_compile_options(HeexConfig PRIVATE ${HEEX_CONFIG_COMPILE_OPTIONS})
set(HEEX_CONFIG_COMPILE_DEFINITIONS
  # Define macro DEBUG_BUILD for Debug build
  "$<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:DEBUG>>:DEBUG_BUILD;>"
  "$<$<AND:$<CXX_COMPILER_ID:MSVC>,$<CONFIG:DEBUG>>:/DDEBUG_BUILD;>"
  # Set target platform to be win7+
  "$<$<CXX_COMPILER_ID:MSVC>:_WIN32_WINNT=0x0601;>"
  # Disable autolinking in boost (avoid linking date_time and regex)
  "$<$<CXX_COMPILER_ID:MSVC>:BOOST_ALL_NO_LIB;>"
  # Ask Boost to report information about automatic linking requests (Windows-only)
  "$<$<CXX_COMPILER_ID:MSVC>:Boost_LIB_DIAGNOSTIC_DEFINITIONS;>"
  # Set the SDK project version as preprocessor variable
  HEEX_BUILD_VERSION="${PROJECT_VERSION}"
)
target_compile_definitions(HeexConfig PRIVATE ${HEEX_CONFIG_COMPILE_DEFINITIONS})

## Compile with -fPIC flag if we want to use python bindings
if(CMAKE_COMPILER_IS_GNUCXX AND HEEX_BUILD_USE_PYTHON)
  message(STATUS "Compiling with -fPIC flag")
  set_target_properties(HeexConfig PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif(CMAKE_COMPILER_IS_GNUCXX AND HEEX_BUILD_USE_PYTHON)

## Add to HeexConfig the alias HeexConfig::HeexConfig so that library can be used inside the build tree
add_library(HeexConfig::HeexConfig ALIAS HeexConfig)

##############################################
# Installation instructions

# Export targets
include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/HeexConfig)

install(TARGETS HeexConfig
  EXPORT HeexConfig-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Export the targets to a script
install(EXPORT HeexConfig-targets
  FILE HeexConfigTargets.cmake
  NAMESPACE HeexConfig::
  DESTINATION ${INSTALL_CONFIGDIR}
)

# Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/HeexConfigConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/HeexConfigConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/HeexConfigConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

##############################################
# Exporting from the build tree

export(EXPORT HeexConfig-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/HeexConfigTargets.cmake
  NAMESPACE HeexConfig::
)

# Register package in the User Package Registry
export(PACKAGE HeexConfig)

##############################################
# Add tests

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Tests")
  include(CTest)
  add_subdirectory( Tests )
endif()
