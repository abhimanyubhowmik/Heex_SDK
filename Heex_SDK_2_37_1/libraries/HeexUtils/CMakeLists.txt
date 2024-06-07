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

set(WHEREAMI_DIR ${HEEX_3RDPARTY_DIR}/whereami/src)

# #############################################
# Set version and project info

file (STRINGS "${HEEX_SDK_DIR}/version" HEEX_BUILD_VERSION)
project("HeexUtils"
  VERSION ${HEEX_BUILD_VERSION}
  LANGUAGES CXX
  DESCRIPTION "C++ toolbox to assist operations within the Heex Embedded environment for Smart Data"
)
message(STATUS "${CMAKE_PROJECT_NAME} version: ${HEEX_BUILD_VERSION}")

# Set output for library and execs to main project folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Debug/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Debug/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Debug/)

# #############################################
# Find Heex library dependencies.
# FindHeexUtilsDeps.cmake.in, contains placeholders of the form @VAR@ or ${VAR}.
# With @ONLY, only @VAR@ variables are replaced during the template generation
# HEEX_BUILD_VERSION and HEEX_BUILD_DIR are replaced
# When there are new dependencies to be included in the project, you should add them to
# the template file. This ensures that subsequent configurations will incorporate the new
# libraries and their find_package calls or necessary variables into the build process.
configure_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/FindHeexUtilsDeps.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/FindHeexUtilsDeps.cmake
  @ONLY
)
include(${CMAKE_CURRENT_BINARY_DIR}/FindHeexUtilsDeps.cmake)
include(${HEEX_CMAKE_DIR}/HeexCMake.cmake)
##############################################
# Create target and set properties

# Create HeexUtils static library
add_library(HeexUtils STATIC
  HeexUtils.cpp
  HeexUtilsGnss.cpp
  HeexUtilsFileOperations.cpp
  HeexUtilsFileLock.cpp
  HeexUtilsLog.cpp
  HeexUtilsExeRunner.cpp
  HeexUtilsVersion.cpp
  HeexUtilsSystemInformation.cpp
  ${WHEREAMI_DIR}/whereami.cpp
)

# Set target properties
target_include_directories(HeexUtils
  PUBLIC
    $<INSTALL_INTERFACE:include/HeexUtils>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_include_directories(HeexUtils SYSTEM
  PUBLIC
    ${Boost_INCLUDE_DIRS}
  PRIVATE
    ${WHEREAMI_DIR}
)
target_link_libraries(HeexUtils
  PUBLIC
    ${Boost_LIBRARIES} # See ${HEEX_UTILS_BOOST_DEPS} allocation and final value for more details
)
set(HEEX_UTILS_COMPILE_OPTIONS
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
target_compile_options(HeexUtils PRIVATE ${HEEX_UTILS_COMPILE_OPTIONS})
set(HEEX_UTILS_COMPILE_DEFINITIONS
  # Define macro DEBUG_BUILD for Debug build
  "$<$<CONFIG:DEBUG>:DEBUG_BUILD;>"
  # Set target platform to be win7+
  "$<$<CXX_COMPILER_ID:MSVC>:_WIN32_WINNT=0x0601;>"
  # Disable autolinking in boost (avoid linking date_time and regex)
  "$<$<CXX_COMPILER_ID:MSVC>:BOOST_ALL_NO_LIB;>"
  # Ask Boost to report information about automatic linking requests (Windows-only)
  "$<$<CXX_COMPILER_ID:MSVC>:Boost_LIB_DIAGNOSTIC_DEFINITIONS;>"
  # Set the SDK project version as preprocessor variable
  HEEX_BUILD_VERSION="${PROJECT_VERSION}"
)
target_compile_definitions(HeexUtils PRIVATE ${HEEX_UTILS_COMPILE_DEFINITIONS})

## Compile with -fPIC flag if we want to use python bindings
if(CMAKE_COMPILER_IS_GNUCXX AND HEEX_BUILD_USE_PYTHON)
  message(STATUS "Compiling with -fPIC flag")
  set_target_properties(HeexUtils PROPERTIES POSITION_INDEPENDENT_CODE ON) # -fPIC flag
endif(CMAKE_COMPILER_IS_GNUCXX AND HEEX_BUILD_USE_PYTHON)

## Compile with maximum log level.
# set(HEEX_MAX_LOG_LEVEL trace)
if(HEEX_MAX_LOG_LEVEL)
  message(STATUS "Compiling with Log level >= ${HEEX_MAX_LOG_LEVEL}")
  target_compile_definitions(HeexUtils PUBLIC HEEX_MAX_LOG_LEVEL=${HEEX_MAX_LOG_LEVEL})
endif(HEEX_MAX_LOG_LEVEL)

## Compile with os and architecture info
message(STATUS "Setting system OS and architecture to ${HEEX_BUILD_OS_NAME} ${HEEX_BUILD_ARCHITECTURE_NAME}")
target_compile_definitions(HeexUtils PRIVATE HEEX_BUILD_OS_NAME="${HEEX_BUILD_OS_NAME}")
target_compile_definitions(HeexUtils PRIVATE HEEX_BUILD_ARCHITECTURE_NAME="${HEEX_BUILD_ARCHITECTURE_NAME}")

## Add to HeexUtils the alias HeexUtils::HeexUtils so that library can be used inside the build tree
add_library(HeexUtils::HeexUtils ALIAS HeexUtils)

##############################################
# Installation instructions

# Export targets
include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/HeexUtils)

install(TARGETS HeexUtils
  EXPORT HeexUtils-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Export the targets to a script
install(EXPORT HeexUtils-targets
  FILE HeexUtilsTargets.cmake
  NAMESPACE HeexUtils::
  DESTINATION ${INSTALL_CONFIGDIR}
)

# Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/HeexUtilsConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/HeexUtilsConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/HeexUtilsConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

##############################################
# Exporting from the build tree

export(EXPORT HeexUtils-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/HeexUtilsTargets.cmake
  NAMESPACE HeexUtils::
)

# Register package in the User Package Registry
export(PACKAGE HeexUtils)

##############################################
# Add tests

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Tests")
  include(CTest)
  add_subdirectory( Tests )
endif()
