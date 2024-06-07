cmake_minimum_required(VERSION 3.10.2) # Preferred cmake-3.16.3

# #############################################
# Path works for build. Mostly relative to Heex.

# Set relatively the Heex librairies and 3rd party dirs
if(NOT DEFINED HEEX_SDK_DIR)
  get_filename_component(HEEX_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." REALPATH)
  message(STATUS "Auto Setting Heex SDK path to HEEX_SDK_DIR=${HEEX_SDK_DIR}")
else()
  # Ensure that the provided path of the Heex SDK is coherent with the current package path. Reveal find package silent errors due to guesses.
  # We make sure to resolve any symbolic link that may failed the check
  get_filename_component(HEEX_SDK_DIR "${HEEX_SDK_DIR}" REALPATH)
  get_filename_component(HEEX_SDK_DIR_CURRENT "${CMAKE_CURRENT_SOURCE_DIR}/.." REALPATH)
  if(NOT HEEX_SDK_DIR STREQUAL HEEX_SDK_DIR_CURRENT)
    message(FATAL_ERROR "Heex SDK path doesn't match: Provided ${HEEX_SDK_DIR} , Current: ${HEEX_SDK_DIR_CURRENT} . This error is encountered if you have multiple Heex SDK installed. Please define HEEX_SDK_DIR and then import using find_package(HeexCustomerSide ${HEEX_BUILD_VERSION} REQUIRED PATHS ${HEEX_SDK_DIR}/sdk/build NO_DEFAULT_PATH)")
  else()
    message(STATUS "Heex SDK path set to: ${HEEX_SDK_DIR}")
  endif()
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
project("HeexEmbeddedSDK"
  VERSION ${HEEX_BUILD_VERSION}
  LANGUAGES CXX
  DESCRIPTION "SDK to deploy Smart Data for Autonomous Systems"
)
message(STATUS "${CMAKE_PROJECT_NAME} version: ${HEEX_BUILD_VERSION}")

# Set output for library and execs to main project folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/lib/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/lib/)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Debug/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/lib/Debug/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/lib/Debug/)

set(HEEX_BUILD_DIR "build/")
if(CMAKE_BUILD_TYPE MATCHES "Debug")
  set(HEEX_BUILD_DIR "debugBuild/")
endif()

# #############################################
# Find Heex library dependencies.
# FindHeexCustomerSideDeps.cmake.in, contains placeholders of the form @VAR@ or ${VAR}.
# With @ONLY, only @VAR@ variables are replaced during the template generation
# HEEX_BUILD_VERSION and HEEX_BUILD_DIR are replaced
# When there are new dependencies to be included in the project, you should add them to
# the template file. This ensures that subsequent configurations will incorporate the new
# libraries and their find_package calls or necessary variables into the build process.
configure_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/FindHeexCustomerSideDeps.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/FindHeexCustomerSideDeps.cmake
  @ONLY
)
include(${CMAKE_CURRENT_BINARY_DIR}/FindHeexCustomerSideDeps.cmake)
include(${HEEX_CMAKE_DIR}/HeexCMake.cmake)
##############################################
# Create target and set properties

# Add all subprojects
add_subdirectory(CustomerSide)

# Build HeexCustomerSide library
add_library(HeexCustomerSide STATIC ${SOURCE_CUSTOMERSIDE} ${SOURCE_COMMON})

## Set target properties
target_include_directories(HeexCustomerSide
  PUBLIC
    $<INSTALL_INTERFACE:include/HeexCustomerSide>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/CustomerSide>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/CustomerSide
    # ${Boost_INCLUDE_DIR}
)
target_include_directories(HeexCustomerSide SYSTEM
  PUBLIC
  ${HEEX_3RDPARTY_UNITS}/include/units
)
target_link_libraries(HeexCustomerSide
  PUBLIC
    ${Boost_LIBRARIES} # Boost::program_options Boost::thread Boost::filesystem
    units::units
    HeexMessages::HeexMessages
    HeexCom::TcpClient
    HeexConfig::HeexConfig
    HeexUtils::HeexUtils
)

# Set SDK common target properties
## SDE Compile options
set(HEEX_SDK_COMPILE_OPTIONS
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
target_compile_options(HeexCustomerSide PRIVATE ${HEEX_SDK_COMPILE_OPTIONS})

## Make Boost uses /dev/random instead of getrandom syscall for Linux kernel version < 3.17 (Reason: not available resulting in runtime error)
if(UNIX)
  execute_process(COMMAND uname -r OUTPUT_VARIABLE UNAME_RESULT OUTPUT_STRIP_TRAILING_WHITESPACE)
  string(REGEX MATCH "[0-9]+.[0-9]+" LINUX_KERNEL_VERSION ${UNAME_RESULT})
  message(STATUS "-- Kernel version: ${UNAME_RESULT}")
endif(UNIX)

## SDE Compile definitions
set(HEEX_SDK_COMPILE_DEFINITIONS
  # Setting Boost flag BOOST_UUID_RANDOM_PROVIDER_FORCE_POSIX for appropriate UUID support of linux kernel below 3.17 on Centos 7.8
  "$<$<OR:$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:${LINUX_KERNEL_VERSION},3.17>>,$<BOOL:${HEEX_FORCE_BOOST_UUID_RANDOM}>>:BOOST_UUID_RANDOM_PROVIDER_FORCE_POSIX;>"
  # Define macro DEBUG_BUILD for Debug build
  "$<$<CONFIG:DEBUG>:DEBUG_BUILD;>"
  # Set target platform to be win7+
  "$<$<CXX_COMPILER_ID:MSVC>:_WIN32_WINNT=0x0601;>"
  # Disable autolinking for Boost::Container, Boost::data_time and Boost::regex. Prefer to use header-only.
  "$<$<CXX_COMPILER_ID:MSVC>:BOOST_CONTAINER_NO_LIB;BOOST_DATE_TIME_NO_LIB;BOOST_REGEX_NO_LIB;>"
  # Ask Boost to report information about automatic linking requests (Windows-only)
  "$<$<CXX_COMPILER_ID:MSVC>:Boost_LIB_DIAGNOSTIC_DEFINITIONS;>"
)
target_compile_definitions(HeexCustomerSide PRIVATE ${HEEX_SDK_COMPILE_DEFINITIONS})
## Set SDK project version as PUBLIC preprocessor variable
target_compile_definitions(HeexCustomerSide PUBLIC HEEX_BUILD_VERSION="${PROJECT_VERSION}")

## Compile with -fPIC flag if we want to use python bindings, Should not be done on windows as the -fPIC flag is not needed
if(CMAKE_COMPILER_IS_GNUCXX AND HEEX_BUILD_USE_PYTHON)
  message(STATUS "Compiling with -fPIC flag")
  set_target_properties(HeexCustomerSide PROPERTIES POSITION_INDEPENDENT_CODE ON) # -fPIC flag
endif(CMAKE_COMPILER_IS_GNUCXX AND HEEX_BUILD_USE_PYTHON)

## Add preprocessor flag to use alternative Boost json
if(HEEX_BOOST_VERSION VERSION_LESS 1.75.0)
  message(WARNING "Some SDK features won't be available when compiled with Boost ${HEEX_BOOST_VERSION} libs for Ubuntu/Linux. Use 1.75.0 to enable all features.")
  target_compile_definitions(HeexCustomerSide PUBLIC HEEX_SDK_NO_BOOST_JSON)
endif()

## Add to HeexCustomerSide the alias HeexCustomerSide::HeexCustomerSide so that library can be used inside the build tree
add_library(HeexCustomerSide::HeexCustomerSide ALIAS HeexCustomerSide)

# Create Python Bindings for the HeexCustomerSide that builds python wrapper for the Detectors and Recorders of the Heex SDK
if(HEEX_BUILD_USE_PYTHON)
  message(STATUS "Compiling Wrapper of the Heex SDK for Python is ON")
  add_subdirectory(CustomerSidePython)
endif(HEEX_BUILD_USE_PYTHON)

##############################################
# Installation instructions

# Export targets
include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/HeexCustomerSide)

install(TARGETS HeexCustomerSide
  EXPORT HeexCustomerSide-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Export the targets to a script
install(EXPORT HeexCustomerSide-targets
  FILE HeexCustomerSideTargets.cmake
  NAMESPACE HeexCustomerSide::
  DESTINATION ${INSTALL_CONFIGDIR}
)

# Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/HeexCustomerSideConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/HeexCustomerSideConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/HeexCustomerSideConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

##############################################
# Exporting from the build tree

export(EXPORT HeexCustomerSide-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/HeexCustomerSideTargets.cmake
  NAMESPACE HeexCustomerSide::
)

# Register package in the User Package Registry
export(PACKAGE HeexCustomerSide)

##############################################
# Add developper-only samples

# cpp Samples have been moved towards samples/cpp. The current folder is for archived samples, non released samples, or ones that need improving
if(BUILD_SAMPLES AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Samples") # we check it's existence since it's not a necessary folder, so one could delete it (ie pipelines), and build shouldn't fail.
  add_subdirectory(Samples)
endif()

##############################################
# Add tests

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Tests")
  include(CTest)
  add_subdirectory( Tests )
endif()
