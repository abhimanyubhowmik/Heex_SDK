# #############################################
# Find Heex library dependencies.
# This will replace the following variables with their value:
#   HEEX_BUILD_VERSION
# This will define the following variables:
#   Working variables
#     HEEX_BOOST_PYTHON_VERSION
#     Boost_USE_STATIC_LIBS, Boost_USE_MULTITHREADED, Boost_USE_STATIC_RUNTIME
#     HEEX_BOOST_VERSION
#     HEEX_BOOST_FOLDER_NAME
#     HEEX_SDK_BOOST_DEPS
#     BOOST_ROOT, BOOST_INCLUDE_DIRS, BOOST_LIBRARY_DIRS
#
#   Package related variables
#     HeexUtils_DIR, HeexUtils_FOUND
#     HeexCom_DIR, HeexCom_FOUND
#     HeexConfig_DIR, HeexConfig_FOUND
#     HeexMessages_DIR, HeexMessages_FOUND

# Import Heex librairies

message(STATUS "Looking for HeexUtils version " 2.37.1)
find_package(HeexUtils 2.37.1 REQUIRED PATHS ${HEEX_LIBRARIES_DIR}/HeexUtils/build/ NO_DEFAULT_PATH)
message(STATUS "HeexUtils_DIR=${HeexUtils_DIR}")

message(STATUS "Looking for HeexCom version " 2.37.1)
find_package(HeexCom 2.37.1 REQUIRED PATHS ${HEEX_LIBRARIES_DIR}/HeexCom/build/ NO_DEFAULT_PATH)
message(STATUS "HeexCom_DIR=${HeexCom_DIR}")

message(STATUS "Looking for HeexConfig version " 2.37.1)
find_package(HeexConfig 2.37.1 REQUIRED PATHS ${HEEX_LIBRARIES_DIR}/HeexConfig/build/ NO_DEFAULT_PATH)
message(STATUS "HeexConfig_DIR=${HeexConfig_DIR}")

message(STATUS "Looking for HeexMessages_DIR version " 2.37.1)
find_package(HeexMessages 2.37.1 REQUIRED PATHS ${HEEX_LIBRARIES_DIR}/HeexMessages/build/ NO_DEFAULT_PATH)
message(STATUS "HeexMessages_DIR=${HeexMessages_DIR}")

# Look for Python dependencies if the flag HEEX_BUILD_USE_PYTHON is set. Set Boost dependencies for Python using find_package().
if(HEEX_BUILD_USE_PYTHON)
  message(STATUS "Configuring Wrapper of the Heex SDK for Python is ON")
  find_package (Python3 COMPONENTS Interpreter Development REQUIRED)
  find_package(PythonLibs REQUIRED)
  message(STATUS "Python ${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR} headers : ${PYTHON_INCLUDE_DIRS}")
  message(STATUS "Python ${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR} libraries : ${PYTHON_LIBRARIES}")
  set(HEEX_BOOST_PYTHON_VERSION python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR})
endif(HEEX_BUILD_USE_PYTHON)

# Find Boost
## Prepare Boost flags for header and built libraries linking
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

## Prepare Boost folder using flag specified to build the Heex SDK.
### HEEX_BOOST_VERSION can be override using -DHEEX_BOOST_VERSION:STRING=1.75.0
if(NOT DEFINED HEEX_BOOST_VERSION)
  set(HEEX_BOOST_VERSION 1.75.0)
  if(legacy1804HeexSdk)
      set(HEEX_BOOST_VERSION 1.65.1)
  endif()
endif()
message(STATUS "HEEX_BOOST_VERSION=${HEEX_BOOST_VERSION}")
### Auto-detect boost folder and set HEEX_BUILD_USE_PYTHON if we are using Python build
if(NOT DEFINED HEEX_BUILD_USE_PYTHON)
  string(REPLACE "." "_"  HEEX_BOOST_FOLDER_NAME_TEMP ${HEEX_BOOST_VERSION})
  if(EXISTS "${HEEX_3RDPARTY_DIR}/boost_${HEEX_BOOST_FOLDER_NAME_TEMP}_p")
      set(HEEX_BUILD_USE_PYTHON ON)
  else()
      set(HEEX_BUILD_USE_PYTHON OFF)
  endif()
endif()
if(NOT DEFINED HEEX_BOOST_FOLDER_NAME)
  string(REPLACE "." "_"  HEEX_BOOST_FOLDER_NAME ${HEEX_BOOST_VERSION})
  if(HEEX_BUILD_USE_PYTHON AND HEEX_BOOST_VERSION VERSION_EQUAL 1.75.0)
      set(HEEX_BOOST_FOLDER_NAME boost_${HEEX_BOOST_FOLDER_NAME}_p) # Using Boost with Python
  else()
      set(HEEX_BOOST_FOLDER_NAME boost_${HEEX_BOOST_FOLDER_NAME}) # Using Boost without Python
  endif()
endif()
message(STATUS "HEEX_BOOST_FOLDER_NAME=${HEEX_BOOST_FOLDER_NAME}")
message(STATUS "HEEX_BUILD_USE_PYTHON=${HEEX_BUILD_USE_PYTHON}")

## List Boost lib dependencies before using find package
set(HEEX_SDK_BOOST_DEPS program_options thread filesystem ${HEEX_BOOST_PYTHON_VERSION})

## Add extra dependencies depending on builds
if (legacy1804HeexSdk)
  message(STATUS "Using Boost ${HEEX_BOOST_VERSION} libs for Ubuntu/Linux 18.04 (legacy1804HeexSdk)")
  set(HEEX_SDK_BOOST_DEPS ${HEEX_SDK_BOOST_DEPS} date_time)
endif(legacy1804HeexSdk)

## Find Boost packages in 3rd party paths if it exists, else we look for them in system paths.
if(EXISTS ${HEEX_3RDPARTY_DIR}/${HEEX_BOOST_FOLDER_NAME}/)
  message(STATUS "Using Boost ${HEEX_BOOST_VERSION} libs from 3rd party for Ubuntu/Linux configurations")
  set(Boost_NO_SYSTEM_PATHS TRUE)
  if (Boost_NO_SYSTEM_PATHS)
      set(BOOST_ROOT ${HEEX_3RDPARTY_DIR}/${HEEX_BOOST_FOLDER_NAME}/)      # Hint on Boost project root directory in HEEX_3RDPARTY_DIR
      set(BOOST_INCLUDE_DIRS ${BOOST_ROOT}/boost/)            # Hint on Boost include root directory
      set(BOOST_LIBRARY_DIRS ${BOOST_ROOT}/stage/lib/)        # Hint on Boost lib root directory
  endif (Boost_NO_SYSTEM_PATHS)

  find_package(Boost ${HEEX_BOOST_VERSION} COMPONENTS ${HEEX_SDK_BOOST_DEPS} REQUIRED)
else()
  message(WARNING "Can't find folder ${HEEX_3RDPARTY_DIR}/${HEEX_BOOST_FOLDER_NAME}/ . Have you set HEEX_BOOST_VERSION ?")
  message(STATUS "Using Boost ${HEEX_BOOST_VERSION} libs from system for Ubuntu/Linux configurations as fallback")
  find_package(Boost ${HEEX_BOOST_VERSION} COMPONENTS ${HEEX_SDK_BOOST_DEPS} REQUIRED)

endif()

## Setup to use Boost header files and built librairies
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
message(STATUS "Boost include dir: ${Boost_INCLUDE_DIR}")
message(STATUS "Boost library dir: ${Boost_LIBRARIES}")

# find units
if (NOT DEFINED HEEX_3RDPARTY_UNITS)
  set(HEEX_3RDPARTY_UNITS "${HEEX_3RDPARTY_DIR}/units/build/install")
  if (CMAKE_BUILD_TYPE MATCHES "Debug" AND WIN32 AND CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
      set(HEEX_3RDPARTY_UNITS "${HEEX_3RDPARTY_DIR}/units/build/install/debug")
  endif()
endif()
find_package(units PATHS ${HEEX_3RDPARTY_UNITS} COMPONENTS units REQUIRED NO_DEFAULT_PATH)
if (DEFINED UNITS_LIBRARIES)
  message(STATUS "found units UNITS_LIBRARIES=${UNITS_LIBRARIES}")
endif()
