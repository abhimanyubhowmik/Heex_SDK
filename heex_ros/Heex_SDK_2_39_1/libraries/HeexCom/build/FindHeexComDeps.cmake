# #############################################
# Find HeexCom library dependencies.
# Import Heex librairies
message(STATUS "Looking for HeexUtils version " 2.39.1)
find_package(HeexUtils 2.39.1 REQUIRED PATHS ${HEEX_LIBRARIES_DIR}/HeexUtils/build/ NO_DEFAULT_PATH)
message(STATUS "HeexUtils_DIR=${HeexUtils_DIR}")

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

## List Boost lib dependencies before using find package
set(HEEX_COM_BOOST_DEPS thread)

## Find Boost packages in 3rd party paths if it exists, else we look for them in system paths.
if(EXISTS ${HEEX_3RDPARTY_DIR}/${HEEX_BOOST_FOLDER_NAME}/)
  set(Boost_NO_SYSTEM_PATHS TRUE)
  set(BOOST_ROOT ${HEEX_3RDPARTY_DIR}/${HEEX_BOOST_FOLDER_NAME}/)   # Hint on Boost project root directory in HEEX_3RDPARTY_DIR
  set(BOOST_INCLUDE_DIRS ${BOOST_ROOT}/boost/)                      # Hint on Boost include root directory
  set(BOOST_LIBRARY_DIRS ${BOOST_ROOT}/stage/lib/)                  # Hint on Boost lib root directory
  message(STATUS "Using Boost ${HEEX_BOOST_VERSION} libs from 3rd party for Ubuntu/Linux configurations at ${HEEX_3RDPARTY_DIR}/${HEEX_BOOST_FOLDER_NAME}")
  find_package(Boost ${HEEX_BOOST_VERSION} COMPONENTS ${HEEX_COM_BOOST_DEPS} REQUIRED)
else()
  message(STATUS "Using Boost ${HEEX_BOOST_VERSION} libs from system for Ubuntu/Linux configurations")
  find_package(Boost ${HEEX_BOOST_VERSION} COMPONENTS ${HEEX_COM_BOOST_DEPS} REQUIRED)
endif()

## Setup to use Boost header files and built librairies
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
message(STATUS "Boost include dir: ${Boost_INCLUDE_DIR}")
message(STATUS "Boost library dir: ${Boost_LIBRARIES}")
