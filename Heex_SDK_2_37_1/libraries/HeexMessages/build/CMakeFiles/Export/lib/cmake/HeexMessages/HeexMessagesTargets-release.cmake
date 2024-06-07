#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "HeexMessages::HeexMessages" for configuration "Release"
set_property(TARGET HeexMessages::HeexMessages APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(HeexMessages::HeexMessages PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libHeexMessages.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS HeexMessages::HeexMessages )
list(APPEND _IMPORT_CHECK_FILES_FOR_HeexMessages::HeexMessages "${_IMPORT_PREFIX}/lib/libHeexMessages.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
