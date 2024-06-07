#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "HeexCom::TcpServer" for configuration "Release"
set_property(TARGET HeexCom::TcpServer APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(HeexCom::TcpServer PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libTcpServer.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS HeexCom::TcpServer )
list(APPEND _IMPORT_CHECK_FILES_FOR_HeexCom::TcpServer "${_IMPORT_PREFIX}/lib/libTcpServer.a" )

# Import target "HeexCom::TcpClient" for configuration "Release"
set_property(TARGET HeexCom::TcpClient APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(HeexCom::TcpClient PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libTcpClient.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS HeexCom::TcpClient )
list(APPEND _IMPORT_CHECK_FILES_FOR_HeexCom::TcpClient "${_IMPORT_PREFIX}/lib/libTcpClient.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
