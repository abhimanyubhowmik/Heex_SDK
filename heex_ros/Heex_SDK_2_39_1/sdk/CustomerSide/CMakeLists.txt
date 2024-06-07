set (SOURCE_CUSTOMERSIDE
    ${CMAKE_CURRENT_SOURCE_DIR}/Agent.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Monitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MonitorV2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BaseMonitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ZoneMonitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/InstantMonitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/IntervalMonitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BooleanMonitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/StringMonitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GenericMonitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/HoldSignalOnMonitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Recorder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/RecorderV2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/RecorderInThePast.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ValueConfiguration.cpp
    PARENT_SCOPE
)
