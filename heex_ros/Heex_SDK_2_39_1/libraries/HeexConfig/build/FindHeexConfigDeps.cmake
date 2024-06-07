# #############################################
# Find HeexUtils library dependencies.

message(STATUS "Looking for HeexUtils version " 2.39.1)
find_package(HeexUtils 2.39.1 REQUIRED PATHS ${HEEX_LIBRARIES_DIR}/HeexUtils/build/ NO_DEFAULT_PATH)
message(STATUS "HeexUtils_DIR=${HeexUtils_DIR}")
