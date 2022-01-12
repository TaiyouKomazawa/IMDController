cmake_minimum_required(VERSION 3.0.0)
project(imd_controller VERSION 0.1.0)

include(CTest)
enable_testing()

add_subdirectory(SerialBridge)

add_library(imd_controller IMDController.cpp)

target_link_libraries(imd_controller serial_bridge)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
