cmake_minimum_required(VERSION 3.0.0)
project(imd_controller VERSION 0.1.0)

set(CMAKE_CXX_FLAGS "-O -std=c++11 -Wall")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(imdc_obj OBJECT IMDController.cpp)

include(CTest)
enable_testing()

add_subdirectory(SerialBridge)

include_directories(SerialBridge/src)

target_link_libraries(imdc_obj serial_bridge)
target_link_libraries(imdc_obj udev)

add_library(imd_controller SHARED $<TARGET_OBJECTS:imdc_obj>)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
