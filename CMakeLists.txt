cmake_minimum_required(VERSION 3.5)
project(imd_controller VERSION 0.1.0)


set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(SerialBridge)

include(CTest)
enable_testing()

add_library(${PROJECT_NAME} IMDController.cpp)

target_compile_options(${PROJECT_NAME}
    PUBLIC -Wall
)

target_include_directories(${PROJECT_NAME}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME} serial_bridge)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-export
    LIBRARY         DESTINATION lib
    INCLUDES        DESTINATION include
    PUBLIC_HEADER   DESTINATION lib/${PROJECT_NAME}
)
   
install(EXPORT ${PROJECT_NAME}-export 
    FILE ${PROJECT_NAME}-config.cmake
    DESTINATION share/cmake/${PROJECT_NAME}/
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
