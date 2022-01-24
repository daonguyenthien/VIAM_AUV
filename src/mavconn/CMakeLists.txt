cmake_minimum_required(VERSION 2.8.3)
project(mavconn)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(console_bridge REQUIRED)
find_package(Boost REQUIRED
   COMPONENTS
    system
)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES mavconn)

add_library(mavconn
    src/interface.cpp
    src/serial.cpp
    src/tcp.cpp
    src/udp.cpp
    src/mavlink_helpers.cpp)
target_link_libraries(mavconn
    ${Boost_LIBRARIES}
    ${console_bridge_LIBRARIES})

install(TARGETS mavconn
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
