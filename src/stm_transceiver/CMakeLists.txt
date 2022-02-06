cmake_minimum_required(VERSION 2.8.3)
project(stm_transceiver)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    utils
)

find_package(Qt5 REQUIRED
  COMPONENTS
    Core
    SerialBus
    SerialPort
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS})

add_executable(can_transceiver
    src/can_transceiver.cpp
    include/can_transceiver.h)
target_link_libraries(can_transceiver
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::SerialBus)
catkin_install_python(PROGRAMS scripts/can_transceiver.py scripts/test.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
