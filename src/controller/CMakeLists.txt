cmake_minimum_required(VERSION 2.8.3)
project(controller)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    utils
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS})

add_executable(controller
    src/controller.cpp
    src/pid.cpp)
target_link_libraries(controller
    ${catkin_LIBRARIES})
