cmake_minimum_required(VERSION 2.8.3)
project(utils)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation

    std_msgs
    geometry_msgs
)

add_message_files(
  FILES
    Odometry.msg
    Setpoint.msg
    Error.msg
    MotorsCommand.msg
    KeyboardCommand.msg
    Waypoint.msg
    WaypointList.msg
    ParamValue.msg
    ThrusterStatus.msg
    MassShifterStatus.msg
    PistonStatus.msg
    BoardARM1Status.msg
    BoardARM2Status.msg
)

add_service_files(
  FILES
    CommandInt.srv
    CommandLong.srv
    ParamGet.srv
    ParamSet.srv
    SetMode.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include)

include_directories(
    include
    ${catkin_INCLUDE_DIRS})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
