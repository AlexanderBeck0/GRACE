cmake_minimum_required(VERSION 2.8.12)

project(object_ros_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    message_generation
)

add_message_files(
  FILES
    BoundingBox.msg
    BoundingBoxes.msg
    ImageWithBoundingBoxes.msg
    RangeBearing.msg
    RangeBearings.msg
    Object2DArray.msg
    Object2D.msg
    RobotCommand.msg
)

add_action_files(
  FILES
    CheckForObjects.action
)

add_service_files(
  FILES
    BBNInfer.srv
    CalculateDistance.srv
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    std_msgs
)
