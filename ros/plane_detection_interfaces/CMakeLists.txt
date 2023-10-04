cmake_minimum_required(VERSION 3.0.2)
project(plane_detection_interfaces)

find_package(catkin REQUIRED
  rospy
  std_msgs
  visualization_msgs
  sensor_msgs
  message_generation
)

 add_message_files(
    FILES
    Plane.msg
    PlaneEquation.msg
 )

 add_service_files(
   FILES
   DetectPlanes.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   visualization_msgs
   sensor_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_detection_interfaces
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

