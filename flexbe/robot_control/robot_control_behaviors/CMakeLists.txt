cmake_minimum_required(VERSION 2.8.3)
project(robot_control_behaviors)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
#    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME})

install(DIRECTORY manifest DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})