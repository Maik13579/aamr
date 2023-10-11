cmake_minimum_required(VERSION 2.8.3)
project(perception_states)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME})