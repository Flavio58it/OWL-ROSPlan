cmake_minimum_required(VERSION 2.8.3)
project(rosplan_perception_manager)
add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    std_srvs
    rosplan_knowledge_msgs
    nav_msgs
    geometry_msgs
    diagnostic_msgs
    visualization_msgs
    tf
    armor_msgs
)

## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs rosplan_knowledge_msgs nav_msgs geometry_msgs diagnostic_msgs visualization_msgs tf armor_msgs
  DEPENDS
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare cpp executables
add_executable(update_knowledge src/query.cpp)

## Specify libraries against which to link a library or executable target
target_link_libraries(update_knowledge ${catkin_LIBRARIES})
