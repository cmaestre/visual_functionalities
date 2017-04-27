cmake_minimum_required(VERSION 2.8.3)
project(visual_functionalities)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

add_service_files(
  FILES
  GetRealObjectState.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
#  LIBRARIES crustcrawler_cppkdl
#  CATKIN_DEPENDS moveit_ros_planning_interface roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

file ( GLOB_RECURSE all_sources src/lib/* )

## Service files

add_executable(get_real_object_state_service src/services/get_real_object_state_service.cpp ${all_sources})
target_link_libraries(get_real_object_state_service ${catkin_LIBRARIES})

## Test files


