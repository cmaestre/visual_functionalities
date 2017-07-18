cmake_minimum_required(VERSION 2.8.3)
project(visual_functionalities)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  moveit_ros_planning_interface
  image_geometry
  image_transport
  roscpp
  pcl_conversions
  sensor_msgs
  std_msgs
  tf
  baxter_core_msgs
  crustcrawler_core_msgs
  message_generation
)

find_package(OpenCV REQUIRED COMPONENTS features2d nonfree)

# EIGEN
#find_package(Eigen3 REQUIRED)
find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
find_package(Boost REQUIRED COMPONENTS system )
find_package(PCL 1.7 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_path(RGBD_UTILS_INCLUDE_DIR rgbd_utils/rgbd_subscriber.hpp)
find_library(RGBD_UTILS_LIBRARY NAMES rgbd_utils)

find_path(IMAGE_PROCESSING_INCLUDE_DIR image_processing/MotionDetection.h)
find_library(IMAGE_PROCESSING_LIBRARY NAMES image_processing)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

add_service_files(
  FILES
  object_detection_by_qr_code.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES visual_functionalities
  CATKIN_DEPENDS cv_bridge moveit_ros_planning_interface image_geometry image_transport roscpp sensor_msgs std_msgs tf baxter_core_msgs crustcrawler_core_msgs message_runtime
#  DEPENDS system_lib
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include /opt/ros/indigo/include/)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(visual_functionalities
        src/visual_functionalities.cpp
        src/libs/lib_recording.cpp
        )
target_link_libraries(visual_functionalities /usr/local/lib/libaruco.so ${catkin_LIBRARIES} ${PCL_LIBRARIES} rgbd_utils)

## Mark executables and/or libraries for installation
install(TARGETS visual_functionalities
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


## Mark cpp header files for installation
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})




## Test files
add_executable(test_visual_functionalities src/test_visual_functionalities.cpp)
target_link_libraries(test_visual_functionalities /usr/local/lib/libaruco.so  ${catkin_LIBRARIES} visual_functionalities)

