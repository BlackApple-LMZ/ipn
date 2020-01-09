cmake_minimum_required(VERSION 2.8.3)
project(ipn_pointcloud)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  ipn_msgs
  tf
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package( CATKIN_DEPENDS
  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(
  ${OpenCV_LIB_DIR}
)

add_executable(cloud_node src/cloud_node.cc src/convert.cc src/pointcloudXYZIR.cc src/rawdata.cc)
target_link_libraries(cloud_node ${catkin_LIBRARIES})
                      
add_executable(rawDataTest tests/rawDataTest.cpp)
target_link_libraries(rawDataTest ${catkin_LIBRARIES} ${OpenCV_LIBS})


