cmake_minimum_required(VERSION 3.0.2)
project(data_reader)


set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  roslib
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED QUIET)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

  )
  catkin_package(
    CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs roslib visualization_msgs
    DEPENDS EIGEN3 PCL 
    INCLUDE_DIRS include
  )

add_executable(data_reader src/main.cpp src/data_reader.cpp src/patchworkpp.cpp)
target_link_libraries(data_reader yaml-cpp ${PCL_LIBRARIES} ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})


