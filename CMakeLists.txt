cmake_minimum_required(VERSION 3.3)
project(poest)
#for opencv & pcl
find_package( OpenCV REQUIRED )
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
#
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(SOURCE_FILES  pointcloud.cpp)
#pcl required
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#
add_executable(poest ${SOURCE_FILES})
#opencv & pcl
target_link_libraries( poest ${OpenCV_LIBS} ${PCL_LIBRARIES})
