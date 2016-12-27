cmake_minimum_required(VERSION 2.8)
project(poest)
#for opencv & pcl
find_package( OpenCV REQUIRED )

#
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
set(SOURCE_FILES  main.cpp poest.cpp calibration.cpp calibration.h)
#pcl required
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

add_executable(poest ${SOURCE_FILES})
#opencv & pcl
target_link_libraries( poest ${OpenCV_LIBS} )

