cmake_minimum_required(VERSION 2.8)
project(poest)
#for opencv 2.4
find_package( OpenCV REQUIRED )

INCLUDE_DIRECTORIES("include")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")


set(SOURCE_FILES  main.cpp poest.cpp calibration.cpp src/track.cpp)

add_executable(poest ${SOURCE_FILES})
#opencv & pcl
target_link_libraries( poest ${OpenCV_LIBS} )

