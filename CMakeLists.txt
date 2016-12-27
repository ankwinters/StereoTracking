cmake_minimum_required(VERSION 2.8)
project(tracker)

#for opencv 2.4
find_package( OpenCV REQUIRED )
include_directories(${tracker_SOURCE_DIR}/include)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

set(SOURCE_FILES  main.cpp
        src/track.cpp
        src/feature_match.cpp
        include/track.h
        include/feature_match.h include/reconstruct.h src/reconstruct.cpp)
add_executable(tracker ${SOURCE_FILES})
#opencv
target_link_libraries(tracker ${OpenCV_LIBS} )

