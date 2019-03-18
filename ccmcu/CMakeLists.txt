cmake_minimum_required(VERSION 3.0)

project(
    ccmcu
)

# use fPIC for all libraries
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# compile dependencies as static libraries
add_subdirectory(cave-crawler-lib)

# this is our main target
#add_library(nhve STATIC nhve.c)
#target_include_directories(nhve PRIVATE hardware-video-encoder)
#target_include_directories(nhve PRIVATE minimal-latency-streaming-protocol)

# note that nhve depends through hve on FFMpeg avcodec and avutil, at least 3.4 version
#target_link_libraries(nhve hve mlsp avcodec avutil)

# examples
add_executable(ccmcu main.cpp)
target_link_libraries(ccmcu cave-crawler)



