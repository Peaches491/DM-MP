cmake_minimum_required(VERSION 2.8.4)
project(DM_MP)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES Homework\ 3/myplugin/myplugin.cpp)
add_executable(DM_MP ${SOURCE_FILES})