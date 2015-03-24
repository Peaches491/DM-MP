cmake_minimum_required (VERSION 2.6.0)
project(kdtree)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g")

add_executable( kdtree kdtree.cpp )
install(TARGETS kdtree DESTINATION ./)