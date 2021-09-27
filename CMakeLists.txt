cmake_minimum_required(VERSION 3.20)
project(sequence)
set(CMAKE_CXX_STANDARD 14)

include_directories(include)

add_library(sequence SHARED
            src/sequence_core.cpp
            src/sequence.cpp
            src/util.cpp)
