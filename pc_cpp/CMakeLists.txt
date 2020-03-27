cmake_minimum_required(VERSION 3.15)
project(robotArm)

set(CMAKE_CXX_STANDARD 14)

add_executable(robotArm src/main.cpp src/coupling.cpp h/coupling.hpp src/utils.cpp h/utils.hpp)