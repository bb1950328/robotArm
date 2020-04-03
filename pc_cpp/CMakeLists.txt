cmake_minimum_required(VERSION 3.15)
project(robotArm)

set(CMAKE_CXX_STANDARD 14)

add_library(robotArm_library src/coupling.cpp src/libRobotArm.cpp src/Point3d.cpp src/Point3dLinkNode.cpp src/ServoState.cpp src/util.cpp)
add_executable(robotArm src/main.cpp)
target_link_libraries(robotArm PUBLIC robotArm_library)