cmake_minimum_required(VERSION 3.17)
PROJECT(app_indoor)

MESSAGE(${CMAKE_HOME_DIRECTORY}/Src/)
file(GLOB_RECURSE SOURCE
        ${CMAKE_HOME_DIRECTORY}/Src/*.*
        )
add_executable(drIndoor drIndoor.cpp ${SOURCE})
include_directories(${CMAKE_HOME_DIRECTORY}/Lib/eigen-3.3.9)
target_link_libraries(drIndoor  yaml-cpp glog pthread)