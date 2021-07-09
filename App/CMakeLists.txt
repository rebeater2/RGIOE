cmake_minimum_required(VERSION 3.17)
PROJECT(app_indoor)

MESSAGE(${CMAKE_HOME_DIRECTORY}/Src/)
file(GLOB_RECURSE SOURCE
        ${CMAKE_HOME_DIRECTORY}/Src/*.*
        )
add_executable(drIndoor drIndoor.cpp ${SOURCE})
target_link_libraries(drIndoor  yaml-cpp glog pthread    Eigen3::Eigen)

include_directories(${CMAKE_HOME_DIRECTORY}/Inc)
add_executable(PostDataFusion PostDataFusion.cpp)

target_link_libraries(PostDataFusion
        ${CMAKE_HOME_DIRECTORY}/Lib/libLooselyCouple.a
        yaml-cpp
        glog
        pthread
        Eigen3::Eigen
        )
