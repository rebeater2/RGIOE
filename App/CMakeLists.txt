cmake_minimum_required(VERSION 3.17)
PROJECT(App)

MESSAGE(${CMAKE_HOME_DIRECTORY}/Src/)
file(GLOB_RECURSE SOURCE
        ${CMAKE_HOME_DIRECTORY}/Src/*.*
        )
add_executable(drIndoor drIndoor.cpp ${SOURCE})
target_link_libraries(drIndoor  yaml-cpp glog pthread    Eigen3::Eigen)

include_directories(${CMAKE_HOME_DIRECTORY}/Inc)
add_definitions(-DGLOG_OUTPUT)
add_definitions(-DUSE_YAML=0)
#define GLOG_OUTPUT 0
#define USE_YAML 0
add_executable(PostDataFusion
        PostDataFusion.cpp
        FileIO.cpp
        Config.cpp
        NavLog.cpp
        )

target_link_libraries(PostDataFusion
        ${CMAKE_HOME_DIRECTORY}/Lib/libLooselyCouple.a
        yaml-cpp
        glog
        pthread
        Eigen3::Eigen
        )
