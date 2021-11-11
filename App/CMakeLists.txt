cmake_minimum_required(VERSION 3.17)
PROJECT(App)

MESSAGE(${CMAKE_HOME_DIRECTORY}/Src/)
file(GLOB_RECURSE SOURCE
        ${CMAKE_HOME_DIRECTORY}/Src/*.*
        )
#[[add_executable(drIndoor drIndoor.cpp ${SOURCE})
target_link_libraries(drIndoor  yaml-cpp glog pthread    Eigen3::Eigen)]]

include_directories(${CMAKE_HOME_DIRECTORY}/Inc)
add_definitions(-DGLOG_OUTPUT)
add_definitions(-DUSE_YAML=1)
#define GLOG_OUTPUT 0
#define USE_YAML 0
add_executable(PostDataFusion
        ${SOURCE}
        PostDataFusion.cpp
        FileIO.cpp
        Config.cpp
        NavLog.cpp
        )

target_link_libraries(PostDataFusion
        yaml-cpp
        glog
        pthread
        fmt
        Eigen3::Eigen
        )

find_package(fmt REQUIRED)
add_executable(DeadReckoning DeadReckoning.cpp FileIO.cpp Config.cpp NavLog.cpp    ${SOURCE})
target_link_libraries(DeadReckoning
#        ${CMAKE_HOME_DIRECTORY}/Lib/libLooselyCouple.a
        yaml-cpp
        glog
        pthread
        fmt
        Eigen3::Eigen
        )
# C语言接口
add_definitions(-DUSE_YAML=1)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
add_executable(PostDataFusion_c main.c ReadConfig.cpp ReadConfig.h Config.cpp)
target_link_libraries(PostDataFusion_c
        ${CMAKE_HOME_DIRECTORY}/Lib/libLooselyCouple.a
        Eigen3::Eigen
        yaml-cpp
        fmt
        )
