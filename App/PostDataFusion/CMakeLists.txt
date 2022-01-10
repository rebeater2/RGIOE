cmake_minimum_required(VERSION 3.17)
PROJECT(LooselyCouple2020_cpp)

MESSAGE(${CMAKE_HOME_DIRECTORY}/Src/)
file(GLOB_RECURSE SOURCE
        ${CMAKE_HOME_DIRECTORY}/Src/*.*
        )

include_directories(${CMAKE_HOME_DIRECTORY}/Inc)
include_directories(../AppCommon)

add_definitions(-DGLOG_OUTPUT)
add_definitions(-DUSE_YAML=1)

find_package(fmt REQUIRED)
add_executable( PostDataFusion
        ${SOURCE}
        PostDataFusion.cpp
        ../AppCommon/FileIO.cpp
        ../AppCommon/NavLog.cpp
        ../AppCommon/Config.cpp
        )

target_link_libraries(PostDataFusion
        yaml-cpp
        glog
        pthread
        fmt
        Eigen3::Eigen
        )


#find_package(fmt REQUIRED)
#[[add_executable(DeadReckoning ../DeadReckoning.cpp ../FileIO.cpp Config.cpp NavLog.cpp ${SOURCE})
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
add_executable(PostDataFusion_c ../main.c ../ReadConfig.cpp Config.cpp ../ReadConfig.h)
target_link_libraries(PostDataFusion_c
        ${CMAKE_HOME_DIRECTORY}/Lib/libLooselyCouple.a
        Eigen3::Eigen
        yaml-cpp
        fmt
        )]]
