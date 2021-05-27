cmake_minimum_required( VERSION 3.17 )
PROJECT(test)

MESSAGE(${CMAKE_HOME_DIRECTORY}/Src/)
file(GLOB_RECURSE SOURCE
        ${CMAKE_HOME_DIRECTORY}/Src/*.*
        )


add_executable(test_ins
        ../Src/InsCore.cpp
        ../Src/wgs84.cpp
        ../Src/convert.cpp
        ../Src/wgs84.cpp
         test_ins.cpp)
target_link_libraries(test_ins Eigen3::Eigen)

add_executable(test_kalman
        ./test_kalman.cpp
        ../Src/KalmanFilter.cpp
        )

add_executable(dataFusion
        ./test_dataFusion.cpp
        ${SOURCE}
        )
target_link_libraries(dataFusion
        yaml-cpp
        glog
        lapack
        pthread
        )

add_executable(dataFusion_C
        main.c
        ${SOURCE}
        )
include_directories(Lib/eigen-3.3.9)
add_definitions(-DUSE_YAML=1)
add_definitions(-DUSE_OUTAGE=0)

target_link_libraries(dataFusion_C
        yaml-cpp
        glog
        lapack
        pthread
        )

add_executable(test_align
      test_align.cpp
        ${SOURCE}
        )
target_link_libraries(test_align
        yaml-cpp
        glog
        lapack
        pthread
        )