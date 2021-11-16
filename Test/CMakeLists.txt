cmake_minimum_required( VERSION 3.17 )
PROJECT(LooselyCouple2020_cpp)

MESSAGE(${CMAKE_HOME_DIRECTORY}/Src/)
file(GLOB_RECURSE SOURCE
        ${CMAKE_HOME_DIRECTORY}/Src/*.*
        )

#add_definitions(-DUSE_INCREMENT=1)
#define USE_INCREMENT 1c
find_package(fmt REQUIRED)
add_executable(test_ins
        ../Src/InsCore.cpp
        ../Src/WGS84.cpp
        ../Src/Convert.cpp
        ../Src/WGS84.cpp
        ../App/FileIO.cpp
         test_ins.cpp)
target_link_libraries(test_ins Eigen3::Eigen fmt pthread)

add_executable(test_kalman
        ./test_kalman.cpp
        ../Src/KalmanFilter.cpp
        )
target_link_libraries(test_kalman Eigen3::Eigen)

add_executable(test_align
      test_align.cpp
        ../App/FileIO.cpp
        ../App/Config.cpp
        ${SOURCE}
        )

target_link_libraries(test_align
        yaml-cpp
        glog
        fmt
        lapack
        pthread
        Eigen3::Eigen
        )

add_executable(dataFusiion
        test_dataFusion.cpp
        ../Src/InsCore.cpp
        ../Src/WGS84.cpp
        ../Src/Convert.cpp
        ../Src/DataFusion.cpp
        ../Src/KalmanFilter.cpp
        ../Src/Alignment.cpp
        ../App/FileIO.cpp
        ../App/Config.cpp
        ../App/NavLog.cpp
        ../Src/StaticDetect.cpp
        )
target_link_libraries(dataFusiion
        yaml-cpp
        glog
        fmt
        fmt
        lapack
        pthread
        Eigen3::Eigen
)