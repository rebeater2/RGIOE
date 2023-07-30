cmake_minimum_required( VERSION 3.17 )
PROJECT(LooselyCouple2020_cpp)

MESSAGE(${CMAKE_HOME_DIRECTORY}/Src/)
file(GLOB_RECURSE SOURCE
        ${CMAKE_HOME_DIRECTORY}/Src/*.*
        )

#add_definitions(-DUSE_INCREMENT=1)
#define USE_INCREMENT 1c

set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}../../App/bin)
find_package(fmt REQUIRED)
include_directories(../App/AppCommon)
add_executable(test_ins
        ../Src/InsCore.cpp
        ../Src/Earth.cpp
        ../Src/Convert.cpp
        ../Src/Earth.cpp
        ../App/AppCommon/FileIO.cpp
         test_ins.cpp)
target_link_libraries(test_ins Eigen3::Eigen fmt pthread)

add_executable(test_kalman
        ./test_kalman.cpp
        ../Src/KalmanFilter.cpp
        )
target_link_libraries(test_kalman Eigen3::Eigen)

#add_executable(testOutage
#        ../Src/Outage.cpp
#        testOutage.cpp
#        )