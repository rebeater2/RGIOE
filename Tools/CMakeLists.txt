cmake_minimum_required(VERSION 3.15)
project(LooselyCouple2020_cpp)


set(CMAKE_CXX_STANDARD 11)
file(GLOB_RECURSE SOURCE
        ${CMAKE_HOME_DIRECTORY}/Src/*.*
        )

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
add_executable(showTraj showTraj.cpp ${SOURCE})
target_link_libraries(showTraj ${Pangolin_LIBRARIES} Eigen3::Eigen glog yaml-cpp)