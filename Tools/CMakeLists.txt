cmake_minimum_required(VERSION 3.15)
project(LooselyCouple2020_cpp)


set(CMAKE_CXX_STANDARD 11)
file(GLOB_RECURSE SOURCE
        ${CMAKE_HOME_DIRECTORY}/Src/*.*
        )

add_definitions(-DGLOG_OUTPUT=1)
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})


add_executable(showTraj showTraj.cpp ${SOURCE})
target_link_libraries(showTraj ${Pangolin_LIBRARIES} Eigen3::Eigen glog yaml-cpp fmt)

add_executable(DataGenerator DataGenerator.cpp ${SOURCE})
target_link_libraries(DataGenerator  Eigen3::Eigen  fmt)

add_executable(ZeroDetector ZeroDetector.cpp ${SOURCE})
target_link_libraries(ZeroDetector  Eigen3::Eigen  fmt)

include_directories(../App/AppCommon)
add_executable(ImuFix ImuFix.cpp ../App/AppCommon/FileIO.cpp ${SOURCE})
target_link_libraries(ImuFix  Eigen3::Eigen pthread fmt)

add_executable(ObtainInstallAngle  ObtainInstallAngle.cpp ../App/AppCommon/FileIO.cpp ${SOURCE})
target_link_libraries(ObtainInstallAngle  Eigen3::Eigen pthread fmt glog)