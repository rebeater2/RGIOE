cmake_minimum_required(VERSION 3.17)

project(LooselyCouple2020_cpp)

set(CMAKE_CXX_STANDARD 14)


find_package(Eigen3 REQUIRED)

include_directories(
        Inc Lib/eigen-3.3.9
)
add_executable(LooselyCouple2020 main.c
        Src/InsCore.cpp
        Src/wgs84.cpp
        Src/convert.cpp
        Src/wgs84.cpp)
target_link_libraries(LooselyCouple2020 Eigen3::Eigen)

add_compile_options(-O3)
#
add_executable(test_ins
        Src/InsCore.cpp
        Src/wgs84.cpp
        Src/convert.cpp
        Src/wgs84.cpp
        test_ins.cpp)
target_link_libraries(test_ins Eigen3::Eigen)

add_executable(test_kalman
        test_kalman.cpp
        Src/KalmanFilter.cpp
        )
#target_link_libraries(test_kalman Eigen3::Eigen)


find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)
find_package( lapack REQUIRED)
#find_package(mklibs REQUIRED)
add_definitions(-DGLOG_OUTPUT)
add_definitions(-DEIGEN_USE_LAPACK)
#add_definitions(-DEIGEN_USE_MKL_ALL)

add_executable(main main.c Src/wgs84.cpp)
target_link_libraries(main Eigen3::Eigen)
add_executable(dataFusion
        test_dataFusion.cpp
        Src/KalmanFilter.cpp
        Src/DataFusion.cpp
        Src/InsCore.cpp
        Src/wgs84.cpp
        Src/convert.cpp
        Src/FileIO.cpp
        Src/navigation_log.cpp
        Src/Config.cpp
        )
#include_directories(/home/rebeater/Downloads/cblas/CBLAS/include)
target_link_libraries(dataFusion
#        Eigen3::Eigen
        yaml-cpp
        glog
        lapack
        pthread
        )

add_executable(dataFusion_C
        main.c
        Src/LooselyCouple.cpp
        Src/KalmanFilter.cpp
        Src/DataFusion.cpp
        Src/InsCore.cpp
        Src/wgs84.cpp
        Src/convert.cpp
        Src/FileIO.cpp
        Src/navigation_log.cpp
        Src/Config.cpp
        )
#include_directories(Lib/eigen-3.3.9)
#set_property(SOURCE Lib/eigen-3.3.9 PROPERTY LANGUAGE CXX)
target_link_libraries(dataFusion_C
#        /home/rebeater/CLionProjects/LooselyCouple2020_cpp/cmake-build-debug/libLooselyCouple.a
        yaml-cpp
        glog
        lapack
        pthread
        )
add_library(LooselyCouple STATIC
        Src/LooselyCouple.cpp
        Src/KalmanFilter.cpp
        Src/DataFusion.cpp
        Src/InsCore.cpp
        Src/wgs84.cpp
        Src/convert.cpp
        Src/FileIO.cpp
        Src/navigation_log.cpp
        Src/Config.cpp)
target_link_libraries(LooselyCouple
        yaml-cpp
        glog
        lapack
        pthread
        )
set(CUSTOM_OUTDIR Lib/)
set_target_properties(LooselyCouple PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CUSTOM_OUTDIR}"
        )
#add_link_options(-lgfortran -lm)

