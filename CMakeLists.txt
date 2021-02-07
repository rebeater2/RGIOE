cmake_minimum_required(VERSION 3.17)

project(LooselyCouple2020_cpp)

set(CMAKE_CXX_STANDARD 14)


find_package(Eigen3 REQUIRED)

include_directories(
        Inc
)
add_executable(LooselyCouple2020 main.cpp
        Src/ins_core.cpp
        Src/wgs84.cpp
        Src/convert.cpp
        Src/wgs84.cpp)
target_link_libraries(LooselyCouple2020 Eigen3::Eigen)

add_compile_options(-O3)

add_executable(test_ins
        Src/ins_core.cpp
        Src/wgs84.cpp
        Src/convert.cpp
        Src/wgs84.cpp
        test_ins.cpp)
target_link_libraries(test_ins Eigen3::Eigen)

add_executable(test_kalman
        test_kalman.cpp
        Src/KalmanFilter.cpp
        )
target_link_libraries(test_kalman Eigen3::Eigen)


find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)
#find_package( lapack REQUIRED)
#find_package(mklibs REQUIRED)
add_definitions(-DGLOG_OUTPUT)
add_definitions(-DEIGEN_USE_LAPACK)
#add_definitions(-DEIGEN_USE_MKL_ALL)

add_executable(dataFusion
        test_dataFusion.cpp
        Src/KalmanFilter.cpp
        Src/DataFusion.cpp
        Src/ins_core.cpp
        Src/wgs84.cpp
        Src/convert.cpp
        Src/f_io.cpp
        Src/navigation_log.cpp
        Src/Config.cpp
        )
include_directories(/home/rebeater/Downloads/cblas/CBLAS/include)
target_link_libraries(dataFusion
        Eigen3::Eigen
        yaml-cpp
        glog
#        lapack
        pthread
        )
#add_link_options(-lgfortran -lm)

