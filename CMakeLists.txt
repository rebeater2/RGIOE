cmake_minimum_required(VERSION 3.17)

project(LooselyCouple2020_cpp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "release")
#set(CMAKE_C_COMPILER gcc)
#set(CMAKE_CXX_COMPILER clang++)



include_directories(
        Inc
)

file(GLOB_RECURSE SOURCE
        Src/*.*
        )


#add_compile_options(-O3)
#

#target_link_libraries(test_kalman Eigen3::Eigen)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)
find_package(lapack REQUIRED)

#find_package(mklibs REQUIRED)

#add_definitions(-DEIGEN_USE_LAPACK)
#add_definitions(-DEIGEN_USE_MKL_ALL)
#add_definitions(-DEIGEN_USE_MKL_ALL)


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
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/Lib)
#set(CUSTOM_OUTDIR Lib/)
set_target_properties(LooselyCouple PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CUSTOM_OUTDIR}"
        )
add_subdirectory(Test)
add_subdirectory(App)
add_subdirectory(Tools)
#add_link_options(-lgfortran -lm)

