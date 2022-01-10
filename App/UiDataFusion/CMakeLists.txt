cmake_minimum_required(VERSION 3.19)
set(CMAKE_CXX_STANDARD 14)
project(UiDataFusion)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_compile_options(-O3)
find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)
message("found ${Qt5_VERSION} at ${Qt5_DIR}")
find_package(glog REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_library ( QWT_LIBRARY
        NAMES qwt qwt-qt5
        HINTS ${QT_LIBRARY_DIR}
        )

file(GLOB_RECURSE SOURCE
        ${CMAKE_HOME_DIRECTORY}/Src/*.*
        )

include_directories(Inc)
include_directories(../AppCommon)

add_executable(UiDataFusion
        main.cpp
        datafusionwindow.cpp
        DataFusionThread.cpp
        ../AppCommon/Config.cpp
        ../AppCommon/FileIO.cpp
        ../AppCommon/NavLog.cpp
        ${SOURCE}
        )
target_link_libraries(UiDataFusion
        Qt5::Widgets
        Qt5::Core
        Qt5::Gui
        glog
        fmt
        yaml-cpp
        Eigen3::Eigen
        pthread
        )

#find_package(yaml-cpp REQUIRED)
#[[
add_executable(testConfig config.cpp testConfig.cpp)
target_link_libraries(testConfig yaml-cpp)]]
