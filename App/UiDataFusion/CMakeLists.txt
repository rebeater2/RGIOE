cmake_minimum_required(VERSION 3.19)
set(CMAKE_CXX_STANDARD 14)
project(RGIOE)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_compile_options(-O3)
file(GLOB_RECURSE SOURCE
        ${CMAKE_HOME_DIRECTORY}/Src/*.*
        )

message("=======Searching for glog======")
find_package(glog REQUIRED)
message("found glog-${glog_VERSION} at ${glog_DIR}")
message("=======Searching for fmt======")
find_package(fmt REQUIRED)
message("found fmt-${fmt_VERSION} at ${fmt_DIR}")
message("=======Searching for eigen3======")
find_package(Eigen3 REQUIRED)
message("found Eigen3-${Eigen3_VERSION} at ${Eigen3_DIR}")

MESSAGE("=======Searching for QT5======")
find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)
message("found ${Qt5_VERSION} at ${Qt5_DIR}")

MESSAGE("=======Searching for QWT======")
message("Qt include dir:${QT5_INCLUDE_DIR}")
FIND_PATH(QWT_INCLUDE_DIR NAMES qwt.h PATHS ${QT5_INCLUDE_DIR} PATH_SUFFIXES qwt)
MESSAGE(QWT_INCLUDE_DIR at ${QWT_INCLUDE_DIR})
FIND_LIBRARY(QWT_LIBRARY NAMES qwt-qt5 PATHS /usr/lib /usr/local/lib)
#set( QWT_LIBRARY /usr/lib/libqwt-qt5.so)
MESSAGE(“QWT_LIBRARY at ${QWT_LIBRARY}”)

#设置项目包含目录，包括将第三方库附加
INCLUDE_DIRECTORIES(
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${Projects_QT5_Includes}
        ${QWT_INCLUDE_DIR}
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

add_executable(NavPlot
        navplot_main.cpp
        navplot.cpp
        navplot.ui
        )
target_link_libraries(NavPlot
        Qt5::Widgets
        Qt5::Core
        Qt5::Gui
        qwt-qt5
        glog
        fmt
        yaml-cpp
        Eigen3::Eigen
        pthread
        ${QWT_LIBRARY}
        )
# copy font file
set(font_path "${CMAKE_CURRENT_BINARY_DIR}/fonts")
add_custom_command(TARGET UiDataFusion PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E
        make_directory ${font_path}
        COMMAND ${CMAKE_COMMAND} -E
        copy ${CMAKE_CURRENT_SOURCE_DIR}/fonts/*.ttc ${font_path}
        )
#find_package(yaml-cpp REQUIRED)
#[[
add_executable(testConfig config.cpp testConfig.cpp)
target_link_libraries(testConfig yaml-cpp)]]
