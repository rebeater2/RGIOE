cmake_minimum_required(VERSION 3.17)
PROJECT(RGIOE)

MESSAGE(${CMAKE_HOME_DIRECTORY}/Src/)
file(GLOB_RECURSE SOURCE
        ${CMAKE_HOME_DIRECTORY}/Src/*.*
        )

add_compile_options(-O2)

include_directories(${CMAKE_HOME_DIRECTORY}/Inc)
include_directories(${CMAKE_HOME_DIRECTORY}/Src)
include_directories(../AppCommon)

add_definitions(-DGLOG_OUTPUT)
add_definitions(-DUSE_YAML=1)
add_definitions(-DENABLE_FUSION_RECORDER)

find_package(fmt REQUIRED)
add_executable( PostDataFusion
        ${SOURCE}
        PostDataFusion.cpp
        ../AppCommon/FileIO.cpp
        ../AppCommon/NavLog.cpp
        ../AppCommon/Config.cpp
        )

target_link_libraries(PostDataFusion
        yaml-cpp
        glog
        pthread
        fmt
        Eigen3::Eigen
        )
set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}../../bin)

if(WIN32)
    function(copy_dll_on_win32 cur_lib tar_dir)
        set(libpath ${cur_lib}-NOTFOUND)
        set(DLL_PATH "D:\\Geek\\Winlibs\\msys2\\ucrt64\\bin\\")
        find_library(libpath cur_lib NAMES ${cur_lib}.dll PATHS ${DLL_PATH} REQUIRED)
        message("copy ${cur_lib} from ${libpath}  to ${tar_dir}")
        add_custom_command(TARGET PostDataFusion POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy
                "${libpath}"
                "${tar_dir}")
    endfunction()
    set(LIBS yaml-cpp
            glog
            pthread
            fmt)

    message("target: ${PROJECT_NAME}/plugins/platforms")
    message("EXECUTABLE_OUTPUT_PATH:${CMAKE_CURRENT_BINARY_DIR}")
    foreach(cur_lib ${LIBS})
        copy_dll_on_win32(${cur_lib} ${CMAKE_CURRENT_BINARY_DIR})
    endforeach()
    # execute_process(COMMAND ldd.exe ${CMAKE_CURRENT_BINARY_DIR}/PostDataFusion.exe)

endif()





#find_package(fmt REQUIRED)
#[[add_executable(DeadReckoning ../DeadReckoning.cpp ../FileIO.cpp Config.cpp NavLog.cpp ${SOURCE})
target_link_libraries(DeadReckoning
        #        ${CMAKE_HOME_DIRECTORY}/Lib/libLooselyCouple.a
        yaml-cpp
        glog
        pthread
        fmt
        Eigen3::Eigen
        )
# C语言接口
add_definitions(-DUSE_YAML=1)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
add_executable(PostDataFusion_c ../main.c ../ReadConfig.cpp Config.cpp ../ReadConfig.h)
target_link_libraries(PostDataFusion_c
        ${CMAKE_HOME_DIRECTORY}/Lib/libLooselyCouple.a
        Eigen3::Eigen
        yaml-cpp
        fmt
        )]]
