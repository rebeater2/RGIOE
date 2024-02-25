cmake_minimum_required(VERSION 3.17)
PROJECT(RGIOE)

MESSAGE(${CMAKE_HOME_DIRECTORY}/Src/)
file(GLOB RGIOE_SOURCE
        ${CMAKE_HOME_DIRECTORY}/Src/*.cpp
        )


include_directories(${CMAKE_HOME_DIRECTORY}/Inc)
#include_directories(${CMAKE_HOME_DIRECTORY}/Src)
include_directories(../AppCommon)
include_directories(.)


# compiler options definition
# 1. for sate vector
option(EST_GYRO_SCALE OFF)
option(EST_ACCE_SCALE OFF)
option(EST_GNSS_LEVEL_ARM OFF)
option(EST_ODO_SCALE OFF)
# 2. for recorder
option(RECORDER ON)
# 3. for fast ins
option(FAST_INS OFF)


# estimate acce scale
if(EST_ACCE_SCALE)
    add_compile_options(-DRGIOE_ESTIMATE_ACCE_SCALE_FACTOR=1)
endif()

# estimate gyro scale
if(EST_GYRO_SCALE)
    add_compile_options(-DRGIOE_ESTIMATE_GYRO_SCALE_FACTOR=1)
endif()

# estimate gnss level arm
if(EST_GNSS_LEVEL_ARM)
    add_compile_options(-DRGIOE_ESTIMATE_GNSS_LEVEL_ARM=1)
endif()

#estimate odometer scale
if(EST_ODO_SCALE)
    add_compile_options(-DRGIOE_ESTIMATE_ACCE_SCALE_FACTOR=1)
endif()

if(RECORDER)
    message("recorder: ON")
    add_compile_options(-DENABLE_FUSION_RECORDER=1)
    file(GLOB RECORDER_SOURCE
            ${CMAKE_HOME_DIRECTORY}/Src/Recorder/*.cpp
            ${CMAKE_HOME_DIRECTORY}/Src/Recorder/*.c
    )
    include_directories(${CMAKE_HOME_DIRECTORY}/Src/Recorder)
endif()


if(FAST_INS)
    message("fast INS mode: ON")
    add_compile_options(-DRGIOE_FAST_INS=1)
endif()


find_package(fmt REQUIRED)

if (RECORDER)
    add_executable(PostDataFusion
            ${RGIOE_SOURCE}
            ${RECORDER_SOURCE}
            PostDataFusion.cpp
            DataManager.cpp
            ../AppCommon/Config.cpp
            ../AppCommon/FileIO.cpp
    )
else ()
add_executable(PostDataFusion
        ${RGIOE_SOURCE}
        PostDataFusion.cpp
        DataManager.cpp
        ../AppCommon/Config.cpp
        ../AppCommon/FileIO.cpp
        )
endif ()

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
           # pthread
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
