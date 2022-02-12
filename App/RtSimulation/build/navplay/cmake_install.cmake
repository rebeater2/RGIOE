# Install script for directory: /media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/navplay/msg" TYPE FILE FILES
    "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/gnss.msg"
    "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/imu.msg"
    "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/vel.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/navplay/cmake" TYPE FILE FILES "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/build/navplay/catkin_generated/installspace/navplay-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/devel/include/navplay")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/devel/share/roseus/ros/navplay")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/devel/share/common-lisp/ros/navplay")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/devel/share/gennodejs/ros/navplay")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/rebeater/miniconda3/envs/ros/bin/python3" -m compileall "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/devel/lib/python3/dist-packages/navplay")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/devel/lib/python3/dist-packages/navplay")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/build/navplay/catkin_generated/installspace/navplay.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/navplay/cmake" TYPE FILE FILES "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/build/navplay/catkin_generated/installspace/navplay-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/navplay/cmake" TYPE FILE FILES
    "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/build/navplay/catkin_generated/installspace/navplayConfig.cmake"
    "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/build/navplay/catkin_generated/installspace/navplayConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/navplay" TYPE FILE FILES "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/package.xml")
endif()

