# Install script for directory: /home/alex/src/lunabotics/user/abethel/test_workspace/src/simple_action_server

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/alex/src/lunabotics/user/abethel/test_workspace/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simple_action_server/action" TYPE FILE FILES "/home/alex/src/lunabotics/user/abethel/test_workspace/src/simple_action_server/action/Fibonacci.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simple_action_server/msg" TYPE FILE FILES
    "/home/alex/src/lunabotics/user/abethel/test_workspace/devel/share/simple_action_server/msg/FibonacciAction.msg"
    "/home/alex/src/lunabotics/user/abethel/test_workspace/devel/share/simple_action_server/msg/FibonacciActionGoal.msg"
    "/home/alex/src/lunabotics/user/abethel/test_workspace/devel/share/simple_action_server/msg/FibonacciActionResult.msg"
    "/home/alex/src/lunabotics/user/abethel/test_workspace/devel/share/simple_action_server/msg/FibonacciActionFeedback.msg"
    "/home/alex/src/lunabotics/user/abethel/test_workspace/devel/share/simple_action_server/msg/FibonacciGoal.msg"
    "/home/alex/src/lunabotics/user/abethel/test_workspace/devel/share/simple_action_server/msg/FibonacciResult.msg"
    "/home/alex/src/lunabotics/user/abethel/test_workspace/devel/share/simple_action_server/msg/FibonacciFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simple_action_server/cmake" TYPE FILE FILES "/home/alex/src/lunabotics/user/abethel/test_workspace/src/simple_action_server/catkin_generated/installspace/simple_action_server-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/alex/src/lunabotics/user/abethel/test_workspace/devel/include/simple_action_server")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/alex/src/lunabotics/user/abethel/test_workspace/devel/share/roseus/ros/simple_action_server")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/alex/src/lunabotics/user/abethel/test_workspace/devel/share/common-lisp/ros/simple_action_server")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/alex/src/lunabotics/user/abethel/test_workspace/devel/share/gennodejs/ros/simple_action_server")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/alex/src/lunabotics/user/abethel/test_workspace/devel/lib/python3/dist-packages/simple_action_server")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/alex/src/lunabotics/user/abethel/test_workspace/devel/lib/python3/dist-packages/simple_action_server")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/alex/src/lunabotics/user/abethel/test_workspace/src/simple_action_server/catkin_generated/installspace/simple_action_server.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simple_action_server/cmake" TYPE FILE FILES "/home/alex/src/lunabotics/user/abethel/test_workspace/src/simple_action_server/catkin_generated/installspace/simple_action_server-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simple_action_server/cmake" TYPE FILE FILES
    "/home/alex/src/lunabotics/user/abethel/test_workspace/src/simple_action_server/catkin_generated/installspace/simple_action_serverConfig.cmake"
    "/home/alex/src/lunabotics/user/abethel/test_workspace/src/simple_action_server/catkin_generated/installspace/simple_action_serverConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simple_action_server" TYPE FILE FILES "/home/alex/src/lunabotics/user/abethel/test_workspace/src/simple_action_server/package.xml")
endif()

