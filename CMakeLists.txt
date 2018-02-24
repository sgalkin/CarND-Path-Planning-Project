project(t3p1)
cmake_minimum_required(VERSION 2.8)

option(LOCAL_BUILD "enables local build" ON)
option(DOCKER_BUILD "enables docker build" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

if(DOCKER_BUILD)
  if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "Inplace build is not supported for docker build. Please use separate directory.")
  endif()
  find_program(DOCKER_PATH docker)
  if(NOT DOCKER_PATH)
    message(FATAL_ERROR "Docker not found but DOCKER_BUILD is set")
  endif()
  include(EnableDockerBuild)
endif()

if(LOCAL_BUILD)
  include(EnableLocalBuild)
endif()
