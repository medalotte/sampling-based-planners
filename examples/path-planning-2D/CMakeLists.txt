cmake_minimum_required(VERSION 3.0)
project(path-planning-2D)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

#--- enable output compile_command.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#--- $ cmake -DCMAKE_BUILD_TYPE=debug
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -MMD -Wall -Wextra -Winit-self")

#--- $ cmake
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall -O2 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2 -march=native")

#--- Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  add_definitions(-DCOMPILEDWITHC14)
  message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCOMPILEDWITHC0X)
  message(STATUS "Using flag -std=c++0x.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

find_package(Eigen3 3.0.0 REQUIRED)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV > 3.0 not found.")
endif()

#--- Build
set(LIBRARIES_DIR "${PROJECT_SOURCE_DIR}/../../lib")
set(EXTARNAL_REPOSITORY_DIR "${PROJECT_SOURCE_DIR}/../external")

include_directories(
  ${LIBRARIES_DIR}/include
  ${EXTARNAL_REPOSITORY_DIR}/OpenCV-YAMLHelper/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  )

link_directories(
  ${LIBRARIES_DIR}/build)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

add_executable(${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/src/main.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${EIGEN3_LIBS}
  ${OpenCV_LIBS}
  planner
  )
