cmake_minimum_required(VERSION 3.0)
project(planner)

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

#--- Build libplanner.so
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  )

add_library(${PROJECT_NAME} SHARED
  ${PROJECT_SOURCE_DIR}/src/State/State.cpp
  ${PROJECT_SOURCE_DIR}/src/EuclideanSpace/EuclideanSpace.cpp
  ${PROJECT_SOURCE_DIR}/src/Constraint/ConstraintBase.cpp
  ${PROJECT_SOURCE_DIR}/src/Constraint/SemanticSegmentConstraint/SemanticSegmentConstraint.cpp
  ${PROJECT_SOURCE_DIR}/src/Constraint/PointCloudConstraint/PointCloudConstraint.cpp
  ${PROJECT_SOURCE_DIR}/src/Planner/PlannerBase.cpp
  ${PROJECT_SOURCE_DIR}/src/Planner/RRT/RRT.cpp
  ${PROJECT_SOURCE_DIR}/src/Planner/RRTStar/RRTStar.cpp)
