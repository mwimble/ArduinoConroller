#####################################
cmake_minimum_required (VERSION 2.8) 
project (detecto)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake/)
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -std=gnu++0x")
find_package(OpenCV)
IF  ( OpenCV_FOUND)
  MESSAGE(STATUS "COMPILING OPENCV TESTS")
  add_executable (detecto detecto.cpp EwynCamera.cpp)  
  target_link_libraries (detecto ${OpenCV_LIBS})
ELSE()
  MESSAGE(FATAL_ERROR "OPENCV NOT FOUND IN YOUR SYSTEM")
ENDIF()
#####################################
