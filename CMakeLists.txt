##########################################
# Top-level CMakeLists.txt for InfiniTAM #
##########################################

cmake_minimum_required(VERSION 2.8)

PROJECT(infinitam)

#################################
# Specify the CMake module path #
#################################

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

#####################
# Offer ROS Support #
#####################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/OfferROSSupport.cmake)

#################################
# Add additional compiler flags #
#################################

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
ENDIF()

IF(NOT MSVC_IDE)
  SET(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  SET(CMAKE_CXX_FLAGS "-fPIC -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
  #SET(CMAKE_CXX_FLAGS "-fPIC -g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
ENDIF()

#########################
# Suppress auto-linking #
#########################

IF(MSVC_IDE)
  ADD_DEFINITIONS(-DUSING_CMAKE=1)
ENDIF()

######################
# Add subdirectories #
######################

ADD_SUBDIRECTORY(Apps)
ADD_SUBDIRECTORY(FernRelocLib)
ADD_SUBDIRECTORY(InputSource)
ADD_SUBDIRECTORY(ITMLib)
ADD_SUBDIRECTORY(MiniSlamGraphLib)
ADD_SUBDIRECTORY(ORUtils)

#################################################################################################
# SLAMbench Libraries                                                                           #
#################################################################################################

### CPP
#########

add_library(${appname}-cpp infinitam_library.cpp)
target_link_libraries(${appname}-cpp  ITMLib MiniSlamGraphLib ORUtils FernRelocLib)
target_include_directories(${appname}-cpp  PRIVATE "ITMLib")
target_include_directories(${appname}-cpp  PRIVATE "ITMLib/Core")
target_include_directories(${appname}-cpp  PRIVATE "ITMLib/Engines")
target_include_directories(${appname}-cpp  PRIVATE "ITMLib/Utils")
target_include_directories(${appname}-cpp  PRIVATE "ITMLib/Trackers")
target_include_directories(${appname}-cpp  PRIVATE "ORUtils")

generate_slam_library(${appname}-cpp "")

### OMP
#########

add_library(${appname}-omp infinitam_library.cpp)
target_link_libraries(${appname}-omp  ITMLib MiniSlamGraphLib ORUtils FernRelocLib)
target_include_directories(${appname}-omp  PRIVATE "ITMLib")
target_include_directories(${appname}-omp  PRIVATE "ITMLib/Core")
target_include_directories(${appname}-omp  PRIVATE "ITMLib/Engines")
target_include_directories(${appname}-omp  PRIVATE "ITMLib/Utils")
target_include_directories(${appname}-omp  PRIVATE "ITMLib/Trackers")
target_include_directories(${appname}-omp  PRIVATE "ORUtils")

generate_slam_library(${appname}-omp "${OpenMP_CXX_FLAGS}")

IF (CUDA_FOUND)

  ### CUDA
  #########

  add_library(${appname}-cuda infinitam_library.cpp)
  target_link_libraries(${appname}-cuda  MiniSlamGraphLib ORUtils FernRelocLib ITMLibCUDA UtilsCUDA)
  target_include_directories(${appname}-cuda  PRIVATE "ITMLib")
  target_include_directories(${appname}-cuda  PRIVATE "ITMLib/Core")
  target_include_directories(${appname}-cuda  PRIVATE "ITMLib/Engines")
  target_include_directories(${appname}-cuda  PRIVATE "ITMLib/Utils")
  target_include_directories(${appname}-cuda  PRIVATE "ITMLib/Trackers")
  target_include_directories(${appname}-cuda  PRIVATE "ORUtils")

  generate_slam_library(${appname}-cuda "")

ENDIF()

