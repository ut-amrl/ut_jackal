PROJECT(ut_jackal)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CMAKE_CXX_FLAGS "-march=native -Wall -Werror -g")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -fno-builtin-malloc \
      -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -O3 -DNDEBUG")
  SET(BUILD_SPECIFIC_LIBRARIES "tcmalloc")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
  SET(BUILD_SPECIFIC_LIBRARIES "")
ENDIF()

MESSAGE(STATUS "SOURCE DIR: ${PROJECT_SOURCE_DIR}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

FIND_PACKAGE(Lua51 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

ADD_SUBDIRECTORY(enml/src/shared)

ROSBUILD_ADD_EXECUTABLE(enml_main
    enml/src/non_markov_localization_main.cpp
    enml/src/non_markov_localization.cpp
    enml/src/kdtree.cpp
    enml/src/gui_publisher_helper.cpp
    enml/src/perception_2d.cpp
    enml/src/vector_map/vector_map.cc
    enml/src/visualization/visualization.cc
    enml/src/visualization/gui_helpers.cc)
TARGET_INCLUDE_DIRECTORIES(enml_main PRIVATE
    enml/src
    enml/src/shared
    enml/src/config-reader/include
    ${LUA_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS})
ADD_DEPENDENCIES(enml_main rosbuild_precompile)
TARGET_LINK_LIBRARIES(enml_main ceres roslib roscpp ncurses pthread boost_system ${LUA_LIBRARIES} rt popt
    ${BUILD_SPECIFIC_LIBRARIES} glog rosbag X11 amrl-shared-lib)

ROSBUILD_ADD_EXECUTABLE(navigation_main
    navigation/src/navigation_main.cc
    navigation/src/navigation.cc
    navigation/src/visualization/visualization.cc
    navigation/src/vector_map/vector_map.cc)
TARGET_INCLUDE_DIRECTORIES(navigation_main PRIVATE
    navigation/src
    shared_lib
    config_reader/include
    ${LUA_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(navigation_main gflags roslib roscpp pthread boost_system
    rt glog rosbag X11 amrl-shared-lib ${LUA_LIBRARIES}
    ${BUILD_SPECIFIC_LIBRARIES})