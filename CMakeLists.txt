PROJECT(ut_jackal)
CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)


SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTORCC ON)
SET(CMAKE_AUTOUIC ON)

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

FIND_PACKAGE(Lua51 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(Qt5 COMPONENTS Core Widgets WebSockets REQUIRED)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)


INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

ADD_SUBDIRECTORY(shared_lib)

ROSBUILD_ADD_EXECUTABLE(enml
    enml/src/non_markov_localization_main.cpp
    enml/src/non_markov_localization.cpp
    enml/src/kdtree.cpp
    enml/src/perception_2d.cpp
    enml/src/vector_map/vector_map.cc
    enml/src/visualization/visualization.cc
    enml/src/visualization/gui_helpers.cc)
TARGET_INCLUDE_DIRECTORIES(enml PRIVATE
    enml/src
    enml/src/shared
    enml/src/config-reader/include
    ${LUA_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS})
ADD_DEPENDENCIES(enml rosbuild_precompile)
TARGET_LINK_LIBRARIES(enml ceres roslib roscpp ncurses pthread boost_system
    ${LUA_LIBRARIES} rt popt glog rosbag amrl_shared_lib
    ${BUILD_SPECIFIC_LIBRARIES})


ROSBUILD_ADD_EXECUTABLE(navigation
    navigation/src/navigation/navigation_main.cc
    navigation/src/navigation/navigation.cc
    navigation/src/navigation/motion_primitives.cc
    navigation/src/navigation/ackermann_motion_primitives.cc
    navigation/src/navigation/constant_curvature_arcs.cc
    navigation/src/navigation/linear_evaluator.cc
    navigation/src/visualization/visualization.cc
    navigation/src/vector_map/vector_map.cc)
TARGET_INCLUDE_DIRECTORIES(navigation PRIVATE
    navigation/src
    shared_lib
    config_reader/include
    navigation/src/third_party
    ${LUA_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(navigation gflags roslib roscpp pthread boost_system
    rt glog rosbag amrl_shared_lib ${LUA_LIBRARIES}
    ${BUILD_SPECIFIC_LIBRARIES})

ROSBUILD_ADD_EXECUTABLE(autonomy_arbiter
    autonomy_arbiter/autonomy_arbiter.cc)
TARGET_INCLUDE_DIRECTORIES(autonomy_arbiter PRIVATE
    shared_lib
    config_reader/include
    ${LUA_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(autonomy_arbiter gflags roslib roscpp pthread boost_system
    rt glog rosbag amrl_shared_lib ${LUA_LIBRARIES}
    ${BUILD_SPECIFIC_LIBRARIES})

ROSBUILD_ADD_EXECUTABLE(gps_translator
    gps/gps_translator.cc
    visualization/visualization.cc)
TARGET_INCLUDE_DIRECTORIES(gps_translator PRIVATE
    shared_lib
    config_reader/include
    ${LUA_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(gps_translator gflags roslib roscpp pthread boost_system
    rt glog rosbag amrl_shared_lib ${LUA_LIBRARIES}
    ${BUILD_SPECIFIC_LIBRARIES})

ROSBUILD_ADD_EXECUTABLE(status_translator
    status/status_translator.cc)
TARGET_INCLUDE_DIRECTORIES(status_translator PRIVATE
    shared_lib
    config_reader/include
    ${LUA_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(status_translator 
    gflags roslib roscpp pthread    
    boost_system rt glog rosbag amrl_shared_lib ${LUA_LIBRARIES}
    ${BUILD_SPECIFIC_LIBRARIES})

ROSBUILD_ADD_EXECUTABLE(websocket
    webviz/src/websocket/websocket_main.cc
    webviz/src/websocket/websocket.cc
)
TARGET_INCLUDE_DIRECTORIES(websocket PRIVATE
    shared_lib)
TARGET_LINK_LIBRARIES(websocket
    glog gflags roslib roscpp pthread amrl_shared_lib
    Qt5::Core Qt5::Widgets Qt5::WebSockets)
