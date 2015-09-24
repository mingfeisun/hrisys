# hrisys for gazebo5, ROS J, Ubuntu T
cmake_minimum_required(VERSION 2.8.3)
project(hrisys_gazebo_tutorials)

# check c++11/c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c+11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(WARNING "c++11 required but not supported")
endif()

# find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# check gazebo and sdformat
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()
if (NOT GAZEBO_FOUND)
  message(FATAL_ERROR "gazebo was not found")
elseif (NOT SDF_FOUND)
  message(FATAL_ERROR "sdf was not found")
endif()

message(STATUS "found gazebo version:  " ${GAZEBO_VERSION})
message(STATUS "found sdformat version:  " ${SDF_VERSION})

# compile fails with lower or higher version of gazebo
if (${GAZEBO_VERSION} LESS 5.0)
  message(FATAL_ERROR "gazebo version too old, expected 5")
elseif (${GAZEBO_VERSION} GREATER 5.9)
  message(FATAL_ERROR "gazebo version too new, expected 5")
endif()

# compile also fails with SDF1, 3, or higher
if (${SDF_VERSION} LESS 2.0)
  message(FATAL_ERROR "sdf version too old, expected 2")
elseif (${SDF_VERSION} GREATER 2.9)
  message(FATAL_ERROR "sdf version too new, expected 2")
endif()

# check openal
include (FindOpenAL)
if (NOT OPENAL_FOUND)
  message(WARNING "OpenAL not found, some functions will be disabled")
else()
  message(STATUS "found OpenAL")
  add_definitions(-DHRISYS_HAVE_OPENAL)
  include_directories(${OPENAL_INCLUDE_DIR})
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(bvh_actor SHARED src/bvh_actor_tutorial.cc src/bvhactor.cc)
target_link_libraries(bvh_actor ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

# find libopenni2
if (PKG_CONFIG_FOUND)
  pkg_check_modules(OpenNI2 libopenni2)
endif()
if (NOT OpenNI2_FOUND)
  message(WARNING "OpenNI2 not found, some xtentions will be disabled")
else()
  # find NiTE2
  message(STATUS "found OpenNI2, looking for NiTE2 env")
  message(STATUS $ENV{NITE2_INCLUDE})
  message(STATUS $ENV{NITE2_REDIST64})
  find_path(Nite2_INCLUDEDIR NAMES NiTE.h HINTS $ENV{NITE2_INCLUDE})
  find_library(Nite2_LIBRARY NAMES NiTE2 HINTS $ENV{NITE2_REDIST64} PATH_SUFFIXES lib)
  if (Nite2_INCLUDEDIR AND Nite2_LIBRARY)
    message(STATUS "found NiTE2")
    add_definitions(-DHRISYS_HAVE_OPENNI)
    include_directories(${OpenNI2_INCLUDE_DIRS} ${Nite2_INCLUDEDIR})
    link_directories(${OpenNI2_LIBRARY_DIRS})
  else()
    message(WARNING "NiTE2 not found, some xtentions will be disabled")
  endif()
endif()

# find eigen
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIR)
  message(WARNING "Eigen3 not found, collision disabled")
else()
  message(STATUS "found Eigen3")
  include_directories(${EIGEN3_INCLUDE_DIR})
  add_definitions(-DHRISYS_HAVE_EIGEN)
endif()

# find ROS
find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros wiimote)
if(NOT catkin_INCLUDE_DIRS)
  message(WARNING "catkin not found, wii control is disabled")
else()
  message(STATUS "found catkin")
  include_directories(${catkin_INCLUDE_DIRS})
  add_definitions(-DHRISYS_HAVE_ROS)
endif()

# set(HRISYS_DEVEL 1)

if (COMPILER_SUPPORTS_CXX11 OR COMPILER_SUPPORTS_CXX0X)
  if (HRISYS_DEVEL)
    add_library(xtended_actor SHARED src/xtended_actor_devel.cc src/bvhactor.cc
      src/xtends/XBvhLimb.cc src/xtends/XTrackerDevel.cc src/xtends/XWalkerLimb.cc
      src/xtends/XLegKinematicsLimb.cc src/XtendedActor.cc)
  else()
    add_library(xtended_actor SHARED src/xtended_actor_tutorial.cc src/bvhactor.cc
      src/xtends/XBvhLimb.cc src/xtends/XTrackerLimb.cc src/xtends/XWalkerLimb.cc
      src/xtends/XLegKinematicsLimb.cc src/XtendedActor.cc)
  endif()
  target_link_libraries(xtended_actor ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
  if (Nite2_INCLUDEDIR AND Nite2_LIBRARY)
    target_link_libraries(xtended_actor ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY})
  endif()
  if (catkin_INCLUDE_DIRS)
    target_link_libraries(xtended_actor ${catkin_LIBRARIES})
  endif()
endif()