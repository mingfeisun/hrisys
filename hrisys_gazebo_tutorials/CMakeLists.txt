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

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# check gazebo and sdformat
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
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
  add_definitions(-DHRISYS_HAVE_OPENAL)
  include_directories(${OPENAL_INCLUDE_DIR})
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

if (COMPILER_SUPPORTS_CXX11 OR COMPILER_SUPPORTS_CXX0X)
  add_library(xtended_actor SHARED src/xtended_actor_tutorial.cc src/bvhactor.cc src/xtends/XBvhLimb.cc src/XtendedActor.cc)
  target_link_libraries(xtended_actor ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
endif()

add_library(bvh_actor SHARED src/bvh_actor_tutorial.cc src/bvhactor.cc)
target_link_libraries(bvh_actor ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

catkin_package(
)
