cmake_minimum_required(VERSION 2.8.3)
project(hrisys_gazebo_tutorials)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(bvh_actor SHARED src/bvh_actor_tutorial.cc src/bvhactor.cc)
target_link_libraries(bvh_actor ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

catkin_package(
)
