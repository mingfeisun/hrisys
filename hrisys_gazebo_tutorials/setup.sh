#!/usr/bin/env sh
export HRISYS_SDF_PATH=$(rospack find hrisys_gazebo_tutorials)/sdf
export GAZEBO_RESOURCE_PATH=$(rospack find hrisys_gazebo_tutorials)/worlds:$GAZEBO_RESOURCE_PATH
#export GAZEBO_PLUGIN_PATH=$(rospack find hrisys_gazebo_tutorials)/../../../build/hrisys_gazebo_tutorials:$GAZEBO_PLUGIN_PATH
