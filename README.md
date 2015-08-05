# hrisys
This package applies BVH motions to collada polygons in Gazebo for future human-robot-interaction simulations.

Currently, the package is compiled under Gazebo5 and SDF2 (which is the standard for ROS Jade and Ubuntu 14.04 or higher).
Packages for Gazebo7 or higher are to be developed in the near future.

recommended:
Ubuntu 14.04 LTS, ROS Indigo, Gazebo 5.1.0, SDF 2.3.2

### Build

1. catkin build (with ROS)

 If you're not using ROS, go to step 2.  
 Assuming, Gazebo and sdformat is already installed, and that you have *catkin_tools*, go to your workspace,
 ```
 catkin build hrisys_gazebo_tutorials
 ```
 after build, setup the environments
 ```
 roscd hrisys_gazebo_tutorials
 source ./setup.sh
 ```
 Don't forget to set the GAZEBO_PLUGIN_PATH to whereever plugins were built.  
 An example of setting plugin path is commented out in setup.sh.

2. without ROS

 This step is only for those who did not compile with step 1.  
 Assuming, Gazebo and sdformat is already installed,
 ```
 cd hrisys_gazebo_tutorials
 mkdir build
 cd build
 cmake ..
 make
 ```
 after build, setup the environments
 ```
 cd ..
 export HRISYS_SDF_PATH=$(pwd)/sdf
 export GAZEBO_RESOURCE_PATH=$(pwd)/worlds:$GAZEBO_RESOURCE_PATH
 export GAZEBO_PLUGIN_PATH=$(pwd)/build:$GAZEBO_PLUGIN_PATH
 ```

### Tutorials

1. Prepare your BVH

 Assuming that you've finished the environmental setups above,  
 open worlds/bvh_test.instance with your favorite editor,  
 and add the BVH file you want to apply.
 ```
 <bvh_animation name="01_01">
   <filename><!--writehere--></filename>
 ```
 If you don't have any BVH files, a good place to start is trying the  
 CMU MotionBuilder-friendly BVH files.

2. Run the sample

 Now that everything is prepared, let's try the following sample
 ```
 gazebo bvh_actor_tutorial.world
 ```
 You should see a human model moving according to the BVH motions.

3. Sample using ROS

 As an alternative, if you have ROS,
 ```
 roslaunch hrisys_gazebo_tutorials bvh_actor_tutorial.launch
 ```
 However, the launch files may not work with current installed *gazebo_ros* package, but might work with compiled-from-source packages.

### Progress on hrisys

- Extended actor class for loading bvh motions.
- Tutorials for bvhactor class. 

### To be developed

- Operating actor via openni2 and tf.
- Apply different BVH motions to each limb.
- Automatic object grasping for actor operation using inverse-kinematics.
- BVH loading for non-compatible models. 
- Eye-sight movement.
- Morph targeting for facial expressions.
- Original polygon model with eye-sight movements and facial expressions. 
