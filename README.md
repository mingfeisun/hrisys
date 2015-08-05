# hrisys
This package applies bvh motions to collada polygons in Gazebo for future human-robot-interaction simulations.

Currently, the package is compiled under Gazebo5 and SDF2 (which is the standard for ROS Jade and Ubuntu 14.04 or higher).
Packages for Gazebo7 or higher are to be developed in the near future.

The launch files may not work with current installed gazebo_ros package, but might work with compiled-from-source packages.

recommended:
Ubuntu 14.04 LTS, ROS Indigo, Gazebo 5.1.0, SDF 2.3.2

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
