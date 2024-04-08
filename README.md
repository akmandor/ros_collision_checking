# Robot Collision Checking

A lightweight package to use FCL with ROS messages (heavily inspired by [MoveIt's version](https://moveit.ros.org/documentation/concepts/developer_concepts/)). 

The package can be utilised to perform distance and collision checking of objects by creating a class and maintaining a collision world, or by using utility functions that are free of any class. This package can handle objects represented as [shape_msgs](http://wiki.ros.org/shape_msgs), [OctoMaps](https://github.com/OctoMap/octomap_msgs), and [VoxelGrids](https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/msg/VoxelGrid.msg).

This package requires:
 * [FCL](https://github.com/flexible-collision-library/fcl) version **0.7.0**
 * [libccd](https://github.com/danfis/libccd) 

## Installation

FCL and `libccd-dev` may already be installed on your machine via your ROS distro, but these versions are likely outdated for the current repository's use. The following instructions will enable you to build the `robot_collision_checking` package within a ROS 2 workspace using `colcon build` (or `catkin build` if using ROS 1).

### libccd

Please ensure this option is enabled, when compiling: 
>> -DENABLE_DOUBLE_PRECISION=ON

1. git clone https://github.com/danfis/libccd.git
2. mkdir build && cd build
3. cmake -G "Unix Makefiles" -DENABLE_DOUBLE_PRECISION=ON ..
4. make
5. sudo make install

### FCL

1. git clone https://github.com/flexible-collision-library/fcl.git
2. mkdir build && cd build
3. cmake ..
4. make
5. sudo make install

If there are errors, such as constants not being found, then you are probably still using the older version of FCL.

## Testing

You can run tests for the `robot_collision_checking` package as described in [this ROS 2 tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/CLI.html). First compile the tests:
```
colcon test --ctest-args tests
```

And then examine the results:
```
colcon test-result --all --verbose
```