# Robot Collision Checking

A lightweight package to use FCL with ROS messages (heavily inspired by [MoveIt's version](https://moveit.ros.org/documentation/concepts/developer_concepts/)). 

The package can be utilised to perform distance and collision checking of objects by creating a class and maintaining a world. The world objects can be represented using [shape_msgs](http://wiki.ros.org/shape_msgs), [octomap_msgs/Octomap](https://github.com/OctoMap/octomap_msgs), and [VoxelGrid](https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/msg/VoxelGrid.msg).

This package requires [FCL](http://www.ros.org/wiki/fcl) > 6.0.