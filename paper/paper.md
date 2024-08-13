---
title: 'robot_collision_checking: A lightweight wrapper to enable access to FCL (flexible collision library) via ROS and ROS 2'
tags:
  - ros
  - robotics
  - collision checking
  - flexible collision library
authors:
  - name: Mark Zolotas
    orcid: 0000-0002-7672-940X
    corresponding: true # (This is how to denote the corresponding author)
    equal-contrib: true # (This is how you can denote equal contributions between multiple authors)
    affiliation: 1
  - name: Philip Long
    orcid: 0000-0002-0784-8720
    equal-contrib: true # (This is how you can denote equal contributions between multiple authors)
    affiliation: 2
  - name: Taskin Padir
    orcid: 0000-0001-5123-5801
    affiliation: "1, 3"
affiliations:
 - name: Northeastern University, USA (at the time of this work)
   index: 1
 - name: Atlantic Technological University, Ireland
   index: 2
 - name: Amazon Robotics, USA
   index: 3
date: 13 August 2024
bibliography: paper.bib
---

# Summary
This paper presents `robot_collision_checking`, a C++ library that creates an easy interface to the Flexible Collision Library (FCL) [@Pan2012FCL]. The package allows users to access collision and distance checking functionalities of FCL directly through a Robot Operating System (ROS) interface [@Quigley2009ROS],  given that the robotics community widely relies on ROS as the standard for software development. We include ROS 1 and ROS 2 [@Macenski2022ROS2] implementations of the core C++ library.

Collisions and distances can be calculated between a variety of collision objects, including solid primitives (spheres, box, cylinder), planes, meshes, voxel grids, and octrees (via the Octomap library [@hornung13auro]). Collision worlds that contain multiple collision objects can also be created and maintained. This enables collision and distance checking between single objects, as well as entire collision worlds. The `robot_collision_checking` package includes an example node that demonstrates how to create a collision world of ROS objects, use FCL functionality to perform collision-checking on these objects, and visualize the world in RViz [@kam2015rviz].

The `robot_collision_checking` library is currently being used by the `constrained_manipulability`, developed by the same authors. Within the `constrained_manipulability` package there are more examples of using the `robot_collision_checking` library with URDF files and collision meshes in order to calculate collisions/distances between a robot and environmental objects, such as primitives and octomaps.   

# Statement of Need
Collision-checking is an increasingly important tool as robots are deployed into unstructured and dynamic environments, while ROS 1 and ROS 2 provide the most popular means of controlling robots for research applications. In the ROS ecosystem, one popular means of enabling collision-checking is via MoveIt [@coleman2014reducing], a path planning and trajectory execution open-source software. The MoveIt collision-checking API can expose two different collision checkers: [bullet](https://github.com/bulletphysics/bullet3) and FCL. However, to leverage this functionality users have to install the entire MoveIt suite and either integrate their robot into MoveIt or ensure that their platform is already available to the software suite. Moreover, while MoveIt is an extremely sophisticated motion planning library, accessing the lower-level functionalities for purposes like collision and distance checking requires in-depth knowledge of the library's structure and hierarchy. Pinocchio [@carpentier:hal-03271811] is another powerful robot modeling software that is also built upon FCL but suffers from the same overhead as MoveIt. The `robot_collision_checking` library aims to address the need for a lightweight alternative by providing a simple and transparent ROS interface to the FCL library. 

Our package is similar to [Python-fcl](https://github.com/BerkeleyAutomation/python-fcl), which provides a Python binding of FCL that could also be used in a ROS architecture. The key difference is that our implementation is written in C++. The [ros_collision_checking](https://github.com/CoFra-CaLa/ros_collision_detection) package also offers a collision-checking system for 2D vehicles in a ROS environment. Our collision-checking system instead extends the general capabilities of FCL for proximity querying any geometric model and can thus be applied in numerous robotics contexts.

# Acknowledgements
Taskin Padir holds concurrent appointments as a Professor of Electrical and Computer Engineering at Northeastern University and as an Amazon Scholar. This paper describes work performed at Northeastern University and is not associated with Amazon.

# References


