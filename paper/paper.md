---
title: 'robot_collision_checking: A lightweight wrapper to enable access to FCL (flexible collision library) via ROS and ROS 2'
tags:
  - ros
  - robotics
  - Collision Checking
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
 - name: Amazon
   index: 3
date: 09 August 2024
bibliography: paper.bib
---

# Summary
This paper presents `robot_collision_checking`, a C++ library that creates an easy interface to the flexible collision library (FCL) [@Pan2012FCL] The package allows users to access collision and distance checking functionalities of FCL directly through a Robot Operating System (ROS) interface [@Quigley2009ROS],  given that the robotics community widely relies on ROS as the standard for software development. We include ROS 1 and ROS 2 [@Macenski2022ROS2] implementations of the core C++ library.

Collisions and distances can be calculated between  collision objects such as solid primitives (spheres, box, cylinder), planes, meshes, voxel grids and octrees (via the Octomap library (FCL) [@hornung13auro]. Collision worlds, containing multiple collision objects can be created and maintained enabling collision distance checks between single objects and entire collision worlds. The package includes an example of ROS integration and visualisation of the results.  

The `robot_collision_checking` library is currently being used by the `constrained_manipulability` by the same authors. Within this package there are examples of using `robot_collision_checking` with urdf files and collision meshes to calculate collision/distances between a robot and primitives and octomaps.   



# Statement of Need
Collision checking is becoming an increasingly important tools as robots are deployed to unstructured and dynamic environments, while ROS 1 and ROS 2 provide the most popular means of controlling robots for research applications. ROS 1 and ROS 2 enable collision checking via the popular path planning and trajectory execution open-source software Moveit [@coleman2014reducing]. Moveit collision checking api can avail two different collision checkers [bullet](https://github.com/bulletphysics/bullet3) and FCL. However, to avail of this functionality users have to install Moveit and ensure that their robot has a Moveit accessible package. Moreover, while Moveit is extremely powerful to access the lower level functionalities like collision distance checking requires an in-depth knowledge of the package structure and hierarchy. Similarly Pinochhio [@carpentier:hal-03271811] a powerful robot modeling software is also built upon FCL but suffers from the same overhead as Moveit.  
This package aims to address this need by providing a lightweight and transparent wrapper for the FCL library and ROS. 

Simiar to this package [Python-fcl](https://github.com/BerkeleyAutomation/python-fcl) provides a python wrapper for FCL which could also be used for a ROS interface but is based on python rather than C++. [ros_collision_checking](https://github.com/CoFra-CaLa/ros_collision_detection) provides a collision checking system for 2D vehicles in a ROS environment. 


# Acknowledgements

Taskin Padir holds concurrent appointments as a Professor of Electrical and Computer Engineering at Northeastern University and as an Amazon Scholar. This paper describes work performed at Northeastern University and is not associated with Amazon.

# References


