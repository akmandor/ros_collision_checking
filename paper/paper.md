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

Collisions and distances can be calculated between  collision objects such as solid primitives (spheres, box, cylinder), planes, meshes, voxel grids and octrees (via the Octomap library (FCL) [@hornung13auro]. Collision worlds, containing multiple collision objecst can be created and maintained enabling collision distance checks between single objects and entire collision worlds. 



# Statement of Need
Collision checking is becoming an increasingly important tools as robots are deployed to unstructured and dynamic environments. Thi

Sentence about Moveit [@coleman2014reducing]

Sentence about FCL

Sentence about other packages

# Acknowledgements

Taskin Padir holds concurrent appointments as a Professor of Electrical and Computer Engineering at Northeastern University and as an Amazon Scholar. This paper describes work performed at Northeastern University and is not associated with Amazon.

# References


