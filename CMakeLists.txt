cmake_minimum_required(VERSION 3.5)
project(robot_collision_checking)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_voxel_grid REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

## System dependencies are found with CMake's conventions
find_package(CCD REQUIRED)
find_package(FCL 0.7.0 REQUIRED)
if(FCL_VERSION VERSION_LESS "0.7.0")
  message(WARNING "Your FCL version ${FCL_VERSION} is less than 0.7.0")
else()
  message("FCL version ${FCL_VERSION} is good!")
endif()

find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

### Libraries and executables ###

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  ${CCD_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  rclpy
  geometry_msgs
  geometric_shapes
  nav2_costmap_2d
  nav2_voxel_grid
  octomap_msgs
  shape_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

add_library(fcl_interface SHARED
  src/fcl_interface.cpp
)
target_link_libraries(fcl_interface
  ${Eigen3_LIBRARIES}
  ${FCL_LIBRARIES}
  ${CCD_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(fcl_interface
  ${dependencies}
)

### Install ###

install(TARGETS fcl_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

### Testing ###

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # FCL interface test
  ament_add_gtest(interface_test test/interface_test.cpp)
  target_include_directories(interface_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(interface_test
    ${dependencies}
  )
  target_link_libraries(interface_test fcl_interface)
endif()

### Ament stuff ###

ament_export_include_directories(include)
ament_export_libraries(fcl_interface)
ament_export_dependencies(${dependencies})

ament_package()