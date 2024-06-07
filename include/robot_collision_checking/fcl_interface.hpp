#pragma once

#include <vector>

#include <fcl/fcl.h>

#include <Eigen/Eigen>

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/plane.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include "robot_collision_checking/fcl_interface_types.hpp"
#include "robot_collision_checking/fcl_interface_collision_world.hpp"

namespace robot_collision_checking
{
namespace fcl_interface
{
// Create collision FCL geometries from specific ROS msgs
FCLCollisionGeometryPtr createCollisionGeometry(const FCLObjectPtr& obj);
FCLCollisionGeometryPtr createCollisionGeometry(const shape_msgs::msg::SolidPrimitive& solid);
FCLCollisionGeometryPtr createCollisionGeometry(const shape_msgs::msg::Plane& plane);
FCLCollisionGeometryPtr createCollisionGeometry(const shape_msgs::msg::Mesh& mesh);
FCLCollisionGeometryPtr createCollisionGeometry(const octomap_msgs::msg::Octomap& map);

// Check the collision between an FCL object and a given world
// Returns the number of collisions if any
int checkCollisionObjectWorld(const FCLObjectPtr& obj, const FCLInterfaceCollisionWorld& world);
int checkCollisionObjectWorld(const FCLCollisionObjectPtr& co, const FCLInterfaceCollisionWorld& world);

// Check if two FCL objects are in collision
bool checkCollisionObjects(const FCLObjectPtr& obj1, const FCLObjectPtr& obj2);
bool checkCollisionObjects(const FCLCollisionObjectPtr& co1, const FCLCollisionObjectPtr& co2);

/** Gets the distances from an FCL object to a given world
 * Returns:
 *    1. obj_distances a vector of distances (len = nbr of objects)
 *    2. closest_pt_object a vector of points (len = nbr of objects) on object closest to the given world
 *    3. closest_pt_world a vector of points (len = nbr of objects) on given world closest to the object */
void getObjectDistancesWorld(const FCLObjectPtr& obj,
                             const FCLInterfaceCollisionWorld& world,
                             std::vector<double>& obj_distances,
                             std::vector<Eigen::Vector3d>& closest_pt_obj,
                             std::vector<Eigen::Vector3d>& closest_pt_world);
void getObjectDistancesWorld(const FCLCollisionObjectPtr& co,
                             const FCLInterfaceCollisionWorld& world,
                             std::vector<double>& obj_distances,
                             std::vector<Eigen::Vector3d>& closest_pt_obj,
                             std::vector<Eigen::Vector3d>& closest_pt_world);

// Returns the minimum distance between two FCL objects
double getDistanceObjects(const FCLObjectPtr& obj1, const FCLObjectPtr& obj2);
double getDistanceObjects(const FCLCollisionObjectPtr& co1, const FCLCollisionObjectPtr& co2);

/** Returns the minimum distance between two FCL objects
 * Also returns the position w.r.t to world frame of the closest points */
double getDistanceObjects(const FCLObjectPtr& obj1,
                          const FCLObjectPtr& obj2,
                          Eigen::Vector3d& closest_pt_obj1,
                          Eigen::Vector3d& closest_pt_obj2);
double getDistanceObjects(const FCLCollisionObjectPtr& co1,
                          const FCLCollisionObjectPtr& co2,
                          Eigen::Vector3d& closest_pt_obj1,
                          Eigen::Vector3d& closest_pt_obj2);

// Get the minimum distance between an FCL object and a given world
double getMinimumObjectDistanceWorld(const FCLObjectPtr& obj, const FCLInterfaceCollisionWorld& world);
double getMinimumObjectDistanceWorld(const FCLCollisionObjectPtr& co, const FCLInterfaceCollisionWorld& world);

// Type conversion methods
void convertGeometryPoseEigenTransform(const geometry_msgs::msg::Pose& geo_pose, Eigen::Affine3d& wTt);
void convertEigenTransformGeometryPose(const Eigen::Affine3d& wTt, geometry_msgs::msg::Pose& geo_pose);

void convertGeometryPointEigenVector(const geometry_msgs::msg::Point& geo_point, Eigen::Vector3d& wPt);
void convertEigenVectorGeometryPoint(const Eigen::Vector3d& wPt, geometry_msgs::msg::Point& geo_point);

// Transforms an Eigen Affine3D to FCL coordinate transformation
inline void transform2fcl(const Eigen::Affine3d& b, fcl::Transform3d& f)
{
    f = b.matrix();
}
} // namespace fcl_interface
} // namespace robot_collision_checking