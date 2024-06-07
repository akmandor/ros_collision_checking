#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include <fcl/fcl.h>
#include <Eigen/Eigen>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/plane.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <nav2_voxel_grid/voxel_grid.hpp>
#include <nav2_msgs/msg/voxel_grid.hpp>

#include "robot_collision_checking/fcl_interface_types.hpp"
#include "robot_collision_checking/fcl_interface_collision_world.hpp"
#include "robot_collision_checking/fcl_interface.hpp"

TEST(FCLInterface, TransformToFCL)
{
    // Create some Eigen transforms in the world frame
    Eigen::Vector3d eig_wps1(0.0, 0.0, 0.0), eig_wps2(-1.3, 2.0, 0.3);
    Eigen::Matrix3d eig_identity = Eigen::Matrix3d::Identity();
    Eigen::Affine3d eig_wTs1, eig_wTs2;

    eig_wTs1.linear() = eig_identity;
    eig_wTs1.translation() = eig_wps1;
    eig_wTs2.linear() = eig_identity;
    eig_wTs2.translation() = eig_wps2;

    fcl::Transform3d fcl_wTs1, fcl_wTs2;
    robot_collision_checking::fcl_interface::transform2fcl(eig_wTs1, fcl_wTs1);
    robot_collision_checking::fcl_interface::transform2fcl(eig_wTs2, fcl_wTs2);

    ASSERT_TRUE(fcl_wTs1.translation().isApprox(eig_wTs1.translation()));
    ASSERT_TRUE(fcl_wTs2.translation().isApprox(eig_wTs2.translation()));
    ASSERT_TRUE(fcl_wTs1.linear().isApprox(eig_wTs1.linear()));
    ASSERT_TRUE(fcl_wTs2.linear().isApprox(eig_wTs2.linear()));
}

TEST(FCLInterface, AddRemove)
{
    robot_collision_checking::FCLInterfaceCollisionWorld collision_world;

    // Origin
    Eigen::Vector3d eig_wps(0.0, 0.0, 0.0);
    Eigen::Affine3d eig_wTs;
    eig_wTs.linear() = Eigen::Matrix3d::Identity();
    eig_wTs.translation() = eig_wps;

    // Create shape_msgs geometries
    shape_msgs::msg::SolidPrimitive sphere;
    sphere.dimensions.resize(1);
    sphere.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = 0.3;
    sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;

    robot_collision_checking::FCLObjectPtr fcl_sphere = std::make_shared<robot_collision_checking::FCLObject>(sphere, eig_wTs);
    bool success = collision_world.addCollisionObject(fcl_sphere, 0);

    ASSERT_EQ(collision_world.getNumObjects(), 1);
    ASSERT_TRUE(success);

    // Attempt to add another object with the same ID
    shape_msgs::msg::SolidPrimitive box;
    box.dimensions.resize(3);
    box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.2;
    box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.2;
    box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.2;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    robot_collision_checking::FCLObjectPtr fcl_box = std::make_shared<robot_collision_checking::FCLObject>(box, eig_wTs);

    success = collision_world.addCollisionObject(fcl_box, 0);

    ASSERT_EQ(collision_world.getNumObjects(), 1);
    ASSERT_FALSE(success);

    // Add box with a new ID
    success = collision_world.addCollisionObject(fcl_box, 1);

    ASSERT_EQ(collision_world.getNumObjects(), 2);
    ASSERT_TRUE(success);

    // Remove sphere
    success = collision_world.removeCollisionObject(0);

    ASSERT_EQ(collision_world.getNumObjects(), 1);
    ASSERT_TRUE(success);

    // Add a collection of objects
    std::vector<robot_collision_checking::FCLObjectPtr> fcl_objects;
    std::vector<int> object_ids = {2, 3, 4, 5};

    // Cylinder
    shape_msgs::msg::SolidPrimitive cylinder;
    cylinder.dimensions.resize(2);
    cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 1.0;
    cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.1;
    cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    fcl_objects.push_back(std::make_shared<robot_collision_checking::FCLObject>(cylinder, eig_wTs));

    // Plane
    shape_msgs::msg::Plane plane;
    plane.coef[0] = 0.2;
    plane.coef[1] = 0.4;
    plane.coef[2] = 0.0;
    // D takes this value to satisfy Ax+By+D=0
    plane.coef[3] = -(plane.coef[0] * plane.coef[0] + plane.coef[1] * plane.coef[1]);
    // The coefficients A, B, C give the normal to the plane
    Eigen::Vector3d n(plane.coef[0], plane.coef[1], plane.coef[2]);
    // Plane is centered at this point
    double distance = plane.coef[3] / n.norm();
    Eigen::Affine3d plane_pose;
    plane_pose.translation() = -distance * n.normalized();
    // Calculate the rotation matrix from the original normal z_0 = (0,0,1) to new normal n = (A,B,C)
    Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_0, n);
    plane_pose.linear() = q.toRotationMatrix();
    fcl_objects.push_back(std::make_shared<robot_collision_checking::FCLObject>(
        plane, robot_collision_checking::PLANE, plane_pose));

    // Mesh
    shape_msgs::msg::Mesh mesh;
    // Define vertices of the mesh
    geometry_msgs::msg::Point p1, p2, p3, p4, p5;
    p1.x = 0.0; p1.y = 0.0; p1.z = 1.0;   // Apex of the pyramid
    p2.x = 1.0; p2.y = 0.0; p2.z = 0.0;   // Base vertex 1
    p3.x = -1.0; p3.y = 0.0; p3.z = 0.0;  // Base vertex 2
    p4.x = 0.0; p4.y = 1.0; p4.z = 0.0;   // Base vertex 3
    p5.x = 0.0; p5.y = -1.0; p5.z = 0.0;  // Base vertex 4
    mesh.vertices.push_back(p1);
    mesh.vertices.push_back(p2);
    mesh.vertices.push_back(p3);
    mesh.vertices.push_back(p4);
    mesh.vertices.push_back(p5);
    // Define the faces of the mesh
    shape_msgs::msg::MeshTriangle t1, t2, t3, t4, t5, t6;
    t1.vertex_indices = {0, 1, 2};
    t2.vertex_indices = {0, 1, 3};
    t3.vertex_indices = {0, 2, 3};
    t4.vertex_indices = {0, 3, 4};
    t5.vertex_indices = {1, 2, 4}; 
    t6.vertex_indices = {1, 3, 4};  
    mesh.triangles.push_back(t1);
    mesh.triangles.push_back(t2);
    mesh.triangles.push_back(t3);
    mesh.triangles.push_back(t4);
    mesh.triangles.push_back(t5);
    mesh.triangles.push_back(t6);
    fcl_objects.push_back(std::make_shared<robot_collision_checking::FCLObject>(
        mesh, robot_collision_checking::MESH, eig_wTs));

    // Octomap
    octomap_msgs::msg::Octomap octomap;
    octomap::OcTree octree(0.1);
    // Insert some example points into the octree
    octomap::point3d point1(0.0, 0.0, 0.0);
    octomap::point3d point2(1.0, 1.0, 1.0);
    octree.updateNode(point1, true);
    octree.updateNode(point2, true);
    // Convert the octree to octomap binary format
    octomap_msgs::binaryMapToMsg(octree, octomap);
    fcl_objects.push_back(std::make_shared<robot_collision_checking::FCLObject>(
        octomap, robot_collision_checking::OCTOMAP, eig_wTs));

    // Add objects
    success = collision_world.addCollisionObjects(fcl_objects, object_ids);

    ASSERT_EQ(collision_world.getNumObjects(), 5);
    ASSERT_TRUE(success);

    // Remove objects
    success = collision_world.removeCollisionObjects(object_ids);

    ASSERT_EQ(collision_world.getNumObjects(), 1);
    ASSERT_TRUE(success);
}

TEST(FCLInterface, AddRemoveVoxelGrid)
{
    robot_collision_checking::FCLInterfaceCollisionWorld collision_world;

    // Origin
    Eigen::Vector3d eig_wps(0.0, 0.0, 0.0);
    Eigen::Affine3d eig_wTs;
    eig_wTs.linear() = Eigen::Matrix3d::Identity();
    eig_wTs.translation() = eig_wps;

    // VoxelGrid
    int size_x = 5, size_y = 2, size_z = 15;
    nav2_voxel_grid::VoxelGrid vg(size_x, size_y, size_z);
    // Mark all cells
    for (int x_grid = 0; x_grid < size_x; ++x_grid)
    {
        for (int y_grid = 0; y_grid < size_y; ++y_grid)
        {
            for (int z_grid = 0; z_grid < size_z; ++z_grid)
            {
                vg.markVoxel(x_grid, y_grid, z_grid);
            }
        }
    }
    // Create the VoxelGrid msg
    nav2_msgs::msg::VoxelGrid grid_msg;
    grid_msg.size_x = size_x;
    grid_msg.size_y = size_y;
    grid_msg.size_z = size_z;
    grid_msg.data.resize(size_x * size_y);
    memcpy(&grid_msg.data[0], vg.getData(), size_x * size_y * sizeof(int));

    robot_collision_checking::FCLObjectPtr fcl_voxel_grid = std::make_shared<robot_collision_checking::FCLObject>(
        grid_msg, robot_collision_checking::VOXEL_GRID, eig_wTs);

    bool success = collision_world.addCollisionObject(fcl_voxel_grid, 0);

    ASSERT_EQ(collision_world.getNumObjects(), size_x * size_y * size_z);
    ASSERT_TRUE(success);
}

TEST(FCLInterface, CollisionCheck)
{
    // Create some Eigen transforms in the world frame
    Eigen::Vector3d eig_wps1(0.0, 0.0, 0.0), eig_wps2(-1.3, 2.0, 0.3), eig_wps3(2.0, 0.5, 0.0), eig_wps4(-1.4, 1.9, 0.35), eig_wps5(-1.2, 2.1, 0.2);
    Eigen::Matrix3d eig_identity = Eigen::Matrix3d::Identity();
    Eigen::Affine3d eig_wTs1, eig_wTs2, eig_wTs3, eig_wTs4, eig_wTs5;

    eig_wTs1.linear() = eig_identity;
    eig_wTs1.translation() = eig_wps1;
    eig_wTs2.linear() = eig_identity;
    eig_wTs2.translation() = eig_wps2;
    eig_wTs3.linear() = eig_identity;
    eig_wTs3.translation() = eig_wps3;

    Eigen::Quaterniond q1(0.5, 0.5, 0.23, 0.43);
    q1.normalize();
    Eigen::Quaterniond q2(-0.5, 0.5, -1.23, 0.43);
    q2.normalize();

    eig_wTs4.linear() = q1.toRotationMatrix();
    eig_wTs4.translation() = eig_wps4;
    eig_wTs5.linear() = q2.toRotationMatrix();
    eig_wTs5.translation() = eig_wps5;
    
    // Convert to FCL coordinates
    fcl::Transform3d fcl_wTs1, fcl_wTs2, fcl_wTs3, fcl_wTs4, fcl_wTs5;
    robot_collision_checking::fcl_interface::transform2fcl(eig_wTs1, fcl_wTs1);
    robot_collision_checking::fcl_interface::transform2fcl(eig_wTs2, fcl_wTs2);
    robot_collision_checking::fcl_interface::transform2fcl(eig_wTs3, fcl_wTs3);
    robot_collision_checking::fcl_interface::transform2fcl(eig_wTs4, fcl_wTs4);
    robot_collision_checking::fcl_interface::transform2fcl(eig_wTs5, fcl_wTs5);

    // Create some primitive collision geometries
    std::shared_ptr<fcl::CollisionGeometryd> cg1 = std::make_shared<fcl::Sphered>(0.3);
    std::shared_ptr<fcl::CollisionGeometryd> cg2 = std::make_shared<fcl::Sphered>(0.75);
    std::shared_ptr<fcl::CollisionGeometryd> cg3 = std::make_shared<fcl::Boxd>(0.2, 0.2, 0.2);
    std::shared_ptr<fcl::CollisionGeometryd> cg4 = std::make_shared<fcl::Cylinderd>(0.1, 1.0);
    std::shared_ptr<fcl::CollisionGeometryd> cg5 = std::make_shared<fcl::Boxd>(0.25, 0.5, 0.2);

    // Create FCL collision objects
    fcl::CollisionObjectd *o1 = new fcl::CollisionObjectd(cg1, fcl_wTs1);
    fcl::CollisionObjectd *o2 = new fcl::CollisionObjectd(cg2, fcl_wTs2);
    fcl::CollisionObjectd *o3 = new fcl::CollisionObjectd(cg3, fcl_wTs3);
    fcl::CollisionObjectd *o4 = new fcl::CollisionObjectd(cg4, fcl_wTs4);
    fcl::CollisionObjectd *o5 = new fcl::CollisionObjectd(cg5, fcl_wTs5);

    // Create corresponding shape_msgs geometries and use our FCL Interface for ROS types
    shape_msgs::msg::SolidPrimitive sphere1;
    sphere1.dimensions.resize(1);
    sphere1.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = 0.3;
    sphere1.type = shape_msgs::msg::SolidPrimitive::SPHERE;

    shape_msgs::msg::SolidPrimitive sphere2;
    sphere2.dimensions.resize(1);
    sphere2.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = 0.75;
    sphere2.type = shape_msgs::msg::SolidPrimitive::SPHERE;

    shape_msgs::msg::SolidPrimitive box1;
    box1.dimensions.resize(3);
    box1.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.2;
    box1.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.2;
    box1.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.2;
    box1.type = shape_msgs::msg::SolidPrimitive::BOX;

    shape_msgs::msg::SolidPrimitive cylinder1;
    cylinder1.dimensions.resize(2);
    cylinder1.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 1.0;
    cylinder1.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.1;
    cylinder1.type = shape_msgs::msg::SolidPrimitive::CYLINDER;

    shape_msgs::msg::SolidPrimitive box2;
    box2.dimensions.resize(3);
    box2.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.25;
    box2.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.5;
    box2.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.2;
    box2.type = shape_msgs::msg::SolidPrimitive::BOX;

    // Convert to FCL interface objects
    robot_collision_checking::FCLObjectPtr fcl_sphere1 = std::make_shared<robot_collision_checking::FCLObject>(sphere1, eig_wTs1);
    robot_collision_checking::FCLObjectPtr fcl_sphere2 = std::make_shared<robot_collision_checking::FCLObject>(sphere2, eig_wTs2);
    robot_collision_checking::FCLObjectPtr fcl_box1 = std::make_shared<robot_collision_checking::FCLObject>(box1, eig_wTs3);
    robot_collision_checking::FCLObjectPtr fcl_cylinder1 = std::make_shared<robot_collision_checking::FCLObject>(cylinder1, eig_wTs4);
    robot_collision_checking::FCLObjectPtr fcl_box2 = std::make_shared<robot_collision_checking::FCLObject>(box2, eig_wTs5);

    // Collision checks
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    int total_collisions = 0;

    fcl::collide(o2, o1, request, result);
    bool is_collision = robot_collision_checking::fcl_interface::checkCollisionObjects(fcl_sphere2, fcl_sphere1);
    total_collisions += is_collision;
    ASSERT_EQ(result.isCollision(), is_collision);

    fcl::collide(o2, o3, request, result);
    is_collision = robot_collision_checking::fcl_interface::checkCollisionObjects(fcl_sphere2, fcl_box1);
    total_collisions += is_collision;
    ASSERT_EQ(result.isCollision(), is_collision);

    fcl::collide(o2, o4, request, result);
    is_collision = robot_collision_checking::fcl_interface::checkCollisionObjects(fcl_sphere2, fcl_cylinder1);
    total_collisions += is_collision;
    ASSERT_EQ(result.isCollision(), is_collision);

    fcl::collide(o2, o5, request, result);
    is_collision = robot_collision_checking::fcl_interface::checkCollisionObjects(fcl_sphere2, fcl_box2);
    total_collisions += is_collision;
    ASSERT_EQ(result.isCollision(), is_collision);

    // Check collisions in our world
    robot_collision_checking::FCLInterfaceCollisionWorld collision_world;

    collision_world.addCollisionObject(fcl_sphere1, 0);
    collision_world.addCollisionObject(fcl_box1, 1);
    collision_world.addCollisionObject(fcl_cylinder1, 2);
    collision_world.addCollisionObject(fcl_box2, 3);

    std::vector<int> collision_object_ids;
    is_collision = collision_world.checkCollisionObject(fcl_sphere2, collision_object_ids);
    int total_fcl_collisions = collision_object_ids.size();

    ASSERT_TRUE(is_collision);
    ASSERT_EQ(total_collisions, total_fcl_collisions);

    // Check against existing object with ID 0
    is_collision = collision_world.checkCollisionObject(0, collision_object_ids);
    total_fcl_collisions = collision_object_ids.size();

    ASSERT_FALSE(is_collision);
    ASSERT_EQ(0, total_fcl_collisions);

    // Check collisions in a given world of FCL objects
    int num_contacts = robot_collision_checking::fcl_interface::checkCollisionObjectWorld(fcl_sphere2, collision_world);
    ASSERT_GT(num_contacts, 0);
    ASSERT_EQ(total_collisions, num_contacts);
}

TEST(FCLInterface, DistanceCheck)
{
    // Create some Eigen transforms in the world frame
    Eigen::Vector3d eig_wps1(0.0, 0.0, 0.0), eig_wps2(-1.3, 2.0, 0.3), eig_wps3(2.0, 0.5, 0.0), eig_wps4(-1.4, 1.9, 0.35), eig_wps5(-1.2, 2.1, 0.2);
    Eigen::Matrix3d eig_identity = Eigen::Matrix3d::Identity();
    Eigen::Affine3d eig_wTs1, eig_wTs2, eig_wTs3, eig_wTs4, eig_wTs5;

    eig_wTs1.linear() = eig_identity;
    eig_wTs1.translation() = eig_wps1;
    eig_wTs2.linear() = eig_identity;
    eig_wTs2.translation() = eig_wps2;
    eig_wTs3.linear() = eig_identity;
    eig_wTs3.translation() = eig_wps3;

    Eigen::Quaterniond q1(0.5, 0.5, 0.23, 0.43);
    q1.normalize();
    Eigen::Quaterniond q2(-0.5, 0.5, -1.23, 0.43);
    q2.normalize();

    eig_wTs4.linear() = q1.toRotationMatrix();
    eig_wTs4.translation() = eig_wps4;
    eig_wTs5.linear() = q2.toRotationMatrix();
    eig_wTs5.translation() = eig_wps5;
    
    // Convert to FCL coordinates
    fcl::Transform3d fcl_wTs1, fcl_wTs2, fcl_wTs3, fcl_wTs4, fcl_wTs5;
    robot_collision_checking::fcl_interface::transform2fcl(eig_wTs1, fcl_wTs1);
    robot_collision_checking::fcl_interface::transform2fcl(eig_wTs2, fcl_wTs2);
    robot_collision_checking::fcl_interface::transform2fcl(eig_wTs3, fcl_wTs3);
    robot_collision_checking::fcl_interface::transform2fcl(eig_wTs4, fcl_wTs4);
    robot_collision_checking::fcl_interface::transform2fcl(eig_wTs5, fcl_wTs5);

    // Create some primitive collision geometries
    std::shared_ptr<fcl::CollisionGeometryd> cg1 = std::make_shared<fcl::Sphered>(0.3);
    std::shared_ptr<fcl::CollisionGeometryd> cg2 = std::make_shared<fcl::Sphered>(0.75);
    std::shared_ptr<fcl::CollisionGeometryd> cg3 = std::make_shared<fcl::Boxd>(0.2, 0.2, 0.2);
    std::shared_ptr<fcl::CollisionGeometryd> cg4 = std::make_shared<fcl::Cylinderd>(0.1, 1.0);
    std::shared_ptr<fcl::CollisionGeometryd> cg5 = std::make_shared<fcl::Boxd>(0.25, 0.5, 0.2);

    // Create FCL collision objects
    fcl::CollisionObjectd *o1 = new fcl::CollisionObjectd(cg1, fcl_wTs1);
    fcl::CollisionObjectd *o2 = new fcl::CollisionObjectd(cg2, fcl_wTs2);
    fcl::CollisionObjectd *o3 = new fcl::CollisionObjectd(cg3, fcl_wTs3);
    fcl::CollisionObjectd *o4 = new fcl::CollisionObjectd(cg4, fcl_wTs4);
    fcl::CollisionObjectd *o5 = new fcl::CollisionObjectd(cg5, fcl_wTs5);

    // Create corresponding shape_msgs geometries and use our FCL Interface for ROS types
    shape_msgs::msg::SolidPrimitive sphere1;
    sphere1.dimensions.resize(1);
    sphere1.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = 0.3;
    sphere1.type = shape_msgs::msg::SolidPrimitive::SPHERE;

    shape_msgs::msg::SolidPrimitive sphere2;
    sphere2.dimensions.resize(1);
    sphere2.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = 0.75;
    sphere2.type = shape_msgs::msg::SolidPrimitive::SPHERE;

    shape_msgs::msg::SolidPrimitive box1;
    box1.dimensions.resize(3);
    box1.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.2;
    box1.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.2;
    box1.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.2;
    box1.type = shape_msgs::msg::SolidPrimitive::BOX;

    shape_msgs::msg::SolidPrimitive cylinder1;
    cylinder1.dimensions.resize(2);
    cylinder1.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 1.0;
    cylinder1.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.1;
    cylinder1.type = shape_msgs::msg::SolidPrimitive::CYLINDER;

    shape_msgs::msg::SolidPrimitive box2;
    box2.dimensions.resize(3);
    box2.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.25;
    box2.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.5;
    box2.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.2;
    box2.type = shape_msgs::msg::SolidPrimitive::BOX;

    // Convert to FCL interface objects
    robot_collision_checking::FCLObjectPtr fcl_sphere1 = std::make_shared<robot_collision_checking::FCLObject>(sphere1, eig_wTs1);
    robot_collision_checking::FCLObjectPtr fcl_sphere2 = std::make_shared<robot_collision_checking::FCLObject>(sphere2, eig_wTs2);
    robot_collision_checking::FCLObjectPtr fcl_box1 = std::make_shared<robot_collision_checking::FCLObject>(box1, eig_wTs3);
    robot_collision_checking::FCLObjectPtr fcl_cylinder1 = std::make_shared<robot_collision_checking::FCLObject>(cylinder1, eig_wTs4);
    robot_collision_checking::FCLObjectPtr fcl_box2 = std::make_shared<robot_collision_checking::FCLObject>(box2, eig_wTs5);

    // Distance checks
    fcl::DistanceRequestd request;
    fcl::DistanceResultd result, result1, result2, result3;
    request.enable_nearest_points = false;
    request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

    fcl::distance(o1, o2, request, result);
    double dist = robot_collision_checking::fcl_interface::getDistanceObjects(fcl_sphere1, fcl_sphere2);
    ASSERT_DOUBLE_EQ(result.min_distance, dist);

    std::vector<double> distances;
    distances.push_back(result.min_distance);
    fcl::distance(o1, o3, request, result1);
    distances.push_back(result1.min_distance);
    fcl::distance(o1, o4, request, result2);
    distances.push_back(result2.min_distance);
    fcl::distance(o1, o5, request, result3);
    distances.push_back(result3.min_distance);

    // Create a world
    robot_collision_checking::FCLInterfaceCollisionWorld collision_world;

    collision_world.addCollisionObject(fcl_sphere2, 0);
    collision_world.addCollisionObject(fcl_box1, 1);
    collision_world.addCollisionObject(fcl_cylinder1, 2);
    collision_world.addCollisionObject(fcl_box2, 3);

    std::vector<double> world_distances;
    std::vector<Eigen::Vector3d> closest_pt_obj;
    std::vector<Eigen::Vector3d> closest_pt_world;
    collision_world.getObjectDistances(fcl_sphere1, world_distances, closest_pt_obj, closest_pt_world);

    EXPECT_EQ(distances, world_distances);
}

TEST(FCLInterface, OctomapCollDistCheck)
{
    // Origin
    Eigen::Vector3d eig_wps(0.0, 0.0, 0.0);
    Eigen::Affine3d eig_wTs;
    eig_wTs.linear() = Eigen::Matrix3d::Identity();
    eig_wTs.translation() = eig_wps;

    // Create shape_msgs geometries
    shape_msgs::msg::SolidPrimitive sphere;
    sphere.dimensions.resize(1);
    sphere.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = 0.3;
    sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;

    robot_collision_checking::FCLObjectPtr fcl_sphere = std::make_shared<robot_collision_checking::FCLObject>(sphere, eig_wTs);

    // Octomap
    octomap_msgs::msg::Octomap octomap;
    octomap::OcTree octree(0.1);
    // Insert some example points into the octree
    octomap::point3d point1(0.0, 0.0, 0.0);
    octomap::point3d point2(1.0, 1.0, 1.0);
    octree.updateNode(point1, true);
    octree.updateNode(point2, true);
    // Convert the octree to octomap binary format
    octomap_msgs::binaryMapToMsg(octree, octomap);
    robot_collision_checking::FCLObjectPtr fcl_octomap = std::make_shared<robot_collision_checking::FCLObject>(
        octomap, robot_collision_checking::OCTOMAP, eig_wTs);

    // Check in collision
    bool is_collision = robot_collision_checking::fcl_interface::checkCollisionObjects(fcl_sphere, fcl_octomap);
    ASSERT_TRUE(is_collision);

    // If two objects are in collision, min_distance <= 0.
    double dist = robot_collision_checking::fcl_interface::getDistanceObjects(fcl_sphere, fcl_octomap);
    ASSERT_LE(dist, 0);

    Eigen::Vector3d eig_wps_new(2.0, 2.0, 2.0);
    Eigen::Affine3d eig_wTs_new;
    eig_wTs_new.linear() = Eigen::Matrix3d::Identity();
    eig_wTs_new.translation() = eig_wps_new;

    robot_collision_checking::FCLObjectPtr fcl_sphere_new = std::make_shared<robot_collision_checking::FCLObject>(sphere, eig_wTs_new);

    // Successfully compute distance between Octomap and shape_msgs type
    double new_dist = robot_collision_checking::fcl_interface::getDistanceObjects(fcl_sphere_new, fcl_octomap);
    ASSERT_GT(new_dist, 0.0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
