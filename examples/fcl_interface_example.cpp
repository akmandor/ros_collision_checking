#include <memory>
#include <vector>

#include <fcl/fcl.h>
#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

#include <geometric_shapes/shape_to_marker.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/plane.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include <nav2_voxel_grid/voxel_grid.hpp>
#include <nav2_msgs/msg/voxel_grid.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <robot_collision_checking/fcl_interface.hpp>

bool initCollisionWorld(robot_collision_checking::FCLInterfaceCollisionWorld& world)
{
    // Create some Eigen transforms in the world frame
    Eigen::Vector3d eig_wps1(0.0, 0.0, 0.0), eig_wps2(-1.3, 2.0, 0.3), eig_wps4(-1.4, 1.9, 0.35), eig_wps5(1.2, -1.1, 0.2);
    Eigen::Matrix3d eig_identity = Eigen::Matrix3d::Identity();
    Eigen::Affine3d eig_wTs1, eig_wTs2, eig_wTs3, eig_wTs4, eig_wTs5;

    eig_wTs1.linear() = eig_identity;
    eig_wTs1.translation() = eig_wps1;
    eig_wTs2.linear() = eig_identity;
    eig_wTs2.translation() = eig_wps2;

    Eigen::Quaterniond q1(0.5, 0.5, 0.23, 0.43);
    q1.normalize();
    Eigen::Quaterniond q2(-0.5, 0.5, -1.23, 0.43);
    q2.normalize();

    eig_wTs4.linear() = q1.toRotationMatrix();
    eig_wTs4.translation() = eig_wps4;
    eig_wTs5.linear() = q2.toRotationMatrix();
    eig_wTs5.translation() = eig_wps5;

    // Add a collection of objects
    std::vector<robot_collision_checking::FCLObjectPtr> fcl_objects;
    std::vector<int> object_ids = {0, 1, 2, 3, 4};

    // Sphere
    shape_msgs::msg::SolidPrimitive sphere;
    sphere.dimensions.resize(1);
    sphere.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = 0.3;
    sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    fcl_objects.push_back(std::make_shared<robot_collision_checking::FCLObject>(
        sphere, robot_collision_checking::SPHERE, eig_wTs1));

    // Cylinder
    shape_msgs::msg::SolidPrimitive cylinder;
    cylinder.dimensions.resize(2);
    cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 1.0;
    cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.1;
    cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    fcl_objects.push_back(std::make_shared<robot_collision_checking::FCLObject>(
        cylinder, robot_collision_checking::CYLINDER, eig_wTs2));

    // Plane Ax+By+Cz+D=0
    shape_msgs::msg::Plane plane;
    plane.coef[0] = 1.0;
    plane.coef[1] = 2.0;
    plane.coef[2] = 0.0;
    // D takes this value to satisfy Ax+By+D=0
    plane.coef[3] = -(plane.coef[0] * plane.coef[0] + plane.coef[1] * plane.coef[1]);
    // The coefficients A, B, C give the normal to the plane
    Eigen::Vector3d n(plane.coef[0], plane.coef[1], plane.coef[2]);
    // Plane is centered at this point
    double distance = plane.coef[3] / n.norm();
    eig_wTs3.translation() = -distance * n.normalized();
    // Calculate the rotation matrix from the original normal z_0 = (0,0,1) to new normal n = (A,B,C)
    Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_0, n);
    eig_wTs3.linear() = q.toRotationMatrix();
    fcl_objects.push_back(std::make_shared<robot_collision_checking::FCLObject>(
        plane, robot_collision_checking::PLANE, eig_wTs3));

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
        mesh, robot_collision_checking::MESH, eig_wTs4));

    // Voxel grid
    int size_x = 4, size_y = 2, size_z = 3;
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
    grid_msg.origin.x = 0.0;
    grid_msg.origin.y = 0.0;
    grid_msg.origin.z = 0.0;
    grid_msg.size_x = size_x;
    grid_msg.size_y = size_y;
    grid_msg.size_z = size_z;
    grid_msg.resolutions.x = 0.5;
    grid_msg.resolutions.y = 0.5;
    grid_msg.resolutions.z = 0.5;
    grid_msg.data.resize(size_x * size_y);
    memcpy(&grid_msg.data[0], vg.getData(), size_x * size_y * sizeof(int));

    fcl_objects.push_back(std::make_shared<robot_collision_checking::FCLObject>(
        grid_msg, robot_collision_checking::VOXEL_GRID, eig_wTs5));

    return world.addCollisionObjects(fcl_objects, object_ids);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("fcl_interface_example");

    auto mkr_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/fcl_world_markers", rclcpp::QoS(3));

    RCLCPP_INFO(node->get_logger(), "FCL interface example to visualize collision world objects");

    // Initialize the FCL collision world
    robot_collision_checking::FCLInterfaceCollisionWorld collision_world("world");
    bool success = initCollisionWorld(collision_world);
    RCLCPP_INFO(node->get_logger(), "Successful creation of FCL collision world? %s", success ? "Yes!" : "No...");
    
    // MAIN PUBLISHING LOOP
    rclcpp::Rate r(1.0);
    while (rclcpp::ok())
    {
        auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();

        std::vector<robot_collision_checking::FCLInterfaceCollisionObjectPtr> world_objects = collision_world.getCollisionObjects();
        std::string world_frame = collision_world.getWorldFrame();

        // Ignore even shapes
        int num_objects = collision_world.getNumObjects();
        for (int i = 0; i < num_objects; /*i++*/)
        {
            auto world_obj = world_objects[i];
            // Make a marker
            visualization_msgs::msg::Marker mkr;
            mkr.ns = "collision_objects";
            mkr.header.frame_id = world_frame;
            mkr.action = visualization_msgs::msg::Marker::ADD;
            mkr.lifetime = rclcpp::Duration(0, 0);
            std::string obj_type = world_obj->object->getTypeString();

            // Get object pose relative to world_frame
            Eigen::Affine3d object_eig_pose = world_obj->object->object_transform;
            geometry_msgs::msg::Pose object_geo_pose;
            robot_collision_checking::convertEigenTransformGeometryPose(object_eig_pose, object_geo_pose);
            mkr.pose = object_geo_pose;

            if (obj_type == "MESH")
            {
                geometric_shapes::constructMarkerFromShape(*(world_obj->object->ptr.mesh), mkr);

                // Blue mesh
                mkr.id = world_obj->collision_id;
                mkr.color.b = 1.0;
                mkr.color.a = 1.0;
                marker_array_msg->markers.push_back(mkr);
                i++;
            }
            else if (obj_type == "PLANE")
            {
                mkr.scale.x = 10.0;
                mkr.scale.y = 10.0;
                mkr.scale.z = 0.001; // very thin

                // Red cuboid
                mkr.type = visualization_msgs::msg::Marker::CUBE;
                mkr.id = world_obj->collision_id;
                mkr.color.r = 1.0;
                mkr.color.a = 0.3;
                marker_array_msg->markers.push_back(mkr);
                i++;
            }
            else if (obj_type == "OCTOMAP")
            {
                RCLCPP_WARN(node->get_logger(), "Unable to display octomap");
                i++;
            }
            else if (obj_type == "VOXEL_GRID")
            {
                mkr.id = world_obj->collision_id;
                // Treat voxel grid as a cube list
                mkr.type = visualization_msgs::msg::Marker::CUBE_LIST;
                auto grid = *(world_obj->object->ptr.voxel_grid);
                mkr.scale.x = grid.resolutions.x;
                mkr.scale.y = grid.resolutions.y;
                mkr.scale.z = grid.resolutions.z;
                // Voxel cells already account for position in world
                mkr.pose.position.x = mkr.pose.position.y = mkr.pose.position.z = 0.0;

                // Iterate over remaining cells until no more objects in world or a new collision object
                do
                {
                    // The collision object is really a voxel cell
                    auto voxel_cell = *(world_objects[i]->collision_object);
                    fcl::Vector3d cell_center = voxel_cell.getTranslation();
                    // Invert rotation to obtain cell position in original world frame
                    Eigen::Matrix3d rotation_matrix = voxel_cell.getRotation().matrix();
                    rotation_matrix.transposeInPlace();
                    cell_center = rotation_matrix * cell_center;
                    geometry_msgs::msg::Point point;
                    point.x = cell_center[0];
                    point.y = cell_center[1];
                    point.z = cell_center[2];
                    mkr.points.push_back(point);
                    i++;
                } while ((i < num_objects) && (world_objects[i-1]->collision_id == world_objects[i]->collision_id));

                // Purple voxel grid
                mkr.color.r = 1.0;
                mkr.color.b = 1.0;
                mkr.color.a = 1.0;
                marker_array_msg->markers.push_back(mkr);
            }
            else if (obj_type == "SPHERE" || obj_type == "BOX" || obj_type == "CYLINDER" || obj_type == "CONE")
            {
                geometric_shapes::constructMarkerFromShape(*(world_obj->object->ptr.solid), mkr);

                // Green primitives
                mkr.id = world_obj->collision_id;
                mkr.color.g = 1.0;
                mkr.color.a = 1.0;
                marker_array_msg->markers.push_back(mkr);
                i++;
            }

            std::vector<int> collision_object_ids;

            bool is_collision = collision_world.checkCollisionObject(world_obj->collision_id, collision_object_ids);
            if (is_collision)
            {
                for (int obj_id : collision_object_ids)
                {
                    RCLCPP_INFO(node->get_logger(), "%s with ID %d in collision with object with ID %d", 
                                obj_type.c_str(), world_obj->collision_id, obj_id);
                }
            }
        }

        mkr_pub->publish(*marker_array_msg);

        rclcpp::spin_some(node);
        r.sleep();
    }

    rclcpp::shutdown();

    return 0;
}