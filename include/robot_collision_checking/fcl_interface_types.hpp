#pragma once

#include <string>

#include <fcl/fcl.h>

#include <Eigen/Eigen>

#include <octomap_msgs/conversions.h>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/plane.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include <nav2_voxel_grid/voxel_grid.hpp>
#include <nav2_msgs/msg/voxel_grid.hpp>

namespace robot_collision_checking
{
enum ShapeType
{
    BOX = 1,
    SPHERE = 2,
    CYLINDER = 3,
    CONE = 4,
    PLANE = 5,
    MESH = 6,
    OCTOMAP = 7,
    VOXEL_GRID = 8
};

// FCL Object is simply a container for object information
struct FCLObject
{
    FCLObject(const shape_msgs::msg::SolidPrimitive& solid, ShapeType type, const Eigen::Affine3d& transform) 
        : object_type(type), object_transform(transform)
    {
        ptr.solid = new shape_msgs::msg::SolidPrimitive(solid);
    }

    FCLObject(const shape_msgs::msg::Plane& plane, ShapeType type, const Eigen::Affine3d& transform) 
        : object_type(type), object_transform(transform)
    {
        ptr.plane = new shape_msgs::msg::Plane(plane);
    }

    FCLObject(const shape_msgs::msg::Mesh& mesh, ShapeType type, const Eigen::Affine3d& transform) 
        : object_type(type), object_transform(transform)
    {
        ptr.mesh = new shape_msgs::msg::Mesh(mesh);
    }

    FCLObject(const octomap_msgs::msg::Octomap& octomap, ShapeType type, const Eigen::Affine3d& transform) 
        : object_type(type), object_transform(transform)
    {
        ptr.octomap = new octomap_msgs::msg::Octomap(octomap);
    }

    FCLObject(const nav2_msgs::msg::VoxelGrid& voxel_grid, ShapeType type, const Eigen::Affine3d& transform) 
        : object_type(type), object_transform(transform)
    {
        ptr.voxel_grid = new nav2_msgs::msg::VoxelGrid(voxel_grid);
    }

    ~FCLObject() 
    {
        // Release dynamically allocated memory, if any
        if (object_type == PLANE)
        {
            delete ptr.plane;
        }
        else if (object_type == MESH)
        {
            delete ptr.mesh;
        }
        else if (object_type == OCTOMAP)
        {
            delete ptr.octomap;
        }
        else if (object_type == VOXEL_GRID)
        {
            delete ptr.voxel_grid;
        }
        else
        {
            delete ptr.solid;
        }
    }

    // Returns a string of the corresponding
    std::string getTypeString() const
    {
        switch (object_type)
        {
        case BOX:
            return "BOX";
        case SPHERE:
            return "SPHERE";
        case CYLINDER:
            return "CYLINDER";
        case CONE:
            return "CONE";
        case PLANE:
            return "PLANE";
        case MESH:
            return "MESH";
        case OCTOMAP:
            return "OCTOMAP";
        case VOXEL_GRID:
            return "VOXEL_GRID";
        default:
            return "UNKNOWN";
        }
    }

    // Check if two FCLObject point to the same source object 
    bool sameObject(const FCLObject& other) const
    {
        return object_type == other.object_type && ptr.raw == other.ptr.raw;
    }

    ShapeType object_type;
    // The object's pose defined relative to the collision world frame
    Eigen::Affine3d object_transform;

    // Points to the type of FCL object
    union
    {
        const shape_msgs::msg::SolidPrimitive* solid;
        const shape_msgs::msg::Plane* plane;
        const shape_msgs::msg::Mesh* mesh;
        const octomap_msgs::msg::Octomap* octomap;
        const nav2_msgs::msg::VoxelGrid* voxel_grid;
        const void* raw; // Generic pointer type for comparison regardless of object type
    } ptr;
};

typedef std::shared_ptr<FCLObject> FCLObjectPtr;
typedef std::shared_ptr<fcl::CollisionObjectd> FCLCollisionObjectPtr;
typedef std::shared_ptr<fcl::CollisionGeometryd> FCLCollisionGeometryPtr;

// FCL wrapper around collision geometry and objects to interface with ROS
struct FCLInterfaceCollisionObject
{
    FCLInterfaceCollisionObject(const FCLObjectPtr& in_object, const FCLCollisionObjectPtr& in_co, int index) 
        : object(in_object), collision_object(in_co), collision_id(index) {}
    ~FCLInterfaceCollisionObject() {}

    FCLObjectPtr object;
    FCLCollisionObjectPtr collision_object;
    int collision_id;
};

typedef std::shared_ptr<FCLInterfaceCollisionObject> FCLInterfaceCollisionObjectPtr;
} // namespace robot_collision_checking