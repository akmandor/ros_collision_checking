#ifndef FCL_INTERFACE_HPP
#define FCL_INTERFACE_HPP


#include <string>
#include <vector>

#include <fcl/fcl.h>

#include <Eigen/Eigen>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shape_to_marker.h>

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/plane.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include <nav2_voxel_grid/voxel_grid.hpp>
#include <nav2_msgs/msg/voxel_grid.hpp>

namespace robot_collision_checking
{
    rclcpp::Logger getLogger()
    {
        return rclcpp::get_logger("robot_collision_checking.fcl_interface");
    }

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
            ptr.solid = &solid;
        }

        FCLObject(const shape_msgs::msg::Plane& plane, ShapeType type, const Eigen::Affine3d& transform) 
            : object_type(type), object_transform(transform)
        {
            ptr.plane = &plane;
        }

        FCLObject(const shape_msgs::msg::Mesh& mesh, ShapeType type, const Eigen::Affine3d& transform) 
            : object_type(type), object_transform(transform)
        {
            ptr.mesh = &mesh;
        }

        FCLObject(const octomap_msgs::msg::Octomap& octomap, ShapeType type, const Eigen::Affine3d& transform) 
            : object_type(type), object_transform(transform)
        {
            ptr.octomap = &octomap;
        }

        FCLObject(const nav2_msgs::msg::VoxelGrid& voxel_grid, ShapeType type, const Eigen::Affine3d& transform) 
            : object_type(type), object_transform(transform)
        {
            ptr.voxel_grid = &voxel_grid;
        }

        ~FCLObject() {}

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

    class FCLInterface
    {
        public:
            FCLInterface();
            ~FCLInterface();

            bool collisionObjectExists(int object_id) const;

            // Create collision FCL geometries from specific ROS msgs
            FCLCollisionGeometryPtr createCollisionGeometry(const FCLObjectPtr& object) const;
            FCLCollisionGeometryPtr createCollisionGeometry(const shape_msgs::msg::SolidPrimitive& solid) const;
            FCLCollisionGeometryPtr createCollisionGeometry(const shape_msgs::msg::Plane& plane) const;
            FCLCollisionGeometryPtr createCollisionGeometry(const shape_msgs::msg::Mesh& mesh) const;
            FCLCollisionGeometryPtr createCollisionGeometry(const octomap_msgs::msg::Octomap& map) const;
            
            // Add a vector of FCL collision objects to the world
            bool addCollisionObjects(const std::vector<FCLObjectPtr>& objects, const std::vector<int>& object_ids);
            // Add an FCL collision object with a defined id to the world
            bool addCollisionObject(const FCLObjectPtr& objecobjt, int object_id);

            // Delete multiple collision objects
            bool removeCollisionObjects(const std::vector<int>& object_ids);
            // Delete a collision object with object id
            bool removeCollisionObject(int object_id);

            // Check the collision between an FCL object and the known world
            // Returns true if in collision and a list of colliding objects
            bool checkCollisionObject(const FCLObjectPtr& obj, std::vector<int>& collision_object_ids) const;

            // Check the collision between an FCL object and a given world
            // Returns the number of collisions if any
            int checkCollisionObjectWorld(const FCLObjectPtr& obj, const std::vector<FCLObjectPtr>& world) const;

            // Check if two FCL objects are in collision
            bool checkCollisionObjects(const FCLObjectPtr& obj1, const FCLObjectPtr& obj2) const;

            /** Gets the distances from the FCL object to the known world
             * Returns:
             *    1. obj_distances a vector of distances (len = nbr of objects)
             *    2. closest_pt_object a vector of points (len = nbr of objects) on object closest to the world
             *    3. closest_pt_world a vector of points (len = nbr of objects) on world closest to the object */
            void getObjectDistances(const FCLObjectPtr& obj,
                                    std::vector<double>& obj_distances,
                                    std::vector<Eigen::Vector3d>& closest_pt_obj,
                                    std::vector<Eigen::Vector3d>& closest_pt_world) const;

            /** Gets the distances from the FCL object to a given world
             * Returns:
             *    1. obj_distances a vector of distances (len = nbr of objects)
             *    2. closest_pt_object a vector of points (len = nbr of objects) on object closest to the given world
             *    3. closest_pt_world a vector of points (len = nbr of objects) on given world closest to the object */
            void getObjectDistancesWorld(const FCLObjectPtr& obj,
                                         const std::vector<FCLObjectPtr>& world,
                                         std::vector<double>& obj_distances,
                                         std::vector<Eigen::Vector3d>& closest_pt_obj,
                                         std::vector<Eigen::Vector3d>& closest_pt_world) const;

            // Returns the minimum distance between two FCL objects
            double getDistanceObjects(const FCLObjectPtr& obj1, const FCLObjectPtr& obj2) const;
            
            /** Returns the minimum distance between two FCL objects
             * Also returns the position w.r.t to world frame of the closest points */
            double getDistanceObjects(const FCLObjectPtr& obj1,
                                      const FCLObjectPtr& obj2,
                                      Eigen::Vector3d& closest_pt_obj1,
                                      Eigen::Vector3d& closest_pt_obj2) const;

            // Get the minimum distance between an FCL object and the known world or a given world
            double getMinimumObjectDistance(const FCLObjectPtr& obj) const;
            double getMinimumObjectDistanceWorld(const FCLObjectPtr& obj, const std::vector<FCLObjectPtr>& world) const;

            // Type conversion methods
            void convertGeometryPoseEigenTransform(const geometry_msgs::msg::Pose& geo_pose, Eigen::Affine3d& wTt) const;
            void convertEigenTransformGeometryPose(const Eigen::Affine3d& wTt, geometry_msgs::msg::Pose& geo_pose) const;

            void convertGeometryPointEigenVector(const geometry_msgs::msg::Point& geo_point, Eigen::Vector3d& wPt) const;
            void convertEigenVectorGeometryPoint(const Eigen::Vector3d& wPt, geometry_msgs::msg::Point& geo_point) const;

            // Transforms an Eigen Affine3D to FCL coordinate transformation
            inline void transform2fcl(const Eigen::Affine3d& b, fcl::Transform3d& f) const
            {
                f = b.matrix();
            }

            inline int getNumObjects() const
            {
                return obj_counter_;
            }

        private:
            std::vector<std::unique_ptr<FCLInterfaceCollisionObject>> fcl_collision_world_;
            // Object counter ID
            unsigned int obj_counter_;
    };
}

#endif