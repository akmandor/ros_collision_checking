#ifndef FCL_INTERFACE_HPP
#define FCL_INTERFACE_HPP

#include <string>
#include <vector>

#include <fcl/fcl.h>

#include <Eigen/Eigen>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/plane.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include <robot_collision_checking/fcl_interface_types.hpp>

namespace robot_collision_checking
{
    inline rclcpp::Logger getLogger()
    {
        return rclcpp::get_logger("robot_collision_checking");
    }

    class FCLInterfaceCollisionWorld
    {
        public:
            FCLInterfaceCollisionWorld(std::string frame_id = "world");
            ~FCLInterfaceCollisionWorld();

            bool collisionObjectExists(int object_id) const;
            // If index is wanted
            bool collisionObjectExists(int object_id, int& index) const;

            // Add a vector of FCL collision objects to the world
            bool addCollisionObjects(const std::vector<FCLObjectPtr>& objects, const std::vector<int>& object_ids);
            // Add an FCL collision object with a defined id to the world
            bool addCollisionObject(const FCLObjectPtr& obj, int object_id);

            // Delete multiple collision objects
            bool removeCollisionObjects(const std::vector<int>& object_ids);
            // Delete a collision object with object id
            bool removeCollisionObject(int object_id);

            // Check the collision between an FCL object in the known world (with an ID or not)
            // Returns true if in collision and a list of colliding objects
            bool checkCollisionObject(int obj_id, std::vector<int>& collision_object_ids) const;
            bool checkCollisionObject(const FCLObjectPtr& obj, std::vector<int>& collision_object_ids) const;

            /** Gets the distances from an FCL object (with an ID or not) in the known world
             * Returns:
             *    1. obj_distances a vector of distances (len = nbr of objects)
             *    2. closest_pt_object a vector of points (len = nbr of objects) on object closest to the world
             *    3. closest_pt_world a vector of points (len = nbr of objects) on world closest to the object */
            void getObjectDistances(int obj_id,
                                    std::vector<double>& obj_distances,
                                    std::vector<Eigen::Vector3d>& closest_pt_obj,
                                    std::vector<Eigen::Vector3d>& closest_pt_world) const;
            void getObjectDistances(const FCLObjectPtr& obj,
                                    std::vector<double>& obj_distances,
                                    std::vector<Eigen::Vector3d>& closest_pt_obj,
                                    std::vector<Eigen::Vector3d>& closest_pt_world) const;

            // Get the minimum distance between an FCL object and the known world
            double getMinimumObjectDistance(int obj_id) const;
            double getMinimumObjectDistance(const FCLObjectPtr& obj) const;

            inline std::string getWorldFrame() const
            {
                return frame_;
            }

            inline const std::vector<FCLInterfaceCollisionObjectPtr>& getCollisionObjects() const
            {
                return fcl_collision_objects_;
            }

            inline int getNumObjects() const
            {
                return obj_counter_;
            }
        private:
            // Collision world frame. All shapes and subframes are defined relative to this frame.
            std::string frame_;
            std::vector<FCLInterfaceCollisionObjectPtr> fcl_collision_objects_;
            // Object counter ID
            unsigned int obj_counter_;
    };

    // Create collision FCL geometries from specific ROS msgs
    FCLCollisionGeometryPtr createCollisionGeometry(const FCLObjectPtr& obj);
    FCLCollisionGeometryPtr createCollisionGeometry(const shape_msgs::msg::SolidPrimitive& solid);
    FCLCollisionGeometryPtr createCollisionGeometry(const shape_msgs::msg::Plane& plane);
    FCLCollisionGeometryPtr createCollisionGeometry(const shape_msgs::msg::Mesh& mesh);
    FCLCollisionGeometryPtr createCollisionGeometry(const octomap_msgs::msg::Octomap& map);

    // Check the collision between an FCL object and a given world
    // Returns the number of collisions if any
    int checkCollisionObjectWorld(const FCLObjectPtr& obj, const FCLInterfaceCollisionWorld& world);

    // Check if two FCL objects are in collision
    bool checkCollisionObjects(const FCLObjectPtr& obj1, const FCLObjectPtr& obj2);

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

    // Returns the minimum distance between two FCL objects
    double getDistanceObjects(const FCLObjectPtr& obj1, const FCLObjectPtr& obj2);
    
    /** Returns the minimum distance between two FCL objects
     * Also returns the position w.r.t to world frame of the closest points */
    double getDistanceObjects(const FCLObjectPtr& obj1,
                              const FCLObjectPtr& obj2,
                              Eigen::Vector3d& closest_pt_obj1,
                              Eigen::Vector3d& closest_pt_obj2);

    // Get the minimum distance between an FCL object and a given world
    double getMinimumObjectDistanceWorld(const FCLObjectPtr& obj, const FCLInterfaceCollisionWorld& world);

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
}

#endif