#pragma once

#include <string>
#include <vector>

#include <Eigen/Eigen>

#include "robot_collision_checking/fcl_interface_types.hpp"

namespace robot_collision_checking
{
class FCLInterfaceCollisionWorld
{
    public:
        FCLInterfaceCollisionWorld(const std::string& frame_id = "world");
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
        bool checkCollisionObject(const FCLCollisionObjectPtr& co, std::vector<int>& collision_object_ids) const;

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
        void getObjectDistances(const FCLCollisionObjectPtr& co,
                                std::vector<double>& obj_distances,
                                std::vector<Eigen::Vector3d>& closest_pt_obj,
                                std::vector<Eigen::Vector3d>& closest_pt_world) const;

        // Get the minimum distance between an FCL object and the known world
        double getMinimumObjectDistance(int obj_id) const;
        double getMinimumObjectDistance(const FCLObjectPtr& obj) const;
        double getMinimumObjectDistance(const FCLCollisionObjectPtr& co) const;

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
} // namespace robot_collision_checking