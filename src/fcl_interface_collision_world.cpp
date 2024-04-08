#include <limits> // for numeric_limits

#include <robot_collision_checking/fcl_interface.hpp>

namespace robot_collision_checking
{
    FCLInterfaceCollisionWorld::FCLInterfaceCollisionWorld(std::string frame_id) : frame_(frame_id), obj_counter_(0)
    {
        fcl_collision_objects_.clear();
        RCLCPP_INFO(getLogger(), "Creating an FCL Interface");
    }

    FCLInterfaceCollisionWorld::~FCLInterfaceCollisionWorld()
    {
        RCLCPP_INFO(getLogger(), "Destroying world");
    }

    bool FCLInterfaceCollisionWorld::collisionObjectExists(int object_id) const
    {
        // Iterate over the vector of collision objects
        for (const auto& collision_interface_obj : fcl_collision_objects_)
        {
            if (collision_interface_obj->collision_id == object_id)
            {
                return true;
            }
        }

        return false;
    }
    
    bool FCLInterfaceCollisionWorld::collisionObjectExists(int object_id, int& index) const
    {
        // Iterate over the vector of collision objects
        index = -1;
        for (unsigned int i = 0; i < obj_counter_; i++)
        {
            if (fcl_collision_objects_[i]->collision_id == object_id)
            {
                index = i;
                return true;
            }
        }

        return false;
    }

    bool FCLInterfaceCollisionWorld::addCollisionObjects(const std::vector<FCLObjectPtr>& objects, const std::vector<int>& object_ids)
    {
        // Check same number of objects and IDs
        if (objects.size() != object_ids.size())
        {
            RCLCPP_ERROR(getLogger(), "Not the same # of objects and associated IDs");
            return false;
        }

        int num_objects = objects.size();
        for (int i = 0; i < num_objects; i++)
        {
            addCollisionObject(objects[i], object_ids[i]);
        }

        return true;
    }

    bool FCLInterfaceCollisionWorld::addCollisionObject(const FCLObjectPtr& obj, int object_id)
    {
        if (collisionObjectExists(object_id))
        {
            RCLCPP_INFO(getLogger(), "Object with ID %d already exists!", object_id);
            return false;
        }

        if (obj->getTypeString() != "VOXEL_GRID")
        {
            FCLCollisionGeometryPtr cg = createCollisionGeometry(obj);
            fcl::Transform3d world_to_fcl;
            transform2fcl(obj->object_transform, world_to_fcl);
            FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(cg, world_to_fcl);

            FCLInterfaceCollisionObjectPtr new_col_object = std::make_shared<FCLInterfaceCollisionObject>(obj, co, object_id);
            fcl_collision_objects_.push_back(new_col_object);
            obj_counter_++;
        }
        else
        {
            auto grid = *(obj->ptr.voxel_grid);
            const uint32_t *data = &grid.data.front();
            const double x_origin = grid.origin.x;
            const double y_origin = grid.origin.y;
            const double z_origin = grid.origin.z;
            const double x_res = grid.resolutions.x;
            const double y_res = grid.resolutions.y;
            const double z_res = grid.resolutions.z;
            const uint32_t x_size = grid.size_x;
            const uint32_t y_size = grid.size_y;
            const uint32_t z_size = grid.size_z;
            const Eigen::Affine3d& object_transform = obj->object_transform;

            for (uint32_t x_grid = 0; x_grid < x_size; ++x_grid)
            {
                for (uint32_t y_grid = 0; y_grid < y_size; ++y_grid)
                {
                    for (uint32_t z_grid = 0; z_grid < z_size; ++z_grid)
                    {
                        nav2_voxel_grid::VoxelStatus status = nav2_voxel_grid::VoxelGrid::getVoxel(x_grid, y_grid, z_grid,
                                                                                                   x_size, y_size, z_size, data);
                        if (status == nav2_voxel_grid::MARKED)
                        {
                            // Center point of the cell
                            double x = x_origin + (x_grid + 0.5) * x_res;
                            double y = y_origin + (y_grid + 0.5) * y_res;
                            double z = z_origin + (z_grid + 0.5) * z_res;
                            Eigen::Vector3d center(x, y, z);

                            // Translate to origin of cell
                            Eigen::Affine3d world_to_cell = object_transform;
                            world_to_cell.translate(center);

                            fcl::Transform3d world_to_fcl;
                            transform2fcl(world_to_cell, world_to_fcl);

                            // Make collision geometry
                            Eigen::Vector3d side(x_res, y_res, z_res);
                            FCLCollisionGeometryPtr cg = std::make_shared<fcl::Boxd>(side);
                            FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(cg, world_to_fcl);

                            FCLInterfaceCollisionObjectPtr new_col_object = std::make_shared<FCLInterfaceCollisionObject>(obj, co, object_id);
                            fcl_collision_objects_.push_back(new_col_object);
                            obj_counter_++;
                        }
                    }
                }
            }
        }
        
        return true;
    }

    bool FCLInterfaceCollisionWorld::removeCollisionObjects(const std::vector<int>& object_ids)
    {
        bool all_removed = true; // Assume all objects will be removed initially
        for (int id : object_ids)
        {
            // If any object removal fails, set all_removed to false
            if (!removeCollisionObject(id)) 
            {
                all_removed = false;
            }
        }

        return all_removed;
    }

    bool FCLInterfaceCollisionWorld::removeCollisionObject(int object_id)
    {
        int removed_objects = 0;
        for (auto it = fcl_collision_objects_.begin(); it != fcl_collision_objects_.end(); /*++it*/)
        {
            // Check if the object's ID matches the specified ID
            if ((*it)->collision_id == object_id)
            {
                // Erase the object from the vector
                it = fcl_collision_objects_.erase(it);
                obj_counter_--;
                removed_objects++;
            }
            else
            {
                // Iterator in else handles voxel grid's cells with the same object ID
                ++it;
            }
        }

        return removed_objects > 0;
    }

    bool FCLInterfaceCollisionWorld::checkCollisionObject(int obj_id, std::vector<int>& collision_object_ids) const
    {
        // Get the collision object
        int index;
        if (!collisionObjectExists(obj_id, index))
        {
            RCLCPP_ERROR(getLogger(), "No object with ID %d in the world", obj_id);
        }
        FCLCollisionObjectPtr co = fcl_collision_objects_[index]->collision_object;
        
        // Clear vector of any prior entries
        collision_object_ids.clear();

        bool in_collision(false);
        for (unsigned int i = 0; i < obj_counter_; i++)
        {
            if (fcl_collision_objects_[i]->collision_id != index)
            {
                fcl::CollisionRequestd col_req;
                fcl::CollisionResultd col_result;
                fcl::collide(co.get(),
                            fcl_collision_objects_[i]->collision_object.get(),
                            col_req,
                            col_result);
                if (col_result.isCollision())
                {
                    collision_object_ids.push_back(fcl_collision_objects_[i]->collision_id);
                    in_collision = true;
                }
            }
        }

        return in_collision;
    }

    bool FCLInterfaceCollisionWorld::checkCollisionObject(const FCLObjectPtr& obj, std::vector<int>& collision_object_ids) const
    {
        // Create the collision object
        FCLCollisionGeometryPtr cg = createCollisionGeometry(obj);
        fcl::Transform3d world_to_fcl;
        transform2fcl(obj->object_transform, world_to_fcl);
        FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(cg, world_to_fcl);

        // Clear vector of any prior entries
        collision_object_ids.clear();

        fcl::CollisionRequestd col_req;
        fcl::CollisionResultd col_result;
        bool in_collision(false);
        for (const auto& fcl_coll_obj : fcl_collision_objects_)
        {
            fcl::collide(co.get(),
                         fcl_coll_obj->collision_object.get(),
                         col_req,
                         col_result);
            if (col_result.isCollision())
            {
                collision_object_ids.push_back(fcl_coll_obj->collision_id);
                in_collision = true;
            }
        }

        return in_collision;
    }

    void FCLInterfaceCollisionWorld::getObjectDistances(int obj_id,
                                                        std::vector<double>& obj_distances,
                                                        std::vector<Eigen::Vector3d>& closest_pt_obj,
                                                        std::vector<Eigen::Vector3d>& closest_pt_world) const
    {
        // Get the collision object
        int index;
        if (!collisionObjectExists(obj_id, index))
        {
            RCLCPP_ERROR(getLogger(), "No object with ID %d in the world", obj_id);
        }
        FCLCollisionObjectPtr co = fcl_collision_objects_[index]->collision_object;

        // Reset output vector variables
        obj_distances.clear();
        obj_distances.resize(obj_counter_);
        closest_pt_obj.clear();
        closest_pt_obj.resize(obj_counter_);
        closest_pt_world.clear();
        closest_pt_world.resize(obj_counter_);

        // Compute distances over all objects in world
        for (unsigned int i = 0; i < obj_counter_; i++)
        {
            if (fcl_collision_objects_[i]->collision_id != index)
            {
                fcl::DistanceRequestd dist_req;
                dist_req.enable_nearest_points = true;
                dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

                fcl::DistanceResultd dist_result;
                dist_result.nearest_points[0].setZero();
                dist_result.nearest_points[1].setZero();

                fcl::distance(co.get(),
                            fcl_collision_objects_[i]->collision_object.get(),
                            dist_req,
                            dist_result);

                // Iterate over 3 coordinate axes
                for (int j = 0; j < 3; j++)
                {
                    closest_pt_obj[i](j) = dist_result.nearest_points[0][j];
                    closest_pt_world[i](j) = dist_result.nearest_points[1][j];
                }

                obj_distances[i] = dist_result.min_distance;
            }
        }
    }
 
     void FCLInterfaceCollisionWorld::getObjectDistances(const FCLObjectPtr& obj,
                                                         std::vector<double>& obj_distances,
                                                         std::vector<Eigen::Vector3d>& closest_pt_obj,
                                                         std::vector<Eigen::Vector3d>& closest_pt_world) const
    {
        // Create the collision object
        FCLCollisionGeometryPtr cg = createCollisionGeometry(obj);
        fcl::Transform3d world_to_fcl;
        transform2fcl(obj->object_transform, world_to_fcl);
        FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(cg, world_to_fcl);

        // Reset output vector variables
        obj_distances.clear();
        obj_distances.resize(obj_counter_);
        closest_pt_obj.clear();
        closest_pt_obj.resize(obj_counter_);
        closest_pt_world.clear();
        closest_pt_world.resize(obj_counter_);

        // Compute distances over all objects in world
        for (unsigned int i = 0; i < obj_counter_; i++)
        {
            fcl::DistanceRequestd dist_req;
            dist_req.enable_nearest_points = true;
            dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

            fcl::DistanceResultd dist_result;
            dist_result.nearest_points[0].setZero();
            dist_result.nearest_points[1].setZero();

            fcl::distance(co.get(),
                          fcl_collision_objects_[i]->collision_object.get(),
                          dist_req,
                          dist_result);

            // Iterate over 3 coordinate axes
            for (int j = 0; j < 3; j++)
            {
                closest_pt_obj[i](j) = dist_result.nearest_points[0][j];
                closest_pt_world[i](j) = dist_result.nearest_points[1][j];
            }

            obj_distances[i] = dist_result.min_distance;
        }
    }

    double FCLInterfaceCollisionWorld::getMinimumObjectDistance(int obj_id) const
    {
        // Get the collision object
        int index;
        if (!collisionObjectExists(obj_id, index))
        {
            RCLCPP_ERROR(getLogger(), "No object with ID %d in the world", obj_id);
        }
        FCLCollisionObjectPtr co = fcl_collision_objects_[index]->collision_object;

        double min_distance = std::numeric_limits<double>::max();
        for (unsigned int i = 0; i < obj_counter_; i++)
        {
            if (fcl_collision_objects_[i]->collision_id != index)
            {
                fcl::DistanceRequestd dist_req;
                dist_req.enable_nearest_points = false;
                dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
                fcl::DistanceResultd dist_result;

                fcl::distance(co.get(),
                              fcl_collision_objects_[i]->collision_object.get(),
                              dist_req,
                              dist_result);

                if (min_distance > dist_result.min_distance)
                {
                    min_distance = dist_result.min_distance;
                }
            }
        }

        return min_distance;
    }

    double FCLInterfaceCollisionWorld::getMinimumObjectDistance(const FCLObjectPtr& obj) const
    {
        // Create the collision object
        FCLCollisionGeometryPtr cg = createCollisionGeometry(obj);
        fcl::Transform3d world_to_fcl;
        transform2fcl(obj->object_transform, world_to_fcl);
        FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(cg, world_to_fcl);

        double min_distance = std::numeric_limits<double>::max();
        for (const auto& collision_interface_obj : fcl_collision_objects_)
        {
            fcl::DistanceRequestd dist_req;
            dist_req.enable_nearest_points = false;
            dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
            fcl::DistanceResultd dist_result;

            fcl::distance(co.get(),
                          collision_interface_obj->collision_object.get(),
                          dist_req,
                          dist_result);

            if (min_distance > dist_result.min_distance)
            {
                min_distance = dist_result.min_distance;
            }
        }

        return min_distance;
    }
}