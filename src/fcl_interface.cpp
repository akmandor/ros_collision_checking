#include <limits> // for numeric_limits

#include <robot_collision_checking/fcl_interface.hpp>

namespace robot_collision_checking
{
    FCLInterface::FCLInterface() : obj_counter_(0)
    {
        fcl_collision_world_.clear();
        RCLCPP_INFO(getLogger(), "Creating an FCL Interface");
    }

    FCLInterface::~FCLInterface()
    {
        RCLCPP_INFO(getLogger(), "Destroying world");
    }

    bool FCLInterface::collisionObjectExists(int object_id) const
    {
        // Iterate over the vector of collision objects
        for (const auto& collision_interface_obj : fcl_collision_world_)
        {
            if (collision_interface_obj->collision_id == object_id)
            {
                return true;
            }
        }

        return false;
    }

    FCLCollisionGeometryPtr FCLInterface::createCollisionGeometry(const FCLObjectPtr& object) const
    {
        if (object->object_type == VOXEL_GRID)
        {
            return nullptr;
        }

        switch (object->object_type)
        {
        case PLANE:
            return FCLInterface::createCollisionGeometry(*(object->ptr.plane));
        case MESH:
            return FCLInterface::createCollisionGeometry(*(object->ptr.mesh));
        case OCTOMAP:
            return FCLInterface::createCollisionGeometry(*(object->ptr.octomap));
        default:
            return FCLInterface::createCollisionGeometry(*(object->ptr.solid));
        }
    }

    FCLCollisionGeometryPtr FCLInterface::createCollisionGeometry(const shape_msgs::msg::SolidPrimitive& solid) const
    {
        if (solid.type == shape_msgs::msg::SolidPrimitive::SPHERE)
        {
            return std::make_shared<fcl::Sphered>(solid.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS]);
        }
        else if (solid.type == shape_msgs::msg::SolidPrimitive::BOX)
        {
            return std::make_shared<fcl::Boxd>(solid.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X],
                                               solid.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y],
                                               solid.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z]);
        }
        else if (solid.type == shape_msgs::msg::SolidPrimitive::CONE)
        {
            return std::make_shared<fcl::Coned>(solid.dimensions[shape_msgs::msg::SolidPrimitive::CONE_RADIUS],
                                                solid.dimensions[shape_msgs::msg::SolidPrimitive::CONE_HEIGHT]);
        }
        else if (solid.type == shape_msgs::msg::SolidPrimitive::CYLINDER)
        {
            return std::make_shared<fcl::Cylinderd>(solid.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS],
                                                    solid.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT]);
        }
        else
        {
            RCLCPP_ERROR(getLogger(), "NOT a valid solid primitive type");
            return nullptr;
        }
    }

    // Plane is defined as ax + by + cz + d = 0;
    FCLCollisionGeometryPtr FCLInterface::createCollisionGeometry(const shape_msgs::msg::Plane& plane) const
    {
        return FCLCollisionGeometryPtr(
            new fcl::Planed(plane.coef[0], plane.coef[1], plane.coef[2], plane.coef[3]));
    }

    FCLCollisionGeometryPtr FCLInterface::createCollisionGeometry(const shape_msgs::msg::Mesh& mesh) const
    {
        std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> g(new fcl::BVHModel<fcl::OBBRSSd>());

        unsigned int vert_size = mesh.vertices.size();
        unsigned int tri_size = mesh.triangles.size();
        if (vert_size > 0 && tri_size > 0)
        {
            std::vector<fcl::Triangle> tri_indices(tri_size);
            std::vector<fcl::Vector3d> points(vert_size);

            for (unsigned int i = 0; i < tri_size; ++i)
            {
                tri_indices[i] =
                    fcl::Triangle(mesh.triangles[i].vertex_indices[0],
                                  mesh.triangles[i].vertex_indices[1],
                                  mesh.triangles[i].vertex_indices[2]);
            }

            for (unsigned int i = 0; i < vert_size; ++i)
            {
                points[i] = fcl::Vector3d(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z);
            }

            g->beginModel();
            g->addSubModel(points, tri_indices);
            g->endModel();
        }

        return g;
    }

    FCLCollisionGeometryPtr FCLInterface::createCollisionGeometry(const octomap_msgs::msg::Octomap& map) const
    {
        std::shared_ptr<octomap::OcTree> tree(static_cast<octomap::OcTree *>(octomap_msgs::msgToMap(map)));
        return FCLCollisionGeometryPtr(new fcl::OcTreed(tree));
    }

    bool FCLInterface::addCollisionObjects(const std::vector<FCLObjectPtr>& objects, const std::vector<int>& object_ids)
    {
        int num_objects = objects.size();
        // Check same number of objects and IDs
        if (num_objects != object_ids.size())
        {
            return false;
        }

        for (int i = 0; i < num_objects; i++)
        {
            addCollisionObject(objects[i], object_ids[i]);
        }

        return true;
    }

    bool FCLInterface::addCollisionObject(const FCLObjectPtr& obj, int object_id)
    {
        if (collisionObjectExists(object_id))
        {
            return false;
        }

        if (obj->getTypeString() != "VOXEL_GRID")
        {
            FCLCollisionGeometryPtr cg = FCLInterface::createCollisionGeometry(obj);
            fcl::Transform3d world_to_fcl;
            FCLInterface::transform2fcl(obj->object_transform, world_to_fcl);
            FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(cg, world_to_fcl);

            FCLInterfaceCollisionObject *new_col_object(new FCLInterfaceCollisionObject(obj, co, object_id));
            fcl_collision_world_.push_back(std::unique_ptr<FCLInterfaceCollisionObject>(new_col_object));
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
                            FCLInterface::transform2fcl(world_to_cell, world_to_fcl);

                            // Make collision geometry
                            Eigen::Vector3d side(x_res, y_res, z_res);
                            FCLCollisionGeometryPtr cg = std::make_shared<fcl::Boxd>(side);
                            FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(cg, world_to_fcl);

                            FCLInterfaceCollisionObject *new_col_object(new FCLInterfaceCollisionObject(obj, co, object_id));
                            fcl_collision_world_.push_back(std::unique_ptr<FCLInterfaceCollisionObject>(new_col_object));
                            obj_counter_++;
                        }
                    }
                }
            }
        }
        
        return true;
    }

    bool FCLInterface::removeCollisionObjects(const std::vector<int>& object_ids)
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

    bool FCLInterface::removeCollisionObject(int object_id)
    {
        int removed_objects = 0;
        for (auto it = fcl_collision_world_.begin(); it != fcl_collision_world_.end(); /*++it*/)
        {
            // Check if the object's ID matches the specified ID
            if ((*it)->collision_id == object_id)
            {
                // Erase the object from the vector
                it = fcl_collision_world_.erase(it);
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

    bool FCLInterface::checkCollisionObject(const FCLObjectPtr& obj, std::vector<int>& collision_object_ids) const
    {
        FCLCollisionGeometryPtr cg = FCLInterface::createCollisionGeometry(obj);
        fcl::Transform3d world_to_fcl;
        FCLInterface::transform2fcl(obj->object_transform, world_to_fcl);
        FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(cg, world_to_fcl);

        // Clear vector of any prior entries
        collision_object_ids.clear();

        fcl::CollisionRequestd col_req;
        fcl::CollisionResultd col_result;
        bool in_collision(false);
        for (const auto& fcl_coll_obj : fcl_collision_world_)
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

    int FCLInterface::checkCollisionObjectWorld(const FCLObjectPtr& obj, const std::vector<FCLObjectPtr>& world) const
    {
        FCLCollisionGeometryPtr cg = FCLInterface::createCollisionGeometry(obj);
        fcl::Transform3d world_to_fcl;
        FCLInterface::transform2fcl(obj->object_transform, world_to_fcl);
        FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(cg, world_to_fcl);

        fcl::CollisionRequestd col_req;
        fcl::CollisionResultd col_result;
        int num_contacts(0);
        int num_objects = world.size();
        for (const auto& other : world)
        {
            if (checkCollisionObjects(obj, other))
            {
                num_contacts++;
            }
        }

        return num_contacts;
    }

    bool FCLInterface::checkCollisionObjects(const FCLObjectPtr& obj1, const FCLObjectPtr& obj2) const
    {
        // Create the collision geometries
        FCLCollisionGeometryPtr cg1 = FCLInterface::createCollisionGeometry(obj1);
        FCLCollisionGeometryPtr cg2 = FCLInterface::createCollisionGeometry(obj2);
        // Convert to FCL coordinates
        fcl::Transform3d world_to_fcl1, world_to_fcl2;
        FCLInterface::transform2fcl(obj1->object_transform, world_to_fcl1);
        FCLInterface::transform2fcl(obj2->object_transform, world_to_fcl2);
        // Construct FCLCollisionObjects
        FCLCollisionObjectPtr co1 = std::make_shared<fcl::CollisionObjectd>(cg1, world_to_fcl1);
        FCLCollisionObjectPtr co2 = std::make_shared<fcl::CollisionObjectd>(cg2, world_to_fcl2);
        // Use FCL to check for collision
        fcl::CollisionRequestd col_req;
        fcl::CollisionResultd col_result;
        fcl::collide(co1.get(), co2.get(), col_req, col_result);
        return col_result.isCollision();
    }

    void FCLInterface::getObjectDistances(const FCLObjectPtr& obj,
                                          std::vector<double>& obj_distances,
                                          std::vector<Eigen::Vector3d>& closest_pt_obj,
                                          std::vector<Eigen::Vector3d>& closest_pt_world) const
    {
        // Reset output vector variables
        obj_distances.clear();
        obj_distances.resize(obj_counter_);
        closest_pt_obj.clear();
        closest_pt_obj.resize(obj_counter_);
        closest_pt_world.clear();
        closest_pt_world.resize(obj_counter_);

        // Create the collision object
        FCLCollisionGeometryPtr cg = FCLInterface::createCollisionGeometry(obj);
        fcl::Transform3d world_to_fcl;
        FCLInterface::transform2fcl(obj->object_transform, world_to_fcl);

        FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(cg, world_to_fcl);

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
                          fcl_collision_world_[i]->collision_object.get(),
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

    void FCLInterface::getObjectDistancesWorld(const FCLObjectPtr& obj,
                                               const std::vector<FCLObjectPtr>& world,
                                               std::vector<double>& obj_distances,
                                               std::vector<Eigen::Vector3d>& closest_pt_obj,
                                               std::vector<Eigen::Vector3d>& closest_pt_world) const
    {
        // Reset output vector variables
        unsigned int world_size = world.size();
        obj_distances.clear();
        obj_distances.resize(world_size);
        closest_pt_obj.clear();
        closest_pt_obj.resize(world_size);
        closest_pt_world.clear();
        closest_pt_world.resize(world_size);
        
        for (unsigned int i = 0; i < world_size; i++)
        {
            obj_distances[i] = getDistanceObjects(obj, world[i], closest_pt_obj[i], closest_pt_world[i]);
        }
    }

    double FCLInterface::getDistanceObjects(const FCLObjectPtr& obj1,
                                            const FCLObjectPtr& obj2) const
    {
        // Create the collision geometries
        FCLCollisionGeometryPtr cg1 = FCLInterface::createCollisionGeometry(obj1);
        FCLCollisionGeometryPtr cg2 = FCLInterface::createCollisionGeometry(obj2);
        // Convert to FCL coordinates
        fcl::Transform3d world_to_fcl1, world_to_fcl2;
        FCLInterface::transform2fcl(obj1->object_transform, world_to_fcl1);
        FCLInterface::transform2fcl(obj2->object_transform, world_to_fcl2);
        // Construct FCLCollisionObjects
        FCLCollisionObjectPtr co1 = std::make_shared<fcl::CollisionObjectd>(cg1, world_to_fcl1);
        FCLCollisionObjectPtr co2 = std::make_shared<fcl::CollisionObjectd>(cg2, world_to_fcl2);

        fcl::DistanceRequestd dist_req;
        dist_req.enable_nearest_points = false;
        dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
        fcl::DistanceResultd dist_result;

        fcl::distance(co1.get(), co2.get(), dist_req, dist_result);

        return dist_result.min_distance;
    }

    double FCLInterface::getDistanceObjects(const FCLObjectPtr& obj1,
                                            const FCLObjectPtr& obj2,
                                            Eigen::Vector3d& closest_pt_obj1,
                                            Eigen::Vector3d& closest_pt_obj2) const
    {
        // Create the collision geometries
        FCLCollisionGeometryPtr cg1 = FCLInterface::createCollisionGeometry(obj1);
        FCLCollisionGeometryPtr cg2 = FCLInterface::createCollisionGeometry(obj2);
        // Convert to FCL coordinates
        fcl::Transform3d world_to_fcl1, world_to_fcl2;
        FCLInterface::transform2fcl(obj1->object_transform, world_to_fcl1);
        FCLInterface::transform2fcl(obj2->object_transform, world_to_fcl2);
        // Construct FCLCollisionObjects
        FCLCollisionObjectPtr co1 = std::make_shared<fcl::CollisionObjectd>(cg1, world_to_fcl1);
        FCLCollisionObjectPtr co2 = std::make_shared<fcl::CollisionObjectd>(cg2, world_to_fcl2);

        fcl::DistanceRequestd dist_req;
        dist_req.enable_nearest_points = true;
        dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
        fcl::DistanceResultd dist_result;

        dist_result.nearest_points[0].setZero();
        dist_result.nearest_points[1].setZero();
        fcl::distance(co1.get(), co2.get(), dist_req, dist_result);

        for (int i = 0; i < 3; i++)
        {
            closest_pt_obj1(i) = dist_result.nearest_points[0][i];
            closest_pt_obj2(i) = dist_result.nearest_points[1][i];
        }

        return dist_result.min_distance;
    }

    double FCLInterface::getMinimumObjectDistance(const FCLObjectPtr& obj) const
    {
        FCLCollisionGeometryPtr cg = FCLInterface::createCollisionGeometry(obj);
        fcl::Transform3d world_to_fcl;
        FCLInterface::transform2fcl(obj->object_transform, world_to_fcl);
        FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(cg, world_to_fcl);

        double min_distance = std::numeric_limits<double>::max();
        for (const auto& collision_interface_obj : fcl_collision_world_)
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

    double FCLInterface::getMinimumObjectDistanceWorld(const FCLObjectPtr& obj, const std::vector<FCLObjectPtr>& world) const
    {
        double min_distance = std::numeric_limits<double>::max();
        for (const auto& other : world)
        {
            double dist_objs = getDistanceObjects(obj, other);
            if (min_distance > dist_objs)
            {
                min_distance = dist_objs;
            }
        }

        return min_distance;
    }

    void FCLInterface::convertGeometryPoseEigenTransform(const geometry_msgs::msg::Pose& geo_pose, Eigen::Affine3d& wTt) const
    {
        Eigen::Vector3d t(geo_pose.position.x, geo_pose.position.y, geo_pose.position.z);
        Eigen::Quaterniond q(geo_pose.orientation.w,
                             geo_pose.orientation.x,
                             geo_pose.orientation.y,
                             geo_pose.orientation.z);
        wTt.translation() = t;
        wTt.linear() = q.toRotationMatrix();
    }

    void FCLInterface::convertEigenTransformGeometryPose(const Eigen::Affine3d& wTt, geometry_msgs::msg::Pose& geo_pose) const
    {

        Eigen::Quaterniond q(wTt.linear());
        q.normalize();
        Eigen::Vector3d t(wTt.translation());

        geo_pose.position.x = t(0);
        geo_pose.position.y = t(1);
        geo_pose.position.z = t(2);
        geo_pose.orientation.x = q.x();
        geo_pose.orientation.y = q.y();
        geo_pose.orientation.z = q.z();
        geo_pose.orientation.w = q.w();
    }

    void FCLInterface::convertGeometryPointEigenVector(const geometry_msgs::msg::Point& geo_point, Eigen::Vector3d& wPt) const
    {
        wPt.setZero();
        wPt(0) = geo_point.x;
        wPt(1) = geo_point.y;
        wPt(2) = geo_point.z;
    }

    void FCLInterface::convertEigenVectorGeometryPoint(const Eigen::Vector3d& wPt, geometry_msgs::msg::Point& geo_point) const
    {
        geo_point.x = wPt(0);
        geo_point.y = wPt(1);
        geo_point.z = wPt(2);
    }
}