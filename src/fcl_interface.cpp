#include <limits> // for numeric_limits

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "robot_collision_checking/fcl_interface.hpp"

namespace robot_collision_checking
{
namespace fcl_interface
{
namespace
{
    rclcpp::Logger getLogger()
    {
        return rclcpp::get_logger("robot_collision_checking.fcl_interface");
    }
} // namespace

FCLCollisionGeometryPtr createCollisionGeometry(const FCLObjectPtr& obj)
{
    if (obj == nullptr)
    {
        RCLCPP_ERROR(getLogger(), "createCollisionGeometry: called with a NULL FCLObjectPtr.");
        return nullptr;
    }

    // TODO: Find a better representation for voxel grid collision geometry
    if (obj->object_type == VOXEL_GRID)
    {
        RCLCPP_ERROR(getLogger(), "createCollisionGeometry: cannot create collision geometry "
                                   "for voxel grid. Need to break the grid up into cells.");
        return nullptr;
    }

    switch (obj->object_type)
    {
    case PLANE:
        return createCollisionGeometry(*(obj->ptr.plane));
    case MESH:
        return createCollisionGeometry(*(obj->ptr.mesh));
    case OCTOMAP:
        return createCollisionGeometry(*(obj->ptr.octomap));
    default:
        return createCollisionGeometry(*(obj->ptr.solid));
    }
}

FCLCollisionGeometryPtr createCollisionGeometry(const shape_msgs::msg::SolidPrimitive& solid)
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
FCLCollisionGeometryPtr createCollisionGeometry(const shape_msgs::msg::Plane& plane)
{
    return FCLCollisionGeometryPtr(
        new fcl::Planed(plane.coef[0], plane.coef[1], plane.coef[2], plane.coef[3]));
}

FCLCollisionGeometryPtr createCollisionGeometry(const shape_msgs::msg::Mesh& mesh)
{
    auto g = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();

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

        g->beginModel(tri_size, vert_size);
        g->addSubModel(points, tri_indices);
        g->endModel();
    }

    return g;
}

FCLCollisionGeometryPtr createCollisionGeometry(const octomap_msgs::msg::Octomap& map)
{
    std::shared_ptr<octomap::OcTree> tree(static_cast<octomap::OcTree *>(octomap_msgs::msgToMap(map)));
    return FCLCollisionGeometryPtr(new fcl::OcTreed(tree));
}

int checkCollisionObjectWorld(const FCLObjectPtr& obj, const FCLInterfaceCollisionWorld& world)
{
    if (obj == nullptr)
    {
        RCLCPP_ERROR(getLogger(), "checkCollisionObjectWorld: called with a NULL FCLObjectPtr.");
        return -1;
    }

    FCLCollisionGeometryPtr cg = createCollisionGeometry(obj);
    fcl::Transform3d world_to_fcl;
    transform2fcl(obj->object_transform, world_to_fcl);
    FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(cg, world_to_fcl);

    fcl::CollisionRequestd col_req;
    fcl::CollisionResultd col_result;
    std::vector<FCLInterfaceCollisionObjectPtr> world_objects = world.getCollisionObjects();
    int num_contacts(0);
    for (const auto& other : world_objects)
    {
        if (checkCollisionObjects(obj, other->object))
        {
            num_contacts++;
        }
    }

    return num_contacts;
}

int checkCollisionObjectWorld(const FCLCollisionObjectPtr& co, const FCLInterfaceCollisionWorld& world)
{
    if (co == nullptr)
    {
        RCLCPP_ERROR(getLogger(), "checkCollisionObjectWorld: called with a NULL FCLCollisionObjectPtr.");
        return -1;
    }

    fcl::CollisionRequestd col_req;
    fcl::CollisionResultd col_result;
    std::vector<FCLInterfaceCollisionObjectPtr> world_objects = world.getCollisionObjects();
    int num_contacts(0);
    for (const auto& other : world_objects)
    {
        if (checkCollisionObjects(co, other->collision_object))
        {
            num_contacts++;
        }
    }

    return num_contacts;
}

bool checkCollisionObjects(const FCLObjectPtr& obj1, const FCLObjectPtr& obj2)
{
    if (obj1 == nullptr || obj2 == nullptr)
    {
        RCLCPP_ERROR(getLogger(), "checkCollisionObjects: called with a NULL FCLObjectPtr.");
        return false;
    }

    // Create the collision geometries
    FCLCollisionGeometryPtr cg1 = createCollisionGeometry(obj1);
    FCLCollisionGeometryPtr cg2 = createCollisionGeometry(obj2);
    // Convert to FCL coordinates
    fcl::Transform3d world_to_fcl1, world_to_fcl2;
    transform2fcl(obj1->object_transform, world_to_fcl1);
    transform2fcl(obj2->object_transform, world_to_fcl2);
    // Construct FCLCollisionObjects
    FCLCollisionObjectPtr co1 = std::make_shared<fcl::CollisionObjectd>(cg1, world_to_fcl1);
    FCLCollisionObjectPtr co2 = std::make_shared<fcl::CollisionObjectd>(cg2, world_to_fcl2);
    // Use FCL to check for collision
    fcl::CollisionRequestd col_req;
    fcl::CollisionResultd col_result;
    fcl::collide(co1.get(), co2.get(), col_req, col_result);
    return col_result.isCollision();
}

bool checkCollisionObjects(const FCLCollisionObjectPtr& co1, const FCLCollisionObjectPtr& co2)
{
    if (co1 == nullptr || co2 == nullptr)
    {
        RCLCPP_ERROR(getLogger(), "checkCollisionObjects: called with a NULL FCLCollisionObjectPtr.");
        return false;
    }

    // Use FCL to check for collision
    fcl::CollisionRequestd col_req;
    fcl::CollisionResultd col_result;
    fcl::collide(co1.get(), co2.get(), col_req, col_result);
    return col_result.isCollision();
}

void getObjectDistancesWorld(const FCLObjectPtr& obj,
                             const FCLInterfaceCollisionWorld& world,
                             std::vector<double>& obj_distances,
                             std::vector<Eigen::Vector3d>& closest_pt_obj,
                             std::vector<Eigen::Vector3d>& closest_pt_world)
{
    if (obj == nullptr)
    {
        RCLCPP_ERROR(getLogger(), "getObjectDistancesWorld: called with a NULL FCLObjectPtr.");
        return;
    }

    // Reset output vector variables
    int world_size = world.getNumObjects();
    obj_distances.clear();
    obj_distances.resize(world_size);
    closest_pt_obj.clear();
    closest_pt_obj.resize(world_size);
    closest_pt_world.clear();
    closest_pt_world.resize(world_size);
    
    std::vector<FCLInterfaceCollisionObjectPtr> world_objects = world.getCollisionObjects();
    for (int i = 0; i < world_size; i++)
    {
        obj_distances[i] = getDistanceObjects(obj, world_objects[i]->object, closest_pt_obj[i], closest_pt_world[i]);
    }
}

void getObjectDistancesWorld(const FCLCollisionObjectPtr& co,
                             const FCLInterfaceCollisionWorld& world,
                             std::vector<double>& obj_distances,
                             std::vector<Eigen::Vector3d>& closest_pt_obj,
                             std::vector<Eigen::Vector3d>& closest_pt_world)
{
    if (co == nullptr)
    {
        RCLCPP_ERROR(getLogger(), "getObjectDistancesWorld: called with a NULL FCLCollisionObjectPtr.");
        return;
    }

    // Reset output vector variables
    int world_size = world.getNumObjects();
    obj_distances.clear();
    obj_distances.resize(world_size);
    closest_pt_obj.clear();
    closest_pt_obj.resize(world_size);
    closest_pt_world.clear();
    closest_pt_world.resize(world_size);
    
    std::vector<FCLInterfaceCollisionObjectPtr> world_objects = world.getCollisionObjects();
    for (int i = 0; i < world_size; i++)
    {
        obj_distances[i] = getDistanceObjects(co, world_objects[i]->collision_object, closest_pt_obj[i], closest_pt_world[i]);
    }
}

double getDistanceObjects(const FCLObjectPtr& obj1, const FCLObjectPtr& obj2)
{
    if (obj1 == nullptr || obj2 == nullptr)
    {
        RCLCPP_ERROR(getLogger(), "getDistanceObjects: called with a NULL FCLObjectPtr.");
        return -1.0;
    }

    // Create the collision geometries
    FCLCollisionGeometryPtr cg1 = createCollisionGeometry(obj1);
    FCLCollisionGeometryPtr cg2 = createCollisionGeometry(obj2);
    // Convert to FCL coordinates
    fcl::Transform3d world_to_fcl1, world_to_fcl2;
    transform2fcl(obj1->object_transform, world_to_fcl1);
    transform2fcl(obj2->object_transform, world_to_fcl2);
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

double getDistanceObjects(const FCLCollisionObjectPtr& co1, const FCLCollisionObjectPtr& co2)
{
    if (co1 == nullptr || co2 == nullptr)
    {
        RCLCPP_ERROR(getLogger(), "getDistanceObjects: called with a NULL FCLCollisionObjectPtr.");
        return -1.0;
    }

    fcl::DistanceRequestd dist_req;
    dist_req.enable_nearest_points = false;
    dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    fcl::DistanceResultd dist_result;

    fcl::distance(co1.get(), co2.get(), dist_req, dist_result);

    return dist_result.min_distance;
}


double getDistanceObjects(const FCLObjectPtr& obj1,
                          const FCLObjectPtr& obj2,
                          Eigen::Vector3d& closest_pt_obj1,
                          Eigen::Vector3d& closest_pt_obj2)
{
    if (obj1 == nullptr || obj2 == nullptr)
    {
        RCLCPP_ERROR(getLogger(), "getDistanceObjects: called with a NULL FCLObjectPtr.");
        return -1.0;
    }

    // Create the collision geometries
    FCLCollisionGeometryPtr cg1 = createCollisionGeometry(obj1);
    FCLCollisionGeometryPtr cg2 = createCollisionGeometry(obj2);
    // Convert to FCL coordinates
    fcl::Transform3d world_to_fcl1, world_to_fcl2;
    transform2fcl(obj1->object_transform, world_to_fcl1);
    transform2fcl(obj2->object_transform, world_to_fcl2);
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

double getDistanceObjects(const FCLCollisionObjectPtr& co1,
                          const FCLCollisionObjectPtr& co2,
                          Eigen::Vector3d& closest_pt_obj1,
                          Eigen::Vector3d& closest_pt_obj2)
{
    if (co1 == nullptr || co2 == nullptr)
    {
        RCLCPP_ERROR(getLogger(), "getDistanceObjects: called with a NULL FCLCollisionObjectPtr.");
        return -1.0;
    }

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

double getMinimumObjectDistanceWorld(const FCLObjectPtr& obj, const FCLInterfaceCollisionWorld& world)
{
    if (obj == nullptr)
    {
        RCLCPP_ERROR(getLogger(), "getMinimumObjectDistanceWorld: called with a NULL FCLObjectPtr.");
        return -1.0;
    }

    double min_distance = std::numeric_limits<double>::max();
    std::vector<FCLInterfaceCollisionObjectPtr> world_objects = world.getCollisionObjects();
    for (const auto& other : world_objects)
    {
        double dist_objs = getDistanceObjects(obj, other->object);
        if (min_distance > dist_objs)
        {
            min_distance = dist_objs;
        }
    }

    return min_distance;
}

double getMinimumObjectDistanceWorld(const FCLCollisionObjectPtr& co, const FCLInterfaceCollisionWorld& world)
{
    if (co == nullptr)
    {
        RCLCPP_ERROR(getLogger(), "getMinimumObjectDistanceWorld: called with a NULL FCLCollisionObjectPtr.");
        return -1.0;
    }

    double min_distance = std::numeric_limits<double>::max();
    std::vector<FCLInterfaceCollisionObjectPtr> world_objects = world.getCollisionObjects();
    for (const auto& other : world_objects)
    {
        double dist_objs = getDistanceObjects(co, other->collision_object);
        if (min_distance > dist_objs)
        {
            min_distance = dist_objs;
        }
    }

    return min_distance;
}

void convertGeometryPoseEigenTransform(const geometry_msgs::msg::Pose& geo_pose, Eigen::Affine3d& wTt)
{
    Eigen::Vector3d t(geo_pose.position.x, geo_pose.position.y, geo_pose.position.z);
    Eigen::Quaterniond q(geo_pose.orientation.w,
                            geo_pose.orientation.x,
                            geo_pose.orientation.y,
                            geo_pose.orientation.z);
    wTt.translation() = t;
    wTt.linear() = q.toRotationMatrix();
}

void convertEigenTransformGeometryPose(const Eigen::Affine3d& wTt, geometry_msgs::msg::Pose& geo_pose)
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

void convertGeometryPointEigenVector(const geometry_msgs::msg::Point& geo_point, Eigen::Vector3d& wPt)
{
    wPt.setZero();
    wPt(0) = geo_point.x;
    wPt(1) = geo_point.y;
    wPt(2) = geo_point.z;
}

void convertEigenVectorGeometryPoint(const Eigen::Vector3d& wPt, geometry_msgs::msg::Point& geo_point)
{
    geo_point.x = wPt(0);
    geo_point.y = wPt(1);
    geo_point.z = wPt(2);
}
} // namespace fcl_interface
} // namespace robot_collision_checking