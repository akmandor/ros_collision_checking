# C++ API Documentation

The following describes the `robot_collision_checking` ROS package's C++ API for managing and checking collisions 
between various object types in a robot environment, using the Flexible Collision Library (FCL). We break this documentation down into:

1) [Types](#types)
2) [Core Functionality](#core-functionality)
3) [Error Handling](#error-handling)
4) [Testing](#testing)

## Types 

The [fcl_interface_types.hpp](../include/robot_collision_checking/fcl_interface_types.hpp) header file defines the main types of the `robot_collision_checking` package.

### ShapeType

Various geometric shapes can be handled by the package for collision and distance checking, as enumerated by:
```c++
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
```

### FCLObject

A generic container for these different shapes is then provided in the `FCLObject` struct. There are multiple constructors for `FCLObject` that 
can handle each possible `ShapeType`, represented as a ROS message type, e.g., [shape_msgs](https://wiki.ros.org/shape_msgs) for a plane:
```c++
FCLObject(const shape_msgs::msg::Plane& plane, ShapeType type, const Eigen::Affine3d& transform) 
    : object_type(type), object_transform(transform)
{
    ptr.plane = new shape_msgs::msg::Plane(plane);
}
```
In addition to the ROS message type, the `ShapeType` and `Eigen::Affine3d` object transform must also be provided to the constructor. For example:
```c++
// Define the ROS message type
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

// Define the object transform
Eigen::Affine3d plane_pose;
plane_pose.translation() = -distance * n.normalized();
// Calculate the rotation matrix from the original normal z_0 = (0,0,1) to new normal n = (A,B,C)
Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_0, n);
plane_pose.linear() = q.toRotationMatrix();

// Create the FCL object and specify the ShapeType as PLANE
robot_collision_checking::FCLObjectPtr fcl_sphere = std::make_shared<robot_collision_checking::FCLObject>(
    plane, robot_collision_checking::PLANE, plane_pose);
```

`FCLObjectPtr` is an alias for `std::shared_ptr<FCLObject>` and is also defined in [fcl_interface_types.hpp](../include/robot_collision_checking/fcl_interface_types.hpp).

 Note that for `shape_msgs/SolidPrimitive` ROS messages, a `ShapeType` can be inferred within the constructor and is not necessary. 
 Take the following example for a box:
```c++
// Box transform at origin
Eigen::Vector3d eig_wps(0.0, 0.0, 0.0);
Eigen::Affine3d eig_wTs;
eig_wTs.linear() = Eigen::Matrix3d::Identity();
eig_wTs.translation() = eig_wps;

shape_msgs::msg::SolidPrimitive box;
box.dimensions.resize(3);
box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.2;
box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.2;
box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.2;
box.type = shape_msgs::msg::SolidPrimitive::BOX;
// NOTE: no ShapeType is passed to the constructor
robot_collision_checking::FCLObjectPtr fcl_box = std::make_shared<robot_collision_checking::FCLObject>(box, eig_wTs);
```

### FCLInterfaceCollisionObject

Lastly, the `FCLInterfaceCollisionObject` type encapsulates an FCL collision object with its associated `FCLObject` and a collision ID, 
enabling integration with ROS for collision checking:
```c++
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
```
This struct provides the glue between our container for different ROS message types, `FCLObject`, and object types suitable collision and distance checking using FCL, `fcl::CollisionObjectd`. 

To create an `FCLInterfaceCollisionObject`, an `FCLObjectPtr`, an `FCLCollisionObjectPtr`, and an object ID are required. For instance, given a pointer variable `obj` of type `FCLObjectPtr`, an integer `object_id`, and the utility method `fcl_interface::createCollisionGeometry` described in the [fcl_interface section](#fcl_interface), an `FCLInterfaceCollisionObject` can be constructed as follows:
```c++
// Obtain the collision geometry for FCLObjectPtr `obj`
FCLCollisionGeometryPtr cg = fcl_interface::createCollisionGeometry(obj);
// Convert the Eigen Affine3D transform to an FCL coordinate transform
fcl::Transform3d world_to_fcl;
fcl_interface::transform2fcl(obj->object_transform, world_to_fcl);
// Construct an FCLCollisionObjectPtr from the object's corresponding collision geometry and FCL transform
FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(cg, world_to_fcl);

// Create an FCLInterfaceCollisionObjectPtr from the FCLObjectPtr, FCLCollisionObjectPtr, and a given `object_id`
FCLInterfaceCollisionObjectPtr new_col_object = std::make_shared<FCLInterfaceCollisionObject>(obj, co, object_id);
```

## Core Functionality

The package can be used by instantiating a collision world, `FCLInterfaceCollisionWorld`, and/or by using utility functions from our FCL interface, 
[fcl_interface.hpp](../include/robot_collision_checking/fcl_interface.hpp), that are free of any class and have the `robot_collision_checking::fcl_interface` namespace.

### FCLInterfaceCollisionWorld

The `FCLInterfaceCollisionWorld` class is a container and manager for objects in a collision world, providing functionality to 
add, remove, and check collisions between objects. Its state is encapsulated by the following private members:
```c++
// Collision world reference frame
std::string frame_;
// List of collision objects in the world
std::vector<FCLInterfaceCollisionObjectPtr> fcl_collision_objects_;
// Counter for object IDs
unsigned int obj_counter_;
```
There are getter methods to expose these members.

By default, a collision world is constructed without any collision objects (i.e., it's empty) using the constructor:
```c++
FCLInterfaceCollisionWorld(const std::string& frame_id = "world");
```
Which assumes a default frame ID of "world".

Public methods of `FCLInterfaceCollisionWorld` are mainly focused on interacting with collision objects in the world:

#### Collision Object Management

Methods to check if a collision object exists by its `object_id`:
```c++
bool collisionObjectExists(int object_id) const;
// Also returns the index of the object in the list if it exists.
bool collisionObjectExists(int object_id, int& index) const;
```

Methods to add one or more collision objects to the world, each identified by an `object_id`:
```c++
bool addCollisionObjects(const std::vector<FCLObjectPtr>& objects, const std::vector<int>& object_ids);
bool addCollisionObject(const FCLObjectPtr& obj, int object_id);
```
Similar methods exist for removing objects from the world.

#### Collision & Distance Checking

Methods to check whether a specific object (identified by an ID or provided in full) is in collision with any other objects in the world:
```c++
bool checkCollisionObject(int obj_id, std::vector<int>& collision_object_ids) const;
bool checkCollisionObject(const FCLObjectPtr& obj, std::vector<int>& collision_object_ids) const;
bool checkCollisionObject(const FCLCollisionObjectPtr& co, std::vector<int>& collision_object_ids) const;
```
The colliding objects' IDs are returned in the `collision_object_ids` vector.

Get distance methods calculate the distances between a given object and all other objects in the world, e.g., if provided an `obj_id`:
```c++
/** Gets the distances from an FCL object (with an ID or not) in the known world
 * Returns:
 *    1. obj_distances: A vector of distances (len = nbr of objects)
 *    2. closest_pt_object: A vector of points (len = nbr of objects) on the object that are closest to others in the world
 *    3. closest_pt_world: A vector of points (len = nbr of objects) in the world closest to the object 
**/
void getObjectDistances(int obj_id,
                        std::vector<double>& obj_distances,
                        std::vector<Eigen::Vector3d>& closest_pt_obj,
                        std::vector<Eigen::Vector3d>& closest_pt_world) const;
```

There is also a method that returns the minimum distance between a given object and any other object in the world.
```c++
double getMinimumObjectDistance(int obj_id) const;
```

See [interface_test.cpp](../test/interface_test.cpp) and [fcl_interface_example.cpp](../examples/fcl_interface_example.cpp) for example code using objects of type `FCLInterfaceCollisionWorld`.

### fcl_interface

The `robot_collision_checking::fcl_interface` methods declared in [fcl_interface.hpp](../include/robot_collision_checking/fcl_interface.hpp) expose much of the same functionality as `FCLInterfaceCollisionWorld` in terms of checking for collisions, calculating distances between objects, and identifying the closest points on objects. However, no collision world is maintained. Instead these calculations are performed on FCL objects without a specified world, e.g.,:
```c++
bool checkCollisionObjects(const FCLObjectPtr& obj1, const FCLObjectPtr& obj2);
double getDistanceObjects(const FCLCollisionObjectPtr& co1, const FCLCollisionObjectPtr& co2);
```
An assumption when performing these calculations on objects detached from a collision world is that the objects are posed in a common frame of reference. This common frame is typically the "world" frame.

Likewise, an FCL object can also be compared against a given world:
```c++
/** Gets the distances from an FCL object to a given FCLInterfaceCollisionWorld world
 * Returns:
 *    1. obj_distances: A vector of distances (len = nbr of objects)
 *    2. closest_pt_object: A vector of points (len = nbr of objects) on the object that are closest to others in the world
 *    3. closest_pt_world: A vector of points (len = nbr of objects) in the world closest to the object 
**/
void getObjectDistancesWorld(const FCLObjectPtr& obj,
                             const FCLInterfaceCollisionWorld& world,
                             std::vector<double>& obj_distances,
                             std::vector<Eigen::Vector3d>& closest_pt_obj,
                             std::vector<Eigen::Vector3d>& closest_pt_world);
```

There are also other convenience functions to convert ROS or standard types to a format suitable for FCL computation, e.g.,:
```c++
// Create collision FCL geometries from specific ROS msgs
FCLCollisionGeometryPtr createCollisionGeometry(const FCLObjectPtr& obj);
```
For instance, if the `obj` type is a `shape_msgs::msg::SolidPrimitive` then the above method is implemented as:
```c++
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
```

Refer to the [interface_test.cpp](../test/interface_test.cpp) test code for example usage of the methods defined in the `robot_collision_checking::fcl_interface` namespace.

## Error Handling

Errors in `robot_collision_checking` are handled by outputting an `RCLCPP_ERROR` message in the format: **"function_name: error_message"**. Depending on the function where the error occurs, there will also be specific return values, e.g.,:
- Functions with `bool` return types (e.g., `addCollisionObject`), will return `false` on an error.
- Functions with `int` or `double` as return types (e.g., `getMinimumObjectDistance`), will return `-1` or `-1.0`, respectively.
- `void` functions (e.g., `getObjectDistances`) with an error detected will immediately return from the function.
- Functions with pointer return types, like `FCLCollisionGeometryPtr` (e.g., for `createCollisionGeometry`), will return `nullptr` values on an error.

## Testing

There are seven tests implemented in [interface_test.cpp](../test/interface_test.cpp):
1. **TransformToFCL:** To validate the `Eigen::Affine3d` to `fcl::Transform3d` transformation method.
2. **AddRemove:** Asserts that a variety of [shape_msgs](http://wiki.ros.org/shape_msgs) and 
[OctoMaps](https://github.com/OctoMap/octomap_msgs) can be added/removed to a collision world (e.g., adding a sphere shape to a collision world).
3. **AddRemoveVoxelGrid:** Same as "AddRemove", except focused on [VoxelGrids](https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/msg/VoxelGrid.msg).
4. **NullPtrCheck**: Validates that the library outputs error messages for `nullptr` function arguments (e.g., if `nullptr` is passed to `addCollisionObject`).
5. **CollisionCheck**: Validates that FCL collision-checking capabilities operate correctly for ROS types exposed through this package (e.g., testing a collision between a box and sphere in the same coordinate frame).
6. **DistanceCheck**: Validates that FCL distance computations are correct for ROS types exposed through this package (e.g., testing the distance calculations between different object geometries are correct).
7. **OctomapCollDistCheck**: A combination of tests for FCL collision and distance checking when using `robot_collision_checking` as an interface to handle OctoMaps as collision geometries.

