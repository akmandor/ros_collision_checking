#include <ros_collision_checking/fcl_interface.h>



FCLInterface::FCLInterface ( ros::NodeHandle nh ) : nh_ ( nh ) {
    mkr_pub=nh_.advertise<visualization_msgs::Marker>
            ( "/visualization_marker",1 );
    fcl_collision_world.clear();
}

FCLInterface::~FCLInterface() {
    ROS_INFO ( "Destroying world" );
}


bool FCLInterface::checkCollisionObjectWorld ( const shape_msgs::SolidPrimitive  & s1,
        const  Eigen::Affine3d  & wT1
                                             ) {
    FCLCollisionGeometryPtr cg=FCLInterface::createCollisionGeometry ( s1 );
    fcl::Transform3d wTf1;
    FCLInterface::transform2fcl ( wT1,wTf1 );


    FCLCollisionObjectPtr o1 ( new fcl::CollisionObjectd ( cg,wTf1 ) );

    fcl::CollisionRequestd col_req;
    fcl::CollisionResultd col_result;

    for ( unsigned int i=0; i<fcl_collision_world.size(); i++ ) {
        fcl::collide ( o1.get(),
                       fcl_collision_world[i]->collision_object.get(),
                       col_req,
                       col_result );
        if ( col_result.isCollision() ) {
            return true;
        }
    }
    return false;
}



bool FCLInterface::checkCollisionObjectWorld ( const shape_msgs::SolidPrimitive  & s1,
        const  Eigen::Affine3d  & wT1,
        std::vector<unsigned int> & id_collision_objects
                                             ) {
    id_collision_objects.clear();
    FCLCollisionGeometryPtr cg=FCLInterface::createCollisionGeometry ( s1 );
    fcl::Transform3d wTf1;
    FCLInterface::transform2fcl ( wT1,wTf1 );


    FCLCollisionObjectPtr o1 ( new fcl::CollisionObjectd ( cg,wTf1 ) );

    fcl::CollisionRequestd col_req;
    fcl::CollisionResultd col_result;
    bool in_collision ( false );
    for ( unsigned int i=0; i<fcl_collision_world.size(); i++ ) {
        fcl::collide ( o1.get(),
                       fcl_collision_world[i]->collision_object.get(),
                       col_req,
                       col_result );
        if ( col_result.isCollision() ) {
            id_collision_objects.push_back ( fcl_collision_world[i]->collision_id );
            in_collision=true;
        }
    }
    return in_collision;
}

bool FCLInterface::checkDistanceObjectWorld ( const shape_msgs::SolidPrimitive  & s1,
        const  Eigen::Affine3d  & wT1,
        std::vector<int> & id_collision_objects,
        std::vector<double> & objs_distance,
        std::vector<Eigen::Vector3d> & wP1,
        std::vector<Eigen::Vector3d> & wPobjs
                                            ) {

    id_collision_objects.clear();
    id_collision_objects.resize ( fcl_collision_world.size() );
    wP1.clear();
    wP1.resize ( fcl_collision_world.size() );
    wPobjs.clear();
    wPobjs.resize ( fcl_collision_world.size() );
    objs_distance.clear();
    objs_distance.resize ( fcl_collision_world.size() );

    FCLCollisionGeometryPtr cg=FCLInterface::createCollisionGeometry ( s1 );
    fcl::Transform3d wTf1;
    FCLInterface::transform2fcl ( wT1,wTf1 );


    FCLCollisionObjectPtr o1 ( new fcl::CollisionObjectd ( cg,wTf1 ) );



    std::cout<<"Size of world"<<fcl_collision_world.size() <<std::endl;
    for ( unsigned int i=0; i<fcl_collision_world.size(); i++ ) {
        fcl::DistanceRequestd dist_req;
        dist_req.enable_nearest_points=true;
        fcl::DistanceResultd dist_result;
        id_collision_objects[i]=fcl_collision_world[i]->collision_id;
        FCLCollisionObjectPtr p;
        dist_result.nearest_points[0].setZero();
        dist_result.nearest_points[1].setZero();
        dist_req.enable_nearest_points=true;
        dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

        fcl::distance ( o1.get(),
                        fcl_collision_world[i]->collision_object.get(),
                        dist_req,
                        dist_result );
	
        for ( int j=0; j<3; j++ ) {
            wP1[i] ( j ) =dist_result.nearest_points[0][j];
            wPobjs[i] ( j ) =dist_result.nearest_points[1][j];
        }
        objs_distance[i]=dist_result.min_distance;
    }
}


void FCLInterface::publishPoint ( Eigen::Vector3d pose,
                                 std::string mkr_namespace,
                                 unsigned int id,
                                 std::string frame,
                                 std::vector<double> color,
                                 std::vector<double> scale
                               ) {
    visualization_msgs::Marker mkr;
    mkr.ns=mkr_namespace;
    mkr.action=visualization_msgs::Marker::ADD;
    mkr.type=visualization_msgs::Marker::SPHERE;
    mkr.id=id;
    mkr.pose.position.x=pose ( 0 );
    mkr.pose.position.y=pose ( 1 );
    mkr.pose.position.z=pose ( 2 );
    mkr.pose.orientation.x=0.0;
    mkr.pose.orientation.y=0.0;
    mkr.pose.orientation.z=0.0;
    mkr.pose.orientation.w=0.0;
    mkr.header.frame_id=frame;
    mkr.lifetime=ros::Duration ( 0.0 );
    mkr.color.r=color[0];
    mkr.color.g=color[1];
    mkr.color.b=color[2];
    mkr.color.a=0.8;
    mkr.scale.x=scale[0];
    mkr.scale.y=scale[1];
    mkr.scale.z=scale[2];
    while ( mkr_pub.getNumSubscribers() <1 ) {
        ROS_INFO ( "Waiting for subs" );
        ros::spinOnce();
    }
    mkr_pub.publish ( mkr );

}

bool FCLInterface::addCollisionObject ( const shape_msgs::SolidPrimitive & s1,
                                        const  Eigen::Affine3d  & wT1,
                                        unsigned int object_id ) {
    FCLCollisionGeometryPtr cg=FCLInterface::createCollisionGeometry ( s1 );
    fcl::Transform3d wTf1;
    FCLInterface::transform2fcl ( wT1,wTf1 );
    FCLCollisionObjectPtr o1 ( new fcl::CollisionObjectd ( cg,wTf1 ) );
    FCLInterfaceCollisionObject *new_col_object ( new FCLInterfaceCollisionObject() );
    new_col_object->collision_object=o1;
    new_col_object->object_type=s1.type;
    new_col_object->collision_id=object_id;
    new_col_object->solid=s1;
    fcl_collision_world.push_back ( std::unique_ptr<FCLInterfaceCollisionObject> ( new_col_object ) );
}
bool FCLInterface::addCollisionObject ( const shape_msgs::Plane  & s1,
                                        const  Eigen::Affine3d  & wT1, unsigned int object_id ) {

    FCLCollisionGeometryPtr cg=FCLInterface::createCollisionGeometry ( s1 );
    fcl::Transform3d wTf1;
    FCLInterface::transform2fcl ( wT1,wTf1 );
    FCLCollisionObjectPtr o1 ( new fcl::CollisionObjectd ( cg,wTf1 ) );
    FCLInterfaceCollisionObject *new_col_object;
    new_col_object->collision_object=o1;
    new_col_object->object_type=PLANE;
    new_col_object->collision_id=object_id;
    new_col_object->plane=s1;
    fcl_collision_world.push_back ( std::unique_ptr<FCLInterfaceCollisionObject> ( new_col_object ) );

}
bool FCLInterface::addCollisionObject ( const shape_msgs::Mesh  & s1 ,
                                        const  Eigen::Affine3d  & wT1,unsigned int object_id ) {

    FCLCollisionGeometryPtr cg=FCLInterface::createCollisionGeometry ( s1 );
    fcl::Transform3d wTf1;
    FCLInterface::transform2fcl ( wT1,wTf1 );
    FCLCollisionObjectPtr o1 ( new fcl::CollisionObjectd ( cg,wTf1 ) );
    FCLInterfaceCollisionObject *new_col_object;
    new_col_object->collision_object=o1;
    new_col_object->object_type=visualization_msgs::Marker::TRIANGLE_LIST;
    new_col_object->collision_id=object_id;
    new_col_object->mesh=s1;
    fcl_collision_world.push_back ( std::unique_ptr<FCLInterfaceCollisionObject> ( new_col_object ) );

}
bool FCLInterface::removeCollisionObject ( unsigned int object_id ) {
    for ( unsigned int i=0; i<fcl_collision_world.size(); i++ ) {
        if ( object_id==fcl_collision_world[i]->collision_id ) {
            fcl_collision_world.erase ( fcl_collision_world.begin() +i );
            return true;
        }
    }
    return false;
}
bool FCLInterface::displayObjects ( std::vector<unsigned int> object_ids ) {
    for ( unsigned int j=0; j<object_ids.size(); j++ ) {
        for ( unsigned int i=0; i<fcl_collision_world.size(); i++ ) {
            if ( object_ids[j]==fcl_collision_world[i]->collision_id ) {
                visualization_msgs::Marker mkr;
                mkr.id=fcl_collision_world[i]->collision_id;
                if ( fcl_collision_world[i]->object_type==visualization_msgs::Marker::TRIANGLE_LIST ) {
                    geometric_shapes::constructMarkerFromShape ( fcl_collision_world[i]->mesh,mkr );
                } else if ( fcl_collision_world[i]->object_type==PLANE ) {

                } else if ( fcl_collision_world[i]->object_type==shape_msgs::SolidPrimitive::SPHERE ||
                            fcl_collision_world[i]->object_type==shape_msgs::SolidPrimitive::BOX ||
                            fcl_collision_world[i]->object_type==shape_msgs::SolidPrimitive::CYLINDER ) {
                    geometric_shapes::constructMarkerFromShape ( fcl_collision_world[i]->solid,mkr );
                }
                mkr_pub.publish ( mkr );
                ros::spinOnce();
            }
        }

    }
}
bool FCLInterface::displayObjects() {
    visualization_msgs::Marker mkr;
    for ( unsigned int i=0; i<fcl_collision_world.size(); i++ ) {


        fcl::Transform3d wTf2= fcl_collision_world[i]->collision_object->getTransform();


        mkr.id=fcl_collision_world[i]->collision_id;
        if ( fcl_collision_world[i]->object_type==visualization_msgs::Marker::TRIANGLE_LIST ) {
            geometric_shapes::constructMarkerFromShape ( fcl_collision_world[i]->mesh,mkr );
        } else if ( fcl_collision_world[i]->object_type==PLANE ) {

        } else if ( fcl_collision_world[i]->object_type==shape_msgs::SolidPrimitive::SPHERE ||
                    fcl_collision_world[i]->object_type==shape_msgs::SolidPrimitive::BOX ||
                    fcl_collision_world[i]->object_type==shape_msgs::SolidPrimitive::CYLINDER ) {
            std::cout<<fcl_collision_world[i]->solid<<std::endl;

            geometric_shapes::constructMarkerFromShape ( fcl_collision_world[i]->solid,mkr );
        }



        while ( mkr_pub.getNumSubscribers() <1 ) {
            ROS_INFO ( "Waiting for Marker Subs" );
            ros::spinOnce();
            ros::Duration ( 0.1 ).sleep();
        }
        mkr.action=visualization_msgs::Marker::ADD;
        mkr.header.frame_id="world";
        mkr.ns="Objects";
        mkr.lifetime=ros::Duration ( 0.0 );
        mkr.color.a=0.5;

        Eigen::Quaterniond q ( wTf2.linear() );
        mkr.pose.position.x=wTf2 ( 0,3 );
        mkr.pose.position.y=wTf2 ( 1,3 );
        mkr.pose.position.z=wTf2 ( 2,3 );
        mkr.pose.orientation.w=q.w();
        mkr.pose.orientation.x=q.x();
        mkr.pose.orientation.y=q.y();
        mkr.pose.orientation.z=q.z();
        mkr_pub.publish ( mkr );
        ros::spinOnce();
    }
}



// Plane is defined as a x + by + cz + d = 0;
FCLCollisionGeometryPtr FCLInterface::createCollisionGeometry
( const shape_msgs::Mesh  & s1 ) {


    std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd> >  g ( new fcl::BVHModel<fcl::OBBRSSd>() );

    if ( s1.vertices.size() > 0 && s1.triangles.size() > 0 ) {
        std::vector<fcl::Triangle> tri_indices ( s1.triangles.size() );
        std::vector<fcl::Vector3d> points ( s1.vertices.size() );

        for ( unsigned int i = 0; i <  s1.triangles.size(); ++i ) {
            tri_indices[i] =
                fcl::Triangle ( s1.triangles[i].vertex_indices[0],
                                s1.triangles[i].vertex_indices[1],
                                s1.triangles[i].vertex_indices[2] );
        }

        for ( unsigned int i = 0; i < s1.vertices.size(); ++i ) {
            points[i] = fcl::Vector3d ( s1.vertices[i].x, s1.vertices[i].y, s1.vertices[i].z );
        }

        g->beginModel();
        g->addSubModel ( points, tri_indices );
        g->endModel();
    }
    return g;
}


// Plane is defined as a x + by + cz + d = 0;
FCLCollisionGeometryPtr FCLInterface::createCollisionGeometry
( const shape_msgs::Plane  & s1 ) {
    return FCLCollisionGeometryPtr (
               new fcl::Planed ( s1.coef[0],
                                 s1.coef[1],
                                 s1.coef[2],
                                 s1.coef[3]
                               ) );
}

FCLCollisionGeometryPtr FCLInterface::createCollisionGeometry ( const shape_msgs::SolidPrimitive & s1 ) {
    if ( s1.type==shape_msgs::SolidPrimitive::SPHERE ) {
        return  FCLCollisionGeometryPtr (
                    new fcl::Sphered ( s1.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] ) );
    } else if ( s1.type==shape_msgs::SolidPrimitive::BOX ) {

        return  FCLCollisionGeometryPtr (
                    new fcl::Boxd ( s1.dimensions[shape_msgs::SolidPrimitive::BOX_X],
                                    s1.dimensions[shape_msgs::SolidPrimitive::BOX_Y],
                                    s1.dimensions[shape_msgs::SolidPrimitive::BOX_Z]
                                  ) );
    } else if ( s1.type==shape_msgs::SolidPrimitive::CONE ) {
        return  FCLCollisionGeometryPtr (
                    new fcl::Coned ( s1.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS],
                                     s1.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT]
                                   ) );
    } else if ( s1.type==shape_msgs::SolidPrimitive::CYLINDER ) {
        return  FCLCollisionGeometryPtr (
                    new fcl::Cylinderd ( s1.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS],
                                         s1.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]
                                       ) );
    } else {
        return nullptr;
    }

}



void FCLInterface::transform2fcl ( const Eigen::Affine3d& b, fcl::Transform3d& f ) {
    f.linear() =b.linear();
    f.translation() =b.translation();
}


double FCLInterface::checkDistanceObjects ( const shape_msgs::SolidPrimitive & s1,
        const  Eigen::Affine3d  & wT1,
        const shape_msgs::SolidPrimitive  &  s2,
        const Eigen::Affine3d  & wT2
                                          ) {
    fcl::Transform3d wTf1,wTf2;
    transform2fcl ( wT1,wTf1 );
    transform2fcl ( wT2,wTf2 );
    FCLCollisionGeometryPtr cg_1=createCollisionGeometry ( s1 );
    FCLCollisionGeometryPtr cg_2=createCollisionGeometry ( s2 );
    fcl::CollisionObjectd *o1=new fcl::CollisionObjectd ( cg_1,wTf1 );
    fcl::CollisionObjectd *o2=new fcl::CollisionObjectd ( cg_2,wTf2 );
    fcl::DistanceResultd dist_result;
    fcl::DistanceRequestd dist_req;
    //dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    dist_req.enable_nearest_points=false;
    dist_result.nearest_points[0].setZero();
    dist_result.nearest_points[1].setZero();

    fcl::distance ( o1, o2, dist_req, dist_result );
    return dist_result.min_distance;
}
double FCLInterface::checkDistanceObjects ( const shape_msgs::SolidPrimitive & s1,
        const  Eigen::Affine3d  & wT1,
        const shape_msgs::SolidPrimitive  &  s2,
        const Eigen::Affine3d  & wT2,
        Eigen::Vector3d & wP1,
        Eigen::Vector3d & wP2
                                          ) {
    fcl::Transform3d wTf1,wTf2;
    transform2fcl ( wT1,wTf1 );
    transform2fcl ( wT2,wTf2 );
    FCLCollisionGeometryPtr cg_1=createCollisionGeometry ( s1 );
    FCLCollisionGeometryPtr cg_2=createCollisionGeometry ( s2 );
    fcl::CollisionObjectd *o1=new fcl::CollisionObjectd ( cg_1,wTf1 );
    fcl::CollisionObjectd *o2=new fcl::CollisionObjectd ( cg_2,wTf2 );

    std::cout<<"cg_1->computeVolume() "<<cg_1->computeVolume() <<std::endl;
    std::cout<<"cg_2->computeVolume() "<<cg_2->computeVolume() <<std::endl;
    fcl::DistanceRequestd dist_req;
    dist_req.enable_nearest_points=true;
    dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    fcl::DistanceResultd dist_result;

    dist_result.nearest_points[0].setZero();
    dist_result.nearest_points[1].setZero();
    fcl::distance ( o1, o2, dist_req, dist_result );
    for ( int i=0; i<3; i++ ) {
        wP1 ( i ) =dist_result.nearest_points[0][i];
        wP2 ( i ) =dist_result.nearest_points[1][i];
    }

    delete o1,o2;
    return dist_result.min_distance;
}


bool FCLInterface::checkCollisionObjects ( const shape_msgs::SolidPrimitive  & s1,
        const  Eigen::Affine3d  & wT1,
        const shape_msgs::SolidPrimitive  & s2,
        const Eigen::Affine3d  & wT2
                                         ) {
    fcl::Transform3d wTf1,wTf2;
    FCLCollisionGeometryPtr cg_1=createCollisionGeometry ( s1 );
    FCLCollisionGeometryPtr cg_2=createCollisionGeometry ( s2 );
    transform2fcl ( wT1,wTf1 );
    transform2fcl ( wT2,wTf2 );
    fcl::CollisionObjectd *o1=new fcl::CollisionObjectd ( cg_1,wTf1 );
    fcl::CollisionObjectd *o2=new fcl::CollisionObjectd ( cg_2,wTf2 );
    fcl::CollisionRequestd col_req;
    fcl::CollisionResultd col_result;
    fcl::collide ( o1, o2, col_req, col_result );
    return col_result.isCollision();
}



void FCLInterface::convertGeometryPoseEigenTransform ( const geometry_msgs::Pose & geo_pose,
        Eigen::Affine3d& wTt ) {
    Eigen::Vector3d t ( geo_pose.position.x, geo_pose.position.y,geo_pose.position.z );
    Eigen::Quaterniond q ( geo_pose.orientation.w,
                           geo_pose.orientation.x,
                           geo_pose.orientation.y,
                           geo_pose.orientation.z );
    wTt.translation() =t;
    wTt.linear() =q.toRotationMatrix();
}

void FCLInterface::convertEigenTransformGeometryPose ( const  Eigen::Affine3d & wTt,
        geometry_msgs::Pose& geo_pose ) {

    Eigen::Quaterniond q ( wTt.linear() );
    q.normalize();
    Eigen::Vector3d t ( wTt.translation() );

    geo_pose.position.x=t ( 0 );
    geo_pose.position.y=t ( 1 );
    geo_pose.position.z=t ( 2 );
    geo_pose.orientation.x=q.x();
    geo_pose.orientation.y=q.y();
    geo_pose.orientation.z=q.z();
    geo_pose.orientation.w=q.w();

}

void FCLInterface::convertGeometryPointEigenVector ( const geometry_msgs::Point & geo_pose,
        Eigen::Vector3d& t ) {
    t.setZero();
    t ( 0 ) =geo_pose.x;
    t ( 1 ) =geo_pose.y;
    t ( 2 ) =geo_pose.z;
}
void FCLInterface::convertEigenVectorGeometryPoint ( const  Eigen::Vector3d& t,
        geometry_msgs::Point& geo_pose ) {
    geo_pose.x=t ( 0 );
    geo_pose.y=t ( 1 );
    geo_pose.z=t ( 2 );
}







int main ( int argc, char **argv ) {
    std::cout<<"hello"<<std::endl;
    ros::init ( argc, argv, "robust" ); // ros init
    ros::NodeHandle nh; // Create a node handle and start the node
    ROS_INFO ( " RUNNING TEST FCL" );

    ROS_INFO ( "SIMPLE FCL INTERFACE" );
    std::cout<<"---------------------------------------------"<<std::endl;
    Eigen::Vector3d e_wps1 ( 0.0,0.0,0.0 ),e_wps2 ( -1.3,2.0,0.3 ),e_wps3 ( 2.0,0.5,0.0 ),e_wps4 ( 3.0,0.0,-0.25 ),e_wps5 ( 2.21,-1.23,0.21 );
    Eigen::Matrix3d e_wRs1,e_wRs2,e_wRs3,e_wRs4,e_wRs5;
    Eigen::Affine3d e_wTs1,e_wTs2,e_wTs3,e_wTs4,e_wTs5;//(wRs1,wps1);
    
    e_wRs1.setIdentity();
    e_wRs2.setIdentity();
    e_wRs3.setIdentity();
    e_wRs4.setIdentity();
    e_wRs5.setIdentity();
    Eigen::Quaterniond q(0.5,0.5,0.23,0.43);q.normalize();
    Eigen::Quaterniond q2(-0.5,0.5,-1.23,0.43);q2.normalize();
    e_wTs1.linear() =e_wRs1;
    e_wTs1.translation() =e_wps1;
    e_wTs2.linear() =e_wRs2;
    e_wTs2.translation() =e_wps2;
    e_wTs3.linear() =e_wRs3;
    e_wTs3.translation() =e_wps3;
    e_wTs4.linear() =q.toRotationMatrix();
    e_wTs4.translation() =e_wps4;
    e_wTs5.linear() =q2.toRotationMatrix();
    e_wTs5.translation() =e_wps5;

    fcl::Transform3d wTs1,wTs2,wTs3,wTs4,wTs5;//(wRs1,wps1);


    FCLInterface::transform2fcl ( e_wTs1,wTs1 );
    FCLInterface::transform2fcl ( e_wTs2,wTs2 );
    FCLInterface::transform2fcl ( e_wTs3,wTs3 );
    FCLInterface::transform2fcl ( e_wTs4,wTs4 );
    FCLInterface::transform2fcl ( e_wTs5,wTs5 );

    std::shared_ptr<fcl::CollisionGeometryd> cg_1 ( new fcl::Sphered ( 0.3 ) );
    std::shared_ptr<fcl::CollisionGeometryd> cg_2 ( new fcl::Sphered ( 0.75 ) );
    std::shared_ptr<fcl::CollisionGeometryd> cg_3 ( new fcl::Boxd ( 0.2,0.2,0.2 ) );
    std::shared_ptr<fcl::CollisionGeometryd> cg_4 ( new fcl::Cylinderd ( 0.1,1.0 ) );
    std::shared_ptr<fcl::CollisionGeometryd> cg_5 ( new fcl::Boxd ( 0.25,0.5,0.2 ) );
    //
    fcl::CollisionObjectd *o1=new fcl::CollisionObjectd ( cg_1,wTs1 );
    fcl::CollisionObjectd *o2=new fcl::CollisionObjectd ( cg_2,wTs2 );




    fcl::CollisionRequestd request;
    fcl::DistanceRequestd dist_req;
    fcl::detail::GJKSolver_libccd< double> solver1;
    dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    dist_req.enable_nearest_points=true;

//     // result will be returned via the collision result structure
    fcl::CollisionResultd result;
    fcl::DistanceResultd dist_result,dist_result5;
//     // perform distance test
    dist_result.nearest_points[0].setZero();
    dist_result.nearest_points[1].setZero();

    fcl::distance ( o1, o2, dist_req, dist_result );

    std::cout<<"o1 o2 Min distance "<<dist_result.min_distance<<std::endl;
    std::cout<<"Nearest point 1   = "<<dist_result.nearest_points[0]<<std::endl;
    std::cout<<"Nearest point 2   = "<<dist_result.nearest_points[1]<<std::endl;

    delete o2;
    fcl::CollisionObjectd *o3=new fcl::CollisionObjectd ( cg_3,wTs3 );
    fcl::distance ( o1, o3, dist_req, dist_result );
//
    std::cout<<"o1 Box1 Min distance "<<dist_result.min_distance<<std::endl;
    std::cout<<"Nearest point 1   = "<<dist_result.nearest_points[0]<<std::endl;
    std::cout<<"Nearest point 2   = "<<dist_result.nearest_points[1]<<std::endl;


    delete o3;
    fcl::CollisionObjectd *o4=new fcl::CollisionObjectd ( cg_4,wTs4 );

    fcl::distance ( o1, o4, dist_req, dist_result );
//
    std::cout<<"o1 Cylinder1 Min distance "<<dist_result.min_distance<<std::endl;
    std::cout<<"Nearest point 1   = "<<dist_result.nearest_points[0]<<std::endl;
    std::cout<<"Nearest point 2   = "<<dist_result.nearest_points[1]<<std::endl;

    delete o4;
    fcl::CollisionObjectd *o5=new fcl::CollisionObjectd ( cg_5,wTs5 );
    fcl::distance ( o1, o5, dist_req, dist_result5 );

    std::cout<<"o1 Box2 Min distance "<<dist_result5.min_distance<<std::endl;
    std::cout<<"Nearest point 1   = "<<dist_result5.nearest_points[0]<<std::endl;
    std::cout<<"Nearest point 2   = "<<dist_result5.nearest_points[1]<<std::endl;

    delete o1,o2,o3,o4,o5;
    ROS_INFO ( "Static functions" );
    std::cout<<"---------------------------------------------"<<std::endl;
    shape_msgs::SolidPrimitive sphere1,sphere2,cylinder1,box1,box2;

    sphere1.dimensions.resize ( 1 );
    sphere1.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.3;
    sphere1.type=shape_msgs::SolidPrimitive::SPHERE;
    sphere2.dimensions.resize ( 1 );
    sphere2.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.75;
    sphere2.type=shape_msgs::SolidPrimitive::SPHERE;
    box1.dimensions.resize ( 3 );
    box1.type=shape_msgs::SolidPrimitive::BOX;
    box2.type=shape_msgs::SolidPrimitive::BOX;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_X]=0.2;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_Y]=0.2;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_Z]=0.2;
    box2.dimensions.resize ( 3 );
    box2.dimensions[shape_msgs::SolidPrimitive::BOX_X]=0.25;
    box2.dimensions[shape_msgs::SolidPrimitive::BOX_Y]=0.5;
    box2.dimensions[shape_msgs::SolidPrimitive::BOX_Z]=0.2;
    cylinder1.dimensions.resize ( 2 );
    cylinder1.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]=1.0;
    cylinder1.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]=0.1;
    cylinder1.type=shape_msgs::SolidPrimitive::CYLINDER;




    Eigen::Vector3d p1,p2;
    double distance;
    distance=FCLInterface::checkDistanceObjects ( sphere1,e_wTs1,sphere2,e_wTs2,p1,p2 );
    std::cout<<" sphere1 & sphere2  distance = "<<distance<<std::endl;
    std::cout<<" Closest Points p1 = ["<<p1 ( 0 ) <<", "<<p1 ( 1 ) <<", "<<p1 ( 2 ) <<"]"<<std::endl;
    std::cout<<"                p2 = ["<<p2 ( 0 ) <<", "<<p2 ( 1 ) <<", "<<p2 ( 2 ) <<"]"<<std::endl;

    distance=FCLInterface::checkDistanceObjects ( sphere1,e_wTs1,box1,e_wTs3,p1,p2 );
    std::cout<<" sphere1 & box1  distance = "<<distance<<std::endl;
    std::cout<<" Closest Points p1 = ["<<p1 ( 0 ) <<", "<<p1 ( 1 ) <<", "<<p1 ( 2 ) <<"]"<<std::endl;
    std::cout<<"                p2 = ["<<p2 ( 0 ) <<", "<<p2 ( 1 ) <<", "<<p2 ( 2 ) <<"]"<<std::endl;

    distance=FCLInterface::checkDistanceObjects ( sphere1,e_wTs1,cylinder1,e_wTs4,p1,p2 );
    std::cout<<" sphere1 & cylinder1  distance = "<<distance<<std::endl;
    std::cout<<" Closest Points p1 = ["<<p1 ( 0 ) <<", "<<p1 ( 1 ) <<", "<<p1 ( 2 ) <<"]"<<std::endl;
    std::cout<<"                p2 = ["<<p2 ( 0 ) <<", "<<p2 ( 1 ) <<", "<<p2 ( 2 ) <<"]"<<std::endl;

    distance=FCLInterface::checkDistanceObjects ( sphere1,e_wTs1,box2,e_wTs5,p1,p2 );
    std::cout<<" sphere1 & box2  distance = "<<distance<<std::endl;
    std::cout<<" Closest Points p1 = ["<<p1 ( 0 ) <<", "<<p1 ( 1 ) <<", "<<p1 ( 2 ) <<"]"<<std::endl;
    std::cout<<"                p2 = ["<<p2 ( 0 ) <<", "<<p2 ( 1 ) <<", "<<p2 ( 2 ) <<"]"<<std::endl;

    ROS_INFO ( "DYNAMIC functions" );
    std::cout<<"---------------------------------------------"<<std::endl;

    FCLInterface test_node ( nh );
    std::cout<<" ===================== "<<std::endl;
    test_node.addCollisionObject ( sphere2,e_wTs2,0 );
    test_node.addCollisionObject ( box1,e_wTs3,1 );
    test_node.addCollisionObject ( cylinder1,e_wTs4,2 );
    test_node.addCollisionObject ( box2,e_wTs5,3 );
    test_node.displayObjects();
    std::vector<double> obj_distances;
    std::vector<int> obj_ids;
    std::vector<Eigen::Vector3d> p1w,p2w;
    test_node.checkDistanceObjectWorld ( sphere1,e_wTs1,obj_ids,obj_distances,p1w,p2w );

    for ( unsigned int i=0; i<obj_distances.size(); i++ ) {
        std::cout<<"Obj id"<<obj_ids[i]<<std::endl;
        std::cout<<"Sphere distance  "<<obj_distances[i]<<std::endl;
        std::cout<<" Closest Points p1w = ["<<p1w[i] ( 0 ) <<", "<<p1w[i] ( 1 ) <<", "<<p1w[i] ( 2 ) <<"]"<<std::endl;
        std::cout<<"                p2w = ["<<p2w[i] ( 0 ) <<", "<<p2w[i] ( 1 ) <<", "<<p2w[i] ( 2 ) <<"]"<<std::endl;
	test_node.publishPoint(p1w[i],"closest_point1",i,"world",{1.0,0.0,1.0},{0.1,0.1,0.1});
	test_node.publishPoint(p2w[i],"closest_point2",i,"world",{1.0,0.0,0.0},{0.1,0.1,0.1});
    }

    test_node.publishPoint(e_wTs1.translation(),"object",100,"world",{1.0,0.0,0.0},{0.6,0.6,0.6});
    return 1;
}























