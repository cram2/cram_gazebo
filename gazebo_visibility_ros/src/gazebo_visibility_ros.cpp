#include <iostream>
#include <math.h>
#include <deque>
#include <sdf/sdf.hh>

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/Collision.hh>

#include <gazebo_visibility_ros/QueryGazeboVisibility.h>

using namespace std;

namespace gazebo
{

void getBBoxGrid(math::Box const& bbox, int size, std::vector<math::Vector3> &points)
{
    points.clear();
    points.resize(size*size*size);
    math::Vector3 d = bbox.GetSize();
    math::Vector3 lrd = bbox.GetCenter() - d*0.5;
    math::Vector3 s = d*(1.0/(size + 1));
    int l = 0;
    for(int k = 0; k < size; k++)
        for(int j = 0; j < size; j++)
            for(int i = 0; i < size; i++)
            {
                math::Vector3 disp;
                disp.x = s.x*(k + 1);
                disp.y = s.y*(j + 1);
                disp.z = s.z*(i + 1);
                points[l] = lrd + disp;
                l++;
            };
}

math::Vector3 getEndPoint(math::Vector3 const& start, math::Vector3 const& end, double maxDist)
{
    math::Vector3 dir = end - start;
    dir = dir.Normalize();
    dir = dir*maxDist;

    return start + dir;
}

bool inImage(math::Vector3 const& start, math::Vector3 const& end, math::Vector3 const& cx, math::Vector3 const& cy, math::Vector3 const& cz, double upAngle, double sideAngle)
{
    math::Vector3 dir = end - start;
    dir = dir.Normalize();

    math::Vector3 hProj, vProj;
    double sA, uA;

    hProj = dir - (cz*dir)*cz;
    hProj = hProj.Normalize();
    vProj = dir - (cy*dir)*cy;
    vProj = vProj.Normalize();

    sA = hProj.Dot(cx);
    uA = vProj.Dot(cx);
    return ((sideAngle <= sA) && (upAngle <= uA));
}

std::string trimSubEnts(std::string const& name)
{
    return name.substr(0, name.find(':'));
}

bool doQueryGazeboVisibility(physics::WorldPtr world, sdf::ElementPtr sdf, gazebo_visibility_ros::QueryGazeboVisibility::Request &request, gazebo_visibility_ros::QueryGazeboVisibility::Response &response)
{
    ROS_INFO("Got QueryGazeboVisibility service request.");
    physics::ModelPtr model = world->GetModel(request.name);
    if((!model.get()))
    {
        ROS_INFO("Model name not loaded.");
        response.visible = false;
        return true;
    }
    ROS_INFO("Model name seems to make sense.");

    math::Box bbox = model->GetBoundingBox();
    //math::Pose pose = model->GetWorldPose();

    gazebo::physics::PhysicsEnginePtr engine = world->GetPhysicsEngine();
    engine->InitForThread();

    gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(engine->CreateShape("ray", gazebo::physics::CollisionPtr()));
    math::Vector3 start, cameraFwd, cameraSide, cameraUp;
    start.x = request.camera_pose.x;
    start.y = request.camera_pose.y;
    start.z = request.camera_pose.z;
    cameraFwd.x = request.camera_fwd.x;
    cameraFwd.y = request.camera_fwd.y;
    cameraFwd.z = request.camera_fwd.z;
    cameraFwd = cameraFwd.Normalize();
    cameraUp.x = request.camera_up.x;
    cameraUp.y = request.camera_up.y;
    cameraUp.z = request.camera_up.z;
    cameraUp = cameraUp.Normalize();
    cameraUp = cameraUp - (cameraUp*cameraFwd)*cameraFwd;
    cameraUp = cameraUp.Normalize();
    cameraSide = cameraUp.Cross(cameraFwd);

    double h2 = request.height*0.5;
    double w2 = request.width*0.5;

    math::Vector3 aux = request.focal_distance*cameraFwd + h2*cameraUp;
    aux = aux.Normalize();
    double upAngle = aux.Dot(cameraFwd);
    aux = request.focal_distance*cameraFwd + w2*cameraSide;
    aux = aux.Normalize();
    double sideAngle = aux.Dot(cameraFwd);

    std::vector<math::Vector3> points;

    getBBoxGrid(bbox, 8, points);
    ROS_INFO("Got a grid of points to test.");

    int maxK = points.size();
    int okPoints = 0;
    for(int k = 0; k < maxK; k++)
    {
        math::Vector3 end = getEndPoint(start, points[k], request.max_distance);
        if(inImage(start, end, cameraFwd, cameraSide, cameraUp, upAngle, sideAngle))
        {
            ray->SetPoints(start, end);
            double dist;
            std::string entityName;
            ray->GetIntersection(dist, entityName);
            if(request.name == trimSubEnts(entityName))
                okPoints++;
        }
    }
    ROS_INFO("Done with raytrace.");

    response.visible = false;
    if(request.threshold <= (okPoints*1.0)/(maxK*1.0))
        response.visible = true;

    return true;
}

class GazeboVisibilityROS : public WorldPlugin
{
public:
  GazeboVisibilityROS() : WorldPlugin()
  {
  }

  void Load(physics::WorldPtr world, sdf::ElementPtr sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    _world = world;
    _sdf = sdf;

    ROS_INFO("Starting up the QueryGazeboVisibility service.");

    queryGazeboVisibility_service = n.advertiseService<gazebo_visibility_ros::QueryGazeboVisibility::Request, gazebo_visibility_ros::QueryGazeboVisibility::Response>("/gazebo_visibility_ros/QueryGazeboVisibility",
                                                       boost::bind(doQueryGazeboVisibility, _world, _sdf, _1, _2));
    ros::spin();
  }

private:
  physics::WorldPtr _world;
  sdf::ElementPtr _sdf;
  ros::NodeHandle n;
  ros::ServiceServer queryGazeboVisibility_service;

};
GZ_REGISTER_WORLD_PLUGIN(GazeboVisibilityROS)
}
