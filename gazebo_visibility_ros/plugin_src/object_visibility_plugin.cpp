#include <iostream>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

//#include "visibility_query.pb.h"
//#include "visibility_result.pb.h"

namespace gazebo
{
//typedef const boost::shared_ptr<const gazebo_visibility_msgs::msgs::VisibilityQuery> VisibilityQueryPtr;
//typedef const boost::shared_ptr<const gazebo_visibility_msgs::msgs::VisibilityResult> VisibilityResultPtr;

class GazeboVisibility : public WorldPlugin
{
  transport::NodePtr node;
  transport::PublisherPtr imagePub;
  transport::SubscriberPtr commandSubscriber;
  physics::WorldPtr world;

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    node = transport::NodePtr(new transport::Node());
    world = _parent;
    // Initialize the node with the world name
    node->Init(world->GetName());
    std::cout << "Subscribing to: " << "~/gazebo_visibility/query" << std::endl;
    //commandSubscriber = node->Subscribe("~/gazebo_visibility/query", &GazeboVisibility::query, this);
    std::cout << "Advertising: " << "~/gazebo_visibility/result" << std::endl;
    //imagePub = node->Advertise<gazebo_visibility_msgs::msgs::VisibilityResult>("~/gazebo_visibility/result");
  }

  //public: void query(VisibilityQueryPtr &msg)
  //{
  //}
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboVisibility)
}
