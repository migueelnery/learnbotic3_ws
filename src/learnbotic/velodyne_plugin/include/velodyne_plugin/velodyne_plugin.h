#ifndef GAZEBO_PLUGINS_VELODYNE_PLUGIN_HH
#define GAZEBO_PLUGINS_VELODYNE_PLUGIN_HH

//#include <ignition/transport/Node.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE Velodyne_Plugin : public ModelPlugin
  {
    public: Velodyne_Plugin(); 

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();
    
    private: void OnUpdate();
    private: void OnVelMsg(ConstPosePtr &_msg);

    private: transport::NodePtr node;
    private: transport::SubscriberPtr velSub;
    private: physics::ModelPtr model;
    private: physics::JointPtr joint;
    private: event::ConnectionPtr updateConnection;
    private: double wheelSpeed[2];
    private: double wheelSeparation;
    private: double wheelRadius;
    private: double velocity;
    private: common::Time prevUpdateTime;
    private: physics::LinkPtr link, leftWheelLink, rightWheelLink;


    //private: ignition::transport::Node nodeIgn;
  };      
}
#endif