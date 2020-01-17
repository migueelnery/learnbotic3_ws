#include "velodyne_plugin/velodyne_plugin.h"

#include <functional>
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(Velodyne_Plugin)

double velocity;

Velodyne_Plugin::Velodyne_Plugin(){}

void Velodyne_Plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->Name());

  this->velocity = _sdf->GetElement("velocity")->Get<double>();

  this->velSub = this->node->Subscribe(std::string("~/") +
    this->model->GetName() + "/vel_cmd", &Velodyne_Plugin::OnVelMsg, this);
  
  this->joint = _model->GetJoint(_sdf->GetElement("test")->Get<std::string>());
  if (!this->joint)
    gzerr << "Unable to find joint["
          << _sdf->GetElement("test")->Get<std::string>() << "]\n";

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Velodyne_Plugin::OnUpdate, this));
}

void Velodyne_Plugin::Init()
{
  physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
      this->joint->GetChild());
}

void Velodyne_Plugin::OnVelMsg(ConstPosePtr &_msg)
{}

void Velodyne_Plugin::OnUpdate()
  {
  this->joint->SetVelocity(0,velocity);
}

  