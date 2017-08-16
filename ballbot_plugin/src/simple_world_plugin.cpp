#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <thread>
#include <cmath>
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64.h"

namespace gazebo
{
class BallbotPlugin : public ModelPlugin
{
public:
  BallbotPlugin()
  {

  }

  public :virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {


    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

	std::cerr << "\nThe ballbot plugin is attach to model[" <<
	_model->GetName() << "]\n";
	
	std::cerr << "\nThe ballbot's joint number is["<<
	_model->GetJointCount() << "]\n";

	this->model = _model;
        this->joint0 = _model->GetJoints()[0];
	this->joint1 = _model->GetJoints()[1];
	this->joint2 = _model->GetJoints()[2];
/* 	this->joint0 = _model->GetJoint("body_link_JOINT_0");
	this->joint1 = _model->GetJoint("body_link_JOINT_1");
	this->joint2 = _model->GetJoint("body_link_JOINT_2");  */
	this->pid    = common::PID(10,0,0,10,-10,10,-10);
	this->model->GetJointController()->SetVelocityPID(this->joint0->GetScopedName(),this->pid);
	this->model->GetJointController()->SetVelocityPID(this->joint1->GetScopedName(),this->pid);
	this->model->GetJointController()->SetVelocityPID(this->joint2->GetScopedName(),this->pid);

// Initialize ros, if it has not already bee initialized.
	if (!ros::isInitialized())
    {
  	int argc = 0;
  	char **argv = NULL;
  	ros::init(argc, argv, "gazebo_client",
      	ros::init_options::NoSigintHandler);
    }

// Create our ROS node. This acts in a similar manner to
// the Gazebo node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

// Create a named topic, and subscribe to it.
	ros::SubscribeOptions so0 =
  	ros::SubscribeOptions::create<std_msgs::Float64>(
      	"/" + this->model->GetName() + "/joint0_vel_cmd",
      	1,
      	boost::bind(&BallbotPlugin::OnRosMsg0, this, _1),
      	ros::VoidPtr(), &this->rosQueue0);
	this->rosSub0 = this->rosNode->subscribe(so0);

	ros::SubscribeOptions so1 =
  	ros::SubscribeOptions::create<std_msgs::Float64>(
      	"/" + this->model->GetName() + "/joint1_vel_cmd",
      	1,
      	boost::bind(&BallbotPlugin::OnRosMsg1, this, _1),
      	ros::VoidPtr(), &this->rosQueue1);
	this->rosSub1 = this->rosNode->subscribe(so1);

	ros::SubscribeOptions so2 =
  	ros::SubscribeOptions::create<std_msgs::Float64>(
      	"/" + this->model->GetName() + "/joint2_vel_cmd",
      	1,
      	boost::bind(&BallbotPlugin::OnRosMsg2, this, _1),
      	ros::VoidPtr(), &this->rosQueue2);
	this->rosSub2 = this->rosNode->subscribe(so2);


// Spin up the queue helper thread.
	this->rosQueueThread =
  	std::thread(std::bind(&BallbotPlugin::QueueThread, this));

   }

    public: void SetVelocity0(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint0->GetScopedName(), _vel);
    }
    public: void SetVelocity1(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint1->GetScopedName(), _vel);
    }
    public: void SetVelocity2(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint2->GetScopedName(), _vel);
    }
/// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
/// of the Velodyne.
	public: void OnRosMsg0(const std_msgs::Float64ConstPtr &_msg)
   {
  	this->SetVelocity0(_msg->data);
   }
	public: void OnRosMsg1(const std_msgs::Float64ConstPtr &_msg)
   {
  	this->SetVelocity1(_msg->data);
   }
	public: void OnRosMsg2(const std_msgs::Float64ConstPtr &_msg)
   {
  	this->SetVelocity2(_msg->data);
   }
/// \brief ROS helper function that processes messages
	private: void QueueThread()
  {
  	static const double timeout = 0.01;
  	while (this->rosNode->ok())
     {
    	this->rosQueue0.callAvailable(ros::WallDuration(timeout));
    	this->rosQueue1.callAvailable(ros::WallDuration(timeout));
    	this->rosQueue2.callAvailable(ros::WallDuration(timeout));
     }
  }

/// \brief Pointer to the model.
private: physics::ModelPtr model;

/// \brief Pointer to the joint.
private: physics::JointPtr joint0,joint1,joint2;

/// \brief A PID controller for the joint.
private: common::PID pid;

/// \brief A node use for ROS transport
private: std::unique_ptr<ros::NodeHandle> rosNode;

/// \brief A ROS subscriber
private: ros::Subscriber rosSub0,rosSub1,rosSub2;

/// \brief A ROS callbackqueue that helps process messages
private: ros::CallbackQueue rosQueue0,rosQueue1,rosQueue2;

/// \brief A thread the keeps running the rosQueue
private: std::thread rosQueueThread;


};
GZ_REGISTER_MODEL_PLUGIN(BallbotPlugin)
}
