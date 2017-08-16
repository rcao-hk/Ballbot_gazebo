#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <gazebo_msgs/ApplyJointEffort.h>
#include <iostream>
#include <Eigen/Dense>

Eigen::MatrixXd m(3,3);
Eigen::Vector3d r;
Eigen::Vector3d v;

ros::Subscriber yaw_sub;
ros::Subscriber pitch_sub;
ros::Subscriber roll_sub;
double yaw_control_effort, pitch_control_effort, roll_control_effort;

ros::Duration duration(0.005);
ros::ServiceClient client;

void matrixInit(void)
{

  m << -2,-1.732,1,
-2,0,-2,
-2,1.732,1;
}

void yawCallback(const std_msgs::Float64& control_effort_input)
{	
     yaw_control_effort = control_effort_input.data;
  

}

void pitchCallback(const std_msgs::Float64& control_effort_input)
{
     pitch_control_effort = control_effort_input.data;

}

void rollCallback(const std_msgs::Float64& control_effort_input)
{
     roll_control_effort = control_effort_input.data;

}

void controlCallback(const ros::TimerEvent& event)
{
     v(0) = yaw_control_effort;
     v(1) = pitch_control_effort;
     v(2) = roll_control_effort;
     r = m*v;

  gazebo_msgs::ApplyJointEffort joint0, joint1, joint2;
  joint0.request.joint_name = "Ballbot::body_link_JOINT_0";
  joint0.request.effort = r(0);
  joint0.request.duration= duration;
  joint1.request.joint_name = "Ballbot::body_link_JOINT_1";
  joint1.request.effort = r(1);
  joint1.request.duration= duration;
  joint2.request.joint_name = "Ballbot::body_link_JOINT_2";
  joint2.request.effort = r(2);
  joint2.request.duration= duration;
  if ((client.call(joint0)) && (client.call(joint1)) && (client.call(joint2)))
  {
    ROS_INFO("published rpy control_effort: roll=%f pitch=%f yaw=%f", r(0), r(1), r(2));
    ROS_INFO("published rpy angle: roll=%f pitch=%f yaw=%f", v(0), v(0), v(0));
  }

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "apply_joint_effort_client");
  matrixInit();
  ros::NodeHandle n;
  ros::Timer timer1 = n.createTimer(ros::Duration(0.01), controlCallback);
  client = n.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
  yaw_sub = n.subscribe("/ballbot_yaw/control_effort", 1000, yawCallback);
  pitch_sub = n.subscribe("/ballbot_pitch/control_effort", 1000, pitchCallback);
  roll_sub = n.subscribe("/ballbot_roll/control_effort", 1000, rollCallback);
  //ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */

  ros::spin();
  
  return 0;
 }

