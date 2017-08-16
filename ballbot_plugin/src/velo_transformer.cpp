#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <Eigen/Dense>

Eigen::MatrixXd m(3,3);
Eigen::Vector3d r;
Eigen::Vector3d v;

ros::Subscriber yaw_sub;
ros::Subscriber pitch_sub;
ros::Subscriber roll_sub;
ros::Publisher motor1_pub;
ros::Publisher motor2_pub;
ros::Publisher motor3_pub;
double yaw_control_effort, pitch_control_effort, roll_control_effort;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

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

     std_msgs::Float64 motor1_control_effort,motor2_control_effort,motor3_control_effort;
     motor1_control_effort.data = r(0);
     motor2_control_effort.data = r(1);
     motor3_control_effort.data = r(2);
     motor1_pub.publish(motor1_control_effort);
     motor2_pub.publish(motor2_control_effort);
     motor3_pub.publish(motor3_control_effort);
     //ROS_INFO("published rpy control_effort: roll=%f pitch=%f yaw=%f", r(0), r(1), r(2));
     
}

int main(int argc, char **argv)
{
  matrixInit();


  ros::init(argc, argv, "velo_transformer");

  ros::NodeHandle n;

  ros::Timer timer1 = n.createTimer(ros::Duration(0.01), controlCallback);
  yaw_sub = n.subscribe("/ballbot_yaw/control_effort", 1000, yawCallback);
  pitch_sub = n.subscribe("/ballbot_pitch/control_effort", 1000, pitchCallback);
  roll_sub = n.subscribe("/ballbot_roll/control_effort", 1000, rollCallback);
  motor1_pub = n.advertise<std_msgs::Float64>("/ballbot/joint0_vel_cmd", 1000);
  motor2_pub = n.advertise<std_msgs::Float64>("/ballbot/joint1_vel_cmd", 1000);
  motor3_pub = n.advertise<std_msgs::Float64>("/ballbot/joint2_vel_cmd", 1000);
  ros::spin();

  return 0;
}
