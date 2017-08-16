/****************************************************************************

Conversion from a quaternion to roll, pitch and yaw.

Nodes:
subscribed /rotation_quaternion (message of type geometry_msgs::Quaternion)
published /rpy_angles (message oftype geometry_msgs::Vector3.h)

****************************************************************************/

#include "ros/ros.h"
#include <math.h>
#include "sensor_msgs/Imu.h"
#include <std_msgs/Float64.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

// Here I use global publisher and subscriber, since I want to access the
// publisher in the function MsgCallback:
ros::Publisher yaw_state_pub;
ros::Publisher pitch_state_pub;
ros::Publisher roll_state_pub;
ros::Subscriber quat_subscriber;

// Function for conversion of quaternion to roll pitch and yaw. The angles
// are published here too.
void MsgCallback(const sensor_msgs::Imu msg)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    std_msgs::Float64 roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll.data, pitch.data, yaw.data);

    // the found angles are written in a geometry_msgs::Vector3
/*    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;
*/
    yaw.data *=180/M_PI;
    pitch.data *=180/M_PI;
    roll.data *=180/M_PI;
    yaw_state_pub.publish(yaw);
    pitch_state_pub.publish(pitch);
    roll_state_pub.publish(roll);
    // this Vector is then published:
    //rpy_publisher.publish(rpy);
    //ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", roll.data, pitch.data, yaw.data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_transformer");
    ros::NodeHandle n;
    //rpy_publisher = n.advertise<geometry_msgs::Vector3>("imu_angles", 1000);
    yaw_state_pub = n.advertise<std_msgs::Float64>("/ballbot_yaw/state", 1000);
    pitch_state_pub = n.advertise<std_msgs::Float64>("/ballbot_pitch/state", 1000);
    roll_state_pub = n.advertise<std_msgs::Float64>("/ballbot_roll/state", 1000);
    
    quat_subscriber = n.subscribe("imu", 1000, MsgCallback);

    // check for incoming quaternions untill ctrl+c is pressed
    ROS_INFO("waiting for quaternion");
    ros::spin();
    return 0;
}
