// joy teleop turtlesim example 2015-02-08 LLW
// Modified: David Morra
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <unistd.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

// Define scalar coefficients for Linear and Angular velocities from
// joystick input
#define LINEAR_COF 2.0
#define ANGULAR_COF 1.5

// global variable for Twist message specifying 6-axis command velocity
static geometry_msgs::Twist command_velocity;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  /****** Linear Controller Inputs ******/
  command_velocity.linear.x = LINEAR_COF * joy->axes[1];
  command_velocity.linear.y = LINEAR_COF * joy->axes[0];
  command_velocity.linear.z = LINEAR_COF * joy->axes[7];

  /****** Angular Controller Inputs ******/
  command_velocity.angular.x = ANGULAR_COF * joy->axes[3];
  command_velocity.angular.y = ANGULAR_COF * joy->axes[4];
  command_velocity.angular.z = ANGULAR_COF * (joy->axes[5] - joy->axes[2]);
}

void oculusCallback(const geometry_msgs::Quaternion::ConstPtr& oculus) {
	tf::Quaternion q(oculus->x, oculus->y, oculus->z, oculus->w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	command_velocity.angular.z = 3.5 * yaw;
}


int main(int argc, char** argv)
{

  // init ros
  ros::init(argc, argv, "turtle_teleop_joy");

  // create node handle
  ros::NodeHandle node;

  // advertise topic that this node will publish
  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // subcscribe to joy topic
  ros::Subscriber sub = node.subscribe("joy", 10, &joyCallback);

  ros::Subscriber oculus_sub = node.subscribe("oculus/orientation", 10, &oculusCallback);

  // rate at which to publish turtle velocity twist message
  ros::Rate rate(20);

  // publish twist message at specified rate while node is running
  while (node.ok()) {
    turtle_vel.publish(command_velocity);

    ros::spinOnce();

    rate.sleep();
  }
  
  return 0;

};
