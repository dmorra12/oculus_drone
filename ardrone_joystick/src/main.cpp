/*
* This ROS package is adapted from Nikolas Engelhard's package used to teleoperate
* a Parrot AR.Drone using a PS3 controller. This package is modified to use a Logitech
* Gamepad F310 and can be built with catkin (as opposed to the previous rosbuild system).
* This package also implements the ability to call preset flight animations for a backflip
* 
* Author: David Morra
*/


#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "ardrone_autonomy/FlightAnim.h"

#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

const int PUBLISH_FREQ = 50;

using namespace std;

struct TeleopArDrone
{
	string controller_type;
	ros::Subscriber joy_sub;
	ros::Subscriber oculus_sub;
	ros::Publisher pub_takeoff, pub_land, pub_toggle_state, pub_vel;

	bool got_first_joy_msg;

	bool is_flying;
	bool toggle_pressed_in_last_msg;
	bool cam_toggle_pressed_in_last_msg;
	bool anim_toggle_pressed_in_last_msg;

	bool dead_man_pressed;
	bool emergency_toggle_pressed;
	bool cam_toggle_pressed;
	bool anim_toggle_pressed;

	std_srvs::Empty srv_empty;
	ardrone_autonomy::FlightAnim srv_flight;

	ros::NodeHandle nh_;
	geometry_msgs::Twist twist;
	ros::ServiceClient srv_cl_cam;

	ros::ServiceClient srv_cl_anim;

	void joyCb(const sensor_msgs::JoyConstPtr joy_msg){
		if (!got_first_joy_msg){
			ROS_INFO("Found joystick with %zu buttons and %zu axes", joy_msg->buttons.size(), joy_msg->axes.size());
			
			if (joy_msg->buttons.size() == 11 || joy_msg->axes.size() == 8) {
				controller_type = "F310";
			}
			else if (joy_msg->buttons.size() == 19 || joy_msg->axes.size() == 27) {
				controller_type = "PS3";
			}
			else {
				ROS_FATAL("This joystick button map is not recognized");
			}
			ROS_INFO("Controller Type: %s", controller_type.c_str());
			got_first_joy_msg = true;
		}
		if (controller_type == "F310") {
			// mapping from joystick to velocity
			float scale = 1;
			twist.linear.x = scale*joy_msg->axes[1];
			twist.linear.y = scale*joy_msg->axes[0];
			twist.linear.z = scale*joy_msg->axes[4];
			// twist.angular.z = scale*joy_msg->axes[3];
			// (L1): dead man switch
			dead_man_pressed = joy_msg->buttons.at(4);
			// (R1): switch emergeny state 
			emergency_toggle_pressed = joy_msg->buttons.at(5);
			// (select): switch camera mode (front/bottom)
			cam_toggle_pressed = joy_msg->buttons.at(6);
			// (X button (in PS3 layout)): backflip
			anim_toggle_pressed = joy_msg->buttons.at(0);
		}
		else if (controller_type == "PS3") {
			// mapping from joystick to velocity
			float scale = 1;
			twist.linear.x = scale*joy_msg->axes[1];
			twist.linear.y = scale*joy_msg->axes[0];
			twist.linear.z = scale*joy_msg->axes[3];
			// twist.angular.z = scale*joy_msg->axes[2];
			// (L1): dead man switch
			dead_man_pressed = joy_msg->buttons.at(10);
			// (R1): switch emergeny state 
			emergency_toggle_pressed = joy_msg->buttons.at(11);
			// (select): switch camera mode (front/bottom)
			cam_toggle_pressed = joy_msg->buttons.at(0);
			// (X button (in PS3 layout)): backflip
			anim_toggle_pressed = joy_msg->buttons.at(14);
		}

		if (!is_flying && dead_man_pressed){
			ROS_INFO("L1 was pressed, Taking off!");
			pub_takeoff.publish(std_msgs::Empty());
			is_flying = true;
		}

		if (is_flying && !dead_man_pressed){
			ROS_INFO("L1 was released, landing");
			pub_land.publish(std_msgs::Empty());
			is_flying = false;
		}

		// toggle only once!
		if (!toggle_pressed_in_last_msg && emergency_toggle_pressed){
			ROS_INFO("Changing emergency status");
			pub_toggle_state.publish(std_msgs::Empty());
		}
		toggle_pressed_in_last_msg = emergency_toggle_pressed;


		if (!cam_toggle_pressed_in_last_msg && cam_toggle_pressed){
			ROS_INFO("Changing Camera");
			if (!srv_cl_cam.call(srv_empty))  ROS_INFO("Failed to toggle Camera");
		}
		cam_toggle_pressed_in_last_msg = cam_toggle_pressed;

		if (!anim_toggle_pressed_in_last_msg && anim_toggle_pressed){
			ROS_INFO("Executing Backflip");
			srv_flight.request.type = 17;
			srv_flight.request.duration = 0;
			if (!srv_cl_anim.call(srv_flight))  ROS_INFO("Failed to send backflip");
		}
		anim_toggle_pressed_in_last_msg = anim_toggle_pressed;
	}

	void oculusCallback(const geometry_msgs::Quaternion::ConstPtr& oculus) {
		tf::Quaternion q(oculus->x, oculus->y, oculus->z, oculus->w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		twist.angular.z = -3.5 * yaw;
	}


	TeleopArDrone(){

		twist.linear.x = twist.linear.y = twist.linear.z = 0;
		twist.angular.x = twist.angular.y = twist.angular.z = 0;

		is_flying = false;
		got_first_joy_msg = false;

		joy_sub = nh_.subscribe("/joy", 1,&TeleopArDrone::joyCb, this);
		oculus_sub = nh_.subscribe("oculus/orientation", 1, &TeleopArDrone::oculusCallback, this);
		toggle_pressed_in_last_msg = cam_toggle_pressed_in_last_msg = false;

		pub_takeoff       = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
		pub_land          = nh_.advertise<std_msgs::Empty>("/ardrone/land",1);
		pub_toggle_state  = nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);
		pub_vel           = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
		srv_cl_cam        = nh_.serviceClient<std_srvs::Empty>("/ardrone/togglecam",1);
		srv_cl_anim		  = nh_.serviceClient<ardrone_autonomy::FlightAnim>("/ardrone/setflightanimation",1);
	}

	void send_cmd_vel(){
    	pub_vel.publish(twist);
  	}


};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ardrone_teleop");

  ROS_INFO("Started ArDrone joystick-Teleop");
  ROS_INFO("Press L1 to toggle emergency-state");
  ROS_INFO("Press and hold L2 for takeoff");
  ROS_INFO("Press 'select' to choose camera");

  TeleopArDrone teleop;
  ros::Rate pub_rate(PUBLISH_FREQ);

  while (teleop.nh_.ok())
  {
    ros::spinOnce();
    teleop.send_cmd_vel();
    pub_rate.sleep();
  }

  return 0;
}
