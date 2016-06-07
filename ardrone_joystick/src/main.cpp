/*
* This ROS package is used to teleoperate a Parrot AR.Drone 2.0 using
* physical input devices: an Oculus Rift DK1 and a Myo Gesture Control
* Armband.
* 
* Author: David Morra
*/
#include <iostream>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "ardrone_autonomy/FlightAnim.h"

// myo imu
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <ros_myo/EmgArray.h>

// oculus
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

// autopilot
#include <tum_ardrone/filter_state.h>
#include <vector>
#include <queue>
#include <std_msgs/String.h>

#include <stdlib.h>

const int PUBLISH_FREQ = 50;

using namespace std;

struct TeleopArDrone
{
	double OCULUS_YAW_OFFSET;
	double OCULUS_ROLL_OFFSET;
	double OCULUS_PITCH_OFFSET;

	bool oculusCalib;
	string controller_type;
	ros::Subscriber joy_sub;
	ros::Subscriber oculus_sub;
	ros::Subscriber myo_imu_sub;
	ros::Subscriber myo_emg_sub;
	ros::Publisher pub_takeoff, pub_land, pub_toggle_state, pub_vel, myo_imu_pub;
	
	// autopilot
	ros::Subscriber pos_sub;
	ros::Publisher pub_pos;
	queue<tum_ardrone::filter_stateConstPtr> nav_queue;
	bool waypoint_set;
	bool auto_pilot_on;

	bool got_first_joy_msg;

	bool is_flying;
	bool toggle_pressed_in_last_msg;
	bool cam_toggle_pressed_in_last_msg;
	bool anim_toggle_pressed_in_last_msg;

	bool dead_man_pressed;
	bool emergency_toggle_pressed;
	bool cam_toggle_pressed;
	bool anim_toggle_pressed;

	bool executing_flip;

	std_srvs::Empty srv_empty;
	ardrone_autonomy::FlightAnim srv_flight;

	ros::NodeHandle nh_;
	geometry_msgs::Twist twist;
	ros::ServiceClient srv_cl_cam;

	/*
	* Callback function to modify drone command velocity using Quaternion message
	* broadcast by Oculus headset.
	*/
	void oculusCallback(const geometry_msgs::Quaternion::ConstPtr& oculus) {
		// Quick/dirty solution to block out velocity writing while autopilot is active
		if (auto_pilot_on) {return;}
		double scale = 1.0;
		double deadzone = 0.25;

		// translate Quaternion message to TF quaterion
		tf::Quaternion q(oculus->x, oculus->y, oculus->z, oculus->w);
		double roll, pitch, yaw;

		// convert quaternion to RPY angle representation
		tf::Matrix3x3(q).getRPY(pitch, yaw, roll); // roll negative
		roll = -roll;

		// First time function is called, zero calibration orientation of headset
		if (!oculusCalib) {
			OCULUS_ROLL_OFFSET = roll;
			OCULUS_PITCH_OFFSET = pitch;
			OCULUS_YAW_OFFSET = yaw;
			oculusCalib = true;
		}
		// shift and scale rotation angles for velocity commands
		roll -= OCULUS_ROLL_OFFSET;
		pitch -= OCULUS_PITCH_OFFSET;
		yaw -= OCULUS_YAW_OFFSET;
		pitch *= scale;
		yaw *= scale;

		// implement "deadzone" for easier user interaction
		if (abs(yaw) < deadzone) {
			yaw = 0;
		}
		if (abs(pitch) < deadzone) {
			pitch = 0;
		}
		if (abs(roll) < deadzone) {
			roll = 0;
			executing_flip = false;
		}

		// ensure flip preset animation is only executed once
		if (!executing_flip) {
			if (roll > 0.7) {
				// flip right
				system("rosservice call /ardrone/setflightanimation 19 0");
				executing_flip = true;
			}
			if (roll < -0.7) {
				// flip left
				system("rosservice call /ardrone/setflightanimation 18 0");
				executing_flip = true;
			}
		}
		// update command velocities 
		twist.angular.z = yaw;
		twist.linear.x = -.5*pitch;
	}

	/*
	* Callback function to modify drone command velocity using Quaternion message
	* broadcast by Myo armband.
	*/
	void myoIMUCb(const sensor_msgs::Imu::ConstPtr& myo) {
		// Quick/dirty solution to block out velocity writing while autopilot is active
		if (auto_pilot_on) {return;}
		double takeoff_cutoff = 45.0;

		// translate Quaternion message to TF quaterion
		tf::Quaternion q(myo->orientation.x, myo->orientation.y, myo->orientation.z, myo->orientation.w);
		geometry_msgs::Vector3 v;
		tf::Matrix3x3(q).getRPY(v.x, v.y, v.z);
		v.x *= 180.0/M_PI;
		v.y *= 180.0/M_PI;
		v.z *= 180.0/M_PI;

		// rotation degree angles published in message for debugging purposes
		myo_imu_pub.publish(v);

		// use deadman switch to ensure failsafe behavior of vehicle landing
		dead_man_pressed = v.y < takeoff_cutoff;
		if (!is_flying && dead_man_pressed){
			ROS_INFO("L1 was pressed, Taking off!");
			pub_takeoff.publish(std_msgs::Empty());
			is_flying = true;
		}
		if (is_flying && !dead_man_pressed){
			ROS_INFO("L1 was released, landing");
			land();
			is_flying = false;
		}
		// implement deadzone for easier user operation
		if (abs(v.y) < 10.0) {
			v.y = 0.0;
		}

		// update vertical command velocity
		twist.linear.z = -v.y * M_PI/180;
	}

	/*
	* Callback function to modify drone command velocity using EMG sensor readings
	* array message broadcast by Myo armband
	*/
	void myoEMGCb(const ros_myo::EmgArray::ConstPtr& myo) {
		// Quick/dirty solution to block out velocity writing while autopilot is active
		if (auto_pilot_on) {return;}
		double K = 0.003;
		double aveRight = (myo->data[0]+myo->data[1]+myo->data[2]+myo->data[3])/4.0;
		double aveLeft = (myo->data[4]+myo->data[5]+myo->data[6]+myo->data[7])/4.0;
		double ave = (aveLeft + aveRight) / 2.0;

		// Translate left and right horizonatally if wrist motion is sufficiently large
		if (aveLeft > (aveRight + 200.0)) {
			twist.linear.y = K*ave;
		}
		else if (aveRight > (aveLeft + 200.0)) {
			twist.linear.y = -K*ave;
		}
		else {
			twist.linear.y = 0;
		}
	}

	/*
	* Callback function to handle autopilot activation/blocking and deactivation
	* based on drone state message broadcast by the tum_ardrone package
	*/
	void posCb(const tum_ardrone::filter_stateConstPtr est) {
		// if the drone is in the landed state, ensure autopilot does not block other functions
		if (est->droneState == 2) {
			if (auto_pilot_on) {
				ROS_INFO("Turning off autopilot");
				auto_pilot_on = false;
				std_msgs::String str;
				str.data = "c stop";
		  		pub_pos.publish(str);
		  	}
		} // clear autopilot blocking once landed
		if (!auto_pilot_on) {
			return;
		}
	
	}

	TeleopArDrone(){

		twist.linear.x = twist.linear.y = twist.linear.z = 0;
		twist.angular.x = twist.angular.y = twist.angular.z = 0;

		is_flying = false;
		got_first_joy_msg = false;
		oculusCalib = false;
		executing_flip = false;

		oculus_sub = nh_.subscribe("oculus/orientation", 1, &TeleopArDrone::oculusCallback, this);
		myo_imu_sub = nh_.subscribe("/myo_imu", 1, &TeleopArDrone::myoIMUCb, this);
		myo_emg_sub = nh_.subscribe("/myo_emg", 1, &TeleopArDrone::myoEMGCb, this);
		toggle_pressed_in_last_msg = cam_toggle_pressed_in_last_msg = false;

		myo_imu_pub = nh_.advertise<geometry_msgs::Vector3>("/myo_imu/RPY", 1);
		pub_takeoff       = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
		pub_land          = nh_.advertise<std_msgs::Empty>("/ardrone/land",1);
		pub_toggle_state  = nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);
		pub_vel           = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
		srv_cl_cam        = nh_.serviceClient<std_srvs::Empty>("/ardrone/togglecam",1);

		// autopilot
		pos_sub = nh_.subscribe("/ardrone/predictedPose", 1, &TeleopArDrone::posCb, this);
		pub_pos			  = nh_.advertise<std_msgs::String>("/tum_ardrone/com",1);
		auto_pilot_on = false;	
	}

	// function to broadcast updated command velocity
	void send_cmd_vel(){
		if (auto_pilot_on) {return;}
    	pub_vel.publish(twist);
  	}

  	void takeoff() {
  		pub_takeoff.publish(std_msgs::Empty());
  	}

  	void land() {
  		autopilotLand();
  	}

  	/************************************************************************************
  	* The following functions are implemented based on reverse engineering the internal
  	* communication protocol utilized by the tum_ardrone navigation and control package. 
	*************************************************************************************/

  	void startAutoPilot() {
	  	std_msgs::String str;
	  	str.data = "c start";
	  	pub_pos.publish(str);
		str.data = "c autoInit 500 800 4000 0.5";
		pub_pos.publish(str);
  	}

  	void endAutoPilot() {
  		std_msgs::String str;
	  	str.data = "c stop";
	  	pub_pos.publish(str);
		str.data = "u l Autopilot: Stop Controlling";
		pub_pos.publish(str);
		auto_pilot_on = false;
  	}

  	void setAutopilotInitialPose() {
  		auto_pilot_on = true;
  		std_msgs::String str;

  		str.data = "c clearCommands";
	  	pub_pos.publish(str);

	  	str.data = "c setReference $POSE$";
	  	pub_pos.publish(str);

	  	str.data = "c setInitialReachDist 0.4";
	  	pub_pos.publish(str);

	  	str.data = "c setStayWithinDist 0.5";
	  	pub_pos.publish(str);

	  	str.data = "c setStayTime 0.1";
	  	pub_pos.publish(str);

	  	ROS_INFO("Set Initial Pose");
	  	auto_pilot_on = false;
  	}

  	void autopilotLand() {
  		auto_pilot_on = true;
  		ROS_INFO("Landing Using Autopilot");
	  	std_msgs::String str;

	  	str.data = "c goto 0 0 0.5 0";
	  	pub_pos.publish(str);

	  	ROS_INFO("%s",str.data.c_str());

	  	str.data = "c land";
	  	pub_pos.publish(str);

	  	ROS_INFO("%s",str.data.c_str());

	  	str.data = "c start";
	  	pub_pos.publish(str);

	  	ROS_INFO("%s",str.data.c_str());
  	}
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ardrone_teleop");

  TeleopArDrone teleop;
  ros::Rate pub_rate(PUBLISH_FREQ);

  // Wait for the user to perform manual PTAM calibration step
  cout << "Enter a character to confirm calibration: " << endl;
  char c;
  cin >> c;
  ROS_INFO("GOT CALIBRATION");
  teleop.setAutopilotInitialPose();

  while (teleop.nh_.ok())
  {
    ros::spinOnce();
    teleop.send_cmd_vel();
    pub_rate.sleep();
  }
  teleop.land();

  return 0;
}
