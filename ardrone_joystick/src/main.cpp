/*
* This ROS package is adapted from Nikolas Engelhard's package used to teleoperate
* a Parrot AR.Drone using a PS3 controller. This package is modified to use a Logitech
* Gamepad F310 and can be built with catkin (as opposed to the previous rosbuild system).
* This package also implements the ability to call preset flight animations for a backflip
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

	void oculusCallback(const geometry_msgs::Quaternion::ConstPtr& oculus) {
		if (auto_pilot_on) {return;}
		double scale = 1.0;
		double deadzone = 0.25;
		tf::Quaternion q(oculus->x, oculus->y, oculus->z, oculus->w);
		double roll, pitch, yaw;
		// tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
		tf::Matrix3x3(q).getRPY(pitch, yaw, roll); // roll, yaw negative
		roll = -roll;
		if (!oculusCalib) {
			OCULUS_ROLL_OFFSET = roll;
			OCULUS_PITCH_OFFSET = pitch;
			OCULUS_YAW_OFFSET = yaw;
			oculusCalib = true;
		}
		roll -= OCULUS_ROLL_OFFSET;
		pitch -= OCULUS_PITCH_OFFSET;
		yaw -= OCULUS_YAW_OFFSET;
		pitch *= scale;
		yaw *= scale;
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
		twist.angular.z = yaw;
		twist.linear.x = -.5*pitch;
		// ROS_INFO("Oculus Roll = %f", roll);
	}

	void myoIMUCb(const sensor_msgs::Imu::ConstPtr& myo) {
		if (auto_pilot_on) {return;}
		double takeoff_cutoff = 45.0;
		tf::Quaternion q(myo->orientation.x, myo->orientation.y, myo->orientation.z, myo->orientation.w);
		geometry_msgs::Vector3 v;
		tf::Matrix3x3(q).getRPY(v.x, v.y, v.z);
		v.x *= 180.0/M_PI;
		v.y *= 180.0/M_PI;
		v.z *= 180.0/M_PI;
		myo_imu_pub.publish(v);

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
		if (abs(v.y) < 10.0) {
			v.y = 0.0;
		}
		twist.linear.z = -v.y * M_PI/180;
	}

	void myoEMGCb(const ros_myo::EmgArray::ConstPtr& myo) {
		if (auto_pilot_on) {return;}
		double K = 0.003;
		double aveRight = (myo->data[0]+myo->data[1]+myo->data[2]+myo->data[3])/4.0;
		double aveLeft = (myo->data[4]+myo->data[5]+myo->data[6]+myo->data[7])/4.0;
		double ave = (aveLeft + aveRight) / 2.0;

		// if (ave > 700.0) {
		// 	twist.linear.x = 1;
		// }
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

	// AUTOPILOT
	void posCb(const tum_ardrone::filter_stateConstPtr est) {
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
	void setWayPoint(tum_ardrone::filter_stateConstPtr p) {
		std::ostringstream ss;
		ss << "c goto " << p->x << " " << p->y << " " << p->z << " " << p->yaw;
		std_msgs::String str;
		str.data = ss.str();
		pub_pos.publish(str);
	}

	bool pos_eq(tum_ardrone::filter_state::ConstPtr p1, tum_ardrone::filter_state::ConstPtr p2) {
		double dist = sqrt(pow(p1->x - p2->x,2) + pow(p1->y - p2->y,2) + pow(p1->z - p2->z,2) + pow(p1->yaw - p2->yaw,2));
		return dist < 0.25;
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
