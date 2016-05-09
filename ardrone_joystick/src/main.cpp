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

	std_srvs::Empty srv_empty;
	ardrone_autonomy::FlightAnim srv_flight;

	ros::NodeHandle nh_;
	geometry_msgs::Twist twist;
	ros::ServiceClient srv_cl_cam;

	ros::ServiceClient srv_cl_anim;

	void joyCb(const sensor_msgs::JoyConstPtr joy_msg){
		if (auto_pilot_on) {return;}
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
			// twist.linear.z = scale*joy_msg->axes[4];
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
			// twist.linear.z = scale*joy_msg->axes[3];
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
			land();
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
			system("rosservice call /ardrone/togglecam");
			// if (!srv_cl_cam.call(srv_empty))  ROS_INFO("Failed to toggle Camera");
		}
		cam_toggle_pressed_in_last_msg = cam_toggle_pressed;

		if (!anim_toggle_pressed_in_last_msg && anim_toggle_pressed){
			ROS_INFO("Executing Backflip");
			srv_flight.request.type = 17;
			srv_flight.request.duration = 0;
			system("rosservice call /ardrone/setflightanimation 17 0");
			// if (!srv_cl_anim.call(srv_flight))  ROS_INFO("Failed to send backflip");
		}
		anim_toggle_pressed_in_last_msg = anim_toggle_pressed;
	}

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
		twist.angular.z = yaw;
		twist.linear.x = -.5*pitch;
		// ROS_INFO("\nRoll = %f\nPitch = %f\nYaw = %f\n----------", scale*roll, scale*pitch, scale*yaw);
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
		// if (est->droneState == 0) { // emergency state
		// 	pub_toggle_state.publish(std_msgs::Empty());
		// 	takeoff();
		// }
		//ROS_INFO("Drone State = %d", est->droneState);
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

		// if (!nav_queue.empty()) {
		// 	if (!waypoint_set) {
		// 		setWayPoint(nav_queue.front());
		// 		waypoint_set = true;
		// 	}
		// 	if (pos_eq(est,nav_queue.front())) {
		// 		ROS_INFO("Hit Target (%.2f,%.2f,%.2f,%.2f)",nav_queue.front()->x,nav_queue.front()->y,nav_queue.front()->z,nav_queue.front()->yaw);
		// 		nav_queue.pop();
		// 		ros::Duration(1.0).sleep();
		// 		waypoint_set = false;
		// 	}
		// }
		// else {
		// 	system("rosservice call /ardrone/setflightanimation 17 0");
		// 	ros::Duration(1.0).sleep();
		// 	land();
		// 	ros::shutdown();
		// }
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
		// ROS_INFO("***EUCLIDIAN DISTANCE = %f", euclid);
		return dist < 0.25;
	}


	TeleopArDrone(){

		twist.linear.x = twist.linear.y = twist.linear.z = 0;
		twist.angular.x = twist.angular.y = twist.angular.z = 0;

		is_flying = false;
		got_first_joy_msg = false;
		oculusCalib = false;

		// joy_sub = nh_.subscribe("/joy", 1,&TeleopArDrone::joyCb, this);
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

		// tum_ardrone::filter_state p;
		// p.x = -0.25,p.y = 0.25, p.z = 1.5, p.yaw = 0.0;
		// tum_ardrone::filter_stateConstPtr p_(new tum_ardrone::filter_state(p));
		// nav_queue.push(p_);
		// p.x = 0.25,p.y = -0.25, p.z = 1.5, p.yaw = 0.0;
		// tum_ardrone::filter_stateConstPtr p2_(new tum_ardrone::filter_state(p));
		// nav_queue.push(p2_);
		// p.x = -0.25,p.y = -0.25, p.z = 1.5, p.yaw = 0.0;
		// tum_ardrone::filter_stateConstPtr p3_(new tum_ardrone::filter_state(p));
		// nav_queue.push(p3_);
		// p.x = 0.25,p.y = 0.25, p.z = 1.5, p.yaw = 0.0;
		// tum_ardrone::filter_stateConstPtr p4_(new tum_ardrone::filter_state(p));
		// nav_queue.push(p4_);
		// p.x = 0.0,p.y = 0.0, p.z = 0.25, p.yaw = 0.0;
		// tum_ardrone::filter_stateConstPtr p5_(new tum_ardrone::filter_state(p));
		// nav_queue.push(p5_);


		// autopilot
		pos_sub = nh_.subscribe("/ardrone/predictedPose", 1, &TeleopArDrone::posCb, this);
		pub_pos			  = nh_.advertise<std_msgs::String>("/tum_ardrone/com",1);
		auto_pilot_on = false;
		// srv_cl_anim		  = nh_.serviceClient<ardrone_autonomy::FlightAnim>("/ardrone/setflightanimation",1);
	}

	void send_cmd_vel(){
		if (auto_pilot_on) {return;}
    	pub_vel.publish(twist);
  	}
  	void takeoff() {
  		pub_takeoff.publish(std_msgs::Empty());
  	}
  	void land() {
  		// pub_land.publish(std_msgs::Empty());
  		autopilotLand();
  	}
  	void startAutoPilot() {
	  	std_msgs::String str;
	  	str.data = "c start";
	  	pub_pos.publish(str);
		// str.data = "u l Autopilot: Start Controlling";
		str.data = "c autoInit 500 800 4000 0.5";
		pub_pos.publish(str);
		// auto_pilot_on = true;
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

	  	str.data = "c setInitialReachDist 0.2";
	  	pub_pos.publish(str);

	  	str.data = "c setStayWithinDist 0.3";
	  	pub_pos.publish(str);

	  	// str.data = "c start";
	  	// pub_pos.publish(str);

	  	// str.data = "c stop";
	  	// pub_pos.publish(str);
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

	  	// str.data = "c stop";
	  	// pub_pos.publish(str);

	  	// str.data = "u l Autopilot: Start Controlling";
	  	// pub_pos.publish(str);
	  	// std_msgs::String str;
	  	// str.data = "u l New Target: xyz = 0.000, 0.000, 0.500,  yaw=0.000";
	  	// pub_pos.publish(str);;S
	  //	auto_pilot_on = false;
  	}


};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ardrone_teleop");

  ROS_INFO("Started ArDrone joystick-Teleop");
  ROS_INFO("Press L1 to toggle emergency-state");
  ROS_INFO("Press and hold L2 for takeoff");
  ROS_INFO("Press 'select' to choose camera");

  // system("rosrun ros_myo myo-rawNode.py");
  TeleopArDrone teleop;
  ros::Rate pub_rate(PUBLISH_FREQ);

  // teleop.startAutoPilot();
  // ros::Duration(5.0).sleep();
  // teleop.takeoff();
  // // ros::Duration(1.5).sleep();
  // teleop.startAutoPilot();
  // ros::Duration(5.0).sleep();
  // teleop.land();
  // teleop.endAutoPilot();

  cout << "Hit Enter after Calibration: " << endl;
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
