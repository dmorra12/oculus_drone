#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

// Callback function to generate transform based on complete twist message
void twistCallback(const geometry_msgs::Twist& twist) {
	// Transform and broadcaster variables
	static tf::TransformBroadcaster br;
	static tf::Transform teleop_3d(tf::Transform::getIdentity());
	
	tf::Transform teleop_3d_delta;

	// Timer variables to determine time step in between messages
	static ros::Time prev = ros::Time::now();
	ros::Time now = ros::Time::now();
	ros::Duration delta_t = now - prev;

	// Homogeneous matrix to represent transformation
	Eigen::MatrixXd T(4,4);

	// Populate homogeneous matrix with values from twist
	// Rotation matrix is made using skew-symmetric form of angular velocities
	T << 0, -twist.angular.z, twist.angular.y, twist.linear.x,
		twist.angular.z, 0, -twist.angular.x, twist.linear.y,
		-twist.angular.y, twist.angular.x, 0, twist.linear.z,
		0, 0, 0, 0;
	
	// Local transformation is the matrix exponential of the homogeneous 
	// velocity matrix multiplied by the scalar valued time step
	T = (T*delta_t.toSec()).exp();

	// Set components of local transformation using calculated matrix
	teleop_3d_delta.setOrigin(tf::Vector3(T(0,3), T(1,3), T(2,3)));
	teleop_3d_delta.setBasis(tf::Matrix3x3(	T(0,0),T(0,1),T(0,2),
										T(1,0),T(1,1),T(1,2),
										T(2,0),T(2,1),T(2,2)));


	/* 
	 * Control movements relative to body frame
	 * For movements relative to world frame change line to:
	 * teleop_3d = teleop_3d_delta * teleop_3d;
	 */
	teleop_3d = teleop_3d * teleop_3d_delta; // body frame

	// broadcast corresponding transform
	br.sendTransform(tf::StampedTransform(teleop_3d, ros::Time::now(), "world", "teleop_3d"));

	// update previous time
	prev = now;
}

// Node to publish transform when twist message is received
int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop_3d");

	ros::NodeHandle node;

	ros::Subscriber sub = node.subscribe("/turtle1/cmd_vel", 10, &twistCallback);

	ros::spin();
	return 0;
}