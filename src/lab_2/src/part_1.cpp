#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <cmath>
#define PI 3.141593

ros::Publisher vel_pub;
ros::Publisher reset_pub;
double linearX = 0;
double angularZ = 0;
double xt = -1.5;
double yt = -1.5;
double angleLimit(double theta) {
	if (theta > PI)
		return theta-2*PI;
	if (theta < -PI)
		return theta+2*PI;
	return theta;
}
void odomCallback(const nav_msgs::Odometry::ConstPtr& data)
{
	double vmax = 0.3; // m/s
	double arm = 2; // rad/s
	double stopRdist = 0.03; // meters
	double stopVdist = 0.01; // meters, must be less than stopRdist
	double vScale = 2;
	double aScale = 6;
	double stopRang = 0.5; // degrees
	double xr = data->pose.pose.position.x;
	double yr = data->pose.pose.position.y;
	double tr = 2*asin(data->pose.pose.orientation.z);
	double aerr = angleLimit(atan2(yt-yr,xt-xr)-tr);
	double vr = sqrt((xt-xr)*(xt-xr)+(yt-yr)*(yt-yr))*2;
	std::cout << xr << " " << yr << " " << tr/PI << " " << aerr/PI << "\n";
	if (vr > vmax) vr = vmax;
	if (vr < vScale*stopVdist)
		linearX = 0;
	else
		linearX = vr*(1-2*abs(aerr)/PI);
	if (aerr > PI/2) aerr -= PI;
	else if (aerr < -PI/2) aerr += PI;
	double ar = aerr/PI*6;
	if (ar > arm)
		angularZ = arm;
	else if (ar < -arm)
		angularZ = -arm;
	else if ((aerr > -stopRang*PI/180 && aerr < stopRang*PI/180)||vr<stopRdist*vScale)
		angularZ = 0;
	else
		angularZ = ar;
	std::cout << vr << " " << linearX << " " << angularZ <<" "<<aerr/PI << "\n\n";
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlebot_turn");
	ros::NodeHandle nh;
	reset_pub = nh.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1, true);
	reset_pub.publish(std_msgs::Empty());
	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1, true);
	ros::Subscriber sub4 = nh.subscribe("/odom", 10, odomCallback);
	ros::Rate loop_rate(50);
	while (ros::ok()){
		loop_rate.sleep();
		geometry_msgs::Twist vel;
		vel.linear.x = linearX;//linear velocity(m/s)
		vel.angular.z = angularZ;//angular velocity(rad/s)
		vel_pub.publish(vel);
		ros::spinOnce();
	}
	return 0;
}