#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/SensorState.h>
#include <nav_msgs/Odometry.h>

ros::Publisher vel_pub;
double constantX = 0.3;
double targetX = 0;
double constantZ = 0;
double linearX = 0;
double angularZ = 0;
double xspeed;
double zrot;
ros::Time bstart;
bool manual = true;
bool bumper = false;


void joyCallback(const sensor_msgs::Joy::ConstPtr& data)
{
	
	if (data->axes[5] != 0 && !bumper) {
		manual = !manual;
		targetX = 0.3;
		constantX = 0.3;
	}
	if (data->buttons[0] != 0) {
		targetX = 0.5;
		constantX = 0.5;
	} else if (data->buttons[1] != 0) {
		targetX = 0.3;
		constantX = 0.3;
	} else if (data->buttons[2] != 0) {
		targetX = 0;
		constantX = 0;
	} else if (data->buttons[3] != 0) {
		targetX = 0.6;
		constantX = 0.6;
	}
	if (!bumper && manual) {
		targetX = constantX*data->axes[1];//linear velocity(m/s)
		constantZ = 2*data->axes[0];
	}
}
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& data)
{
	if (data->state == 1 && !bumper) {
		bumper = true;
		bstart = ros::Time::now();
	} 
}
double previous_error = 0;
double integral = 0;
ros::Time lasttime;
void odomCallback(const nav_msgs::Odometry::ConstPtr& data)
{
/*	xspeed = data->twist.twist.linear.x;
	zrot = data->twist.twist.angular.z;
	double error = targetX-xspeed;
	integral = integral + error*(ros::Time::now()-lasttime).toSec();
	double de = (error-previous_error)/(ros::Time::now()-lasttime).toSec();
	double output = 6*error+0.2*integral+0.05*de;
	//if (!bumper) {
	linearX += (ros::Time::now()-lasttime).toSec()*output;
	angularZ = constantZ;
	std::cout << "Odom data: "<<linearX <<'\n';
	std::cout<< xspeed<<'\n';
	std::cout << zrot<<"\n\n";
	//}
	previous_error = error;
	lasttime = ros::Time::now();*/
}
int lastL;
int lastR;
void encCallback(const kobuki_msgs::SensorState::ConstPtr& data)
{
	if (lasttime.toSec() == 0) {
		lasttime = ros::Time::now();
		lastL = data->left_encoder;
		lastR = data->right_encoder;
		return;
	}
	int eL = data->left_encoder;
	int eR = data->right_encoder;
	int dx = eL-lastL;
	int dy = eR-lastR;
	if (dx > 32000) dx -= 1<<16;
	if (dx < -32000) dx += 1<<16;
	if (dy > 32000) dy -= 1<<16;
	if (dy < -32000) dy += 1<<16;
	std::cout << "Encoder data: "<<(dx+dy)*0.0021127<<"\n";
	std::cout << (dy-dx)*0.01789 << '\n';
	std::cout << dx << '\n';
	std::cout << dy << "\n\n";
	lastL = eL;
	lastR = eR;
	xspeed = (dx+dy)*0.0021127;
	zrot = (dy-dx)*0.01789;
	double error = targetX-xspeed;
	integral = integral + error*(ros::Time::now()-lasttime).toSec();
	double de = (error-previous_error)/(ros::Time::now()-lasttime).toSec();
	double output = 6*error+0.2*integral+0.05*de;
	//if (!bumper) {
	linearX += (ros::Time::now()-lasttime).toSec()*output;
	angularZ = constantZ;
	std::cout << "Odom data: "<<linearX <<'\n';
	std::cout<< xspeed<<'\n';
	std::cout << error<<"\n\n";
	//}
	previous_error = error;
	lasttime = ros::Time::now();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlebot_turn");
	ros::NodeHandle nh;
	lasttime = ros::Time(0);
	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1, true);
	ros::Subscriber sub2 = nh.subscribe("/joy", 10, joyCallback);
	ros::Subscriber sub3 = nh.subscribe("/mobile_base/events/bumper", 1, bumperCallback);
	ros::Subscriber sub4 = nh.subscribe("/odom", 10, odomCallback);
	ros::Subscriber sub5 = nh.subscribe("/mobile_base/sensors/core", 10, encCallback);
	ros::Rate loop_rate(1000);
	std::srand((int)ros::Time::now().toSec());
	double turntime = 0.75+(1.75*rand())/RAND_MAX;
	while (ros::ok()){
		loop_rate.sleep();
		if ((ros::Time::now()-bstart).toSec()<0.7 && bumper) {
			targetX = -0.3;
			angularZ = 0;
		} else if ((ros::Time::now()-bstart).toSec()>0.7 && (ros::Time::now()-bstart).toSec()<0.5+turntime && bumper) {
			targetX = 0;
			angularZ = 2;

		} else if (bumper) {
			targetX = manual ? 0 : constantX;
			angularZ = 0;
			turntime = 0.75+(1.75*rand())/RAND_MAX;
			bumper = false;
		}
		geometry_msgs::Twist vel;
	vel.linear.x = linearX;//linear velocity(m/s)
	vel.angular.z = angularZ;//angular velocity(rad/s)
	vel_pub.publish(vel);
	ros::spinOnce();
}
return 0;
}

