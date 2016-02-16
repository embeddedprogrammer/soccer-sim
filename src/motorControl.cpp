#include "motorControl.h"
#include <stdio.h>
#include <math.h>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "std_msgs/Float64.h"

using namespace std;
using namespace Eigen;

ros::Publisher motor_pub1;
ros::Publisher motor_pub2;
ros::Publisher motor_pub3;

// Everything is in meters.
#define WHEEL_RADIUS 0.03
#define ROBOT_RADIUS 0.1
#define KICKER_DIST 0.0635

Matrix3d M(3, 3);

void motorControl_driveMotorWithSignedSpeed(int wheelId, long qSpeed)
{
	std_msgs::Float64 msg;
	msg.data = -((double)qSpeed);
	if(wheelId == 0)
		motor_pub3.publish(msg);
	else if(wheelId == 1)
		motor_pub2.publish(msg);
	else if(wheelId == 2)
		motor_pub1.publish(msg);
}

void motorControl_killMotors()
{
	motorControl_driveMotorWithSignedSpeed(0, 0);
	motorControl_driveMotorWithSignedSpeed(1, 0);
	motorControl_driveMotorWithSignedSpeed(2, 0);
}

void motorControl_calcBodyFrameVectors()
{
	Vector3d wheelAngles;
	wheelAngles << -90, 30, 150;
	wheelAngles *= M_PI / 180;
	M << cos(wheelAngles(0)), -sin(wheelAngles(0)), ROBOT_RADIUS, 
		 cos(wheelAngles(1)), -sin(wheelAngles(1)), ROBOT_RADIUS, 
		 cos(wheelAngles(2)), -sin(wheelAngles(2)), ROBOT_RADIUS;
	M /= WHEEL_RADIUS;
}

void motorControl_init(ros::NodeHandle nh)
{
	motorControl_calcBodyFrameVectors();
	motor_pub1 = nh.advertise<std_msgs::Float64>("/zzbot/joint1_velocity_controller/command", 5);
	motor_pub2 = nh.advertise<std_msgs::Float64>("/zzbot/joint2_velocity_controller/command", 5);
	motor_pub3 = nh.advertise<std_msgs::Float64>("/zzbot/joint3_velocity_controller/command", 5);
}

void motorControl_driveMotorsAtSpeed(Vector3d motorVelocities)
{
	motorControl_driveMotorWithSignedSpeed(0, motorVelocities(0));
	motorControl_driveMotorWithSignedSpeed(1, motorVelocities(1));
	motorControl_driveMotorWithSignedSpeed(2, motorVelocities(2));
}

void motorControl_moveRobotBodyVelocities(Vector3d v_body)
{
	Vector3d motorVelocities = M * v_body;
	motorControl_driveMotorsAtSpeed(motorVelocities);
}

void motorControl_moveRobotWorldVelocities(RobotPose robot, Vector3d v_world)
{
	Vector3d v_body = motorControl_rotate(v_world, -robot.theta);
	motorControl_moveRobotBodyVelocities(v_body);
}

Vector3d motorControl_rotate(Vector3d v, double theta)
{
	Matrix3d R(3, 3);
	R <<  cos(theta), sin(theta), 0,
	  	 -sin(theta), cos(theta), 0,
	  	  0,          0,          1;
	Vector3d vNew = R * v;
	return vNew;
}
