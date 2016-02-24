/*
 * motorControl.h
 *
 *  Created on: Mar 28, 2016
 *      Author: Jacob White
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <eigen3/Eigen/Dense>
#include "ros/ros.h"

using namespace std;
using namespace Eigen;

struct RobotPose
{
	Vector2d pos;
	double theta;
};

void motorControl_driveMotorWithSignedSpeed(int wheelId, long qSpeed);

void motorControl_killMotors();

void motorControl_calcBodyFrameVectors();

void motorControl_init(ros::NodeHandle nh, string robotName);

void motorControl_driveMotorsAtSpeed(Vector3d motorVelocities);

void motorControl_moveRobotBodyVelocities(Vector3d v_body);

void motorControl_moveRobotWorldVelocities(RobotPose robot, Vector3d v_world);

Vector3d motorControl_rotate(Vector3d v, double theta);

#endif /* MOTORCONTROL_H_ */
