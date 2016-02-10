/*
 * motorControl.h
 *
 *  Created on: Mar 28, 2015
 *      Author: newuser
 */

#include "utility.h"
#include "stdbool.h"

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

typedef struct {
	long p, i, d, q;
} pidq;

double cameraLatency_ms = 200;
//float pulsesPerRadian = (19456 / 2 / M_PI);
float pulsesPerRadian = (198100 / 10 / 2 / M_PI); //Accurate to .5%

// Everything is in centimeters.
#define P_WHEEL_RADIUS 3 //Measured 2.7
#define P_ROBOT_RADIUS 8.4 //Measured
#define P_KICKER_DIST 6.35 //Measured
const float WHEEL_ANGLES[] = {120, 240, 0};
float M[3][3];
float M_inverse[3][3];
float wheelPowerNeededPerVelocityUnit = 20;
int serial_fd;

#define MOTOR_CONTROL_OVERRIDE_TIMER 7
#define TICKS_PER_SECOND 20
bool override = false;
double timeToOverride;
coord3 currentBodyVelocity;

coord3 robot1currentPosition;
coord3 robot1cameraPosition, robot2cameraPosition, robot1cameraPositionAverage;
coord2 ball;
bool DEBUG_PRINT;

#ifndef M_PI
#define M_PI 3.1415926535
#endif

//bool motorControl_roboErr();

//void motorControl_resetRoboErr();

enum motorControl_statePredictionModes
	{constantlyUpdate, manuallyUpdate, printMotorDiffs} motorControl_statePredictionMode;

//int motorControl_calcChecksum(char command[], int size);

//int motorControl_calcChecksum2(char command[], int commandSize, char result[], int resultSize);

//void motorControl_serial_sendMessage(char command[], int size);

//int motorControl_serial_readMessage(char result[], int size);

//long motorControl_arrayToLong(char array[], int startPos);

//void motorControl_longToArray(char array[], int startPos, long value);

void motorControl_emptyBuffer();

void motorControl_setQuadratureEncoderRegister(int wheelId, long val);

long motorControl_readQuadratureEncoderRegister(int wheelId);

coord3 motorControl_readQuadraureEncoders();

void motorControl_printQuadratureEncoderRegisters();

//void motorControl_resetQuadratureEncoderCounters(int roboclaw);

void motorControl_resetAllQuadratureEncoderCounters();

//long motorControl_readMotorSpeed(int wheelId);

//pidq motorControl_readMotorPidConstants(int wheelId);

//void motorControl_setMotorPidConstants(int wheelId, pidq val);

void motorControl_spinWheel(int wheelId, int power);

void motorControl_spinWheels(int power0, int power1, int power2);

void motorControl_killMotors();

void motorControl_driveMotorWithSignedSpeed(int wheelId, long qSpeed);

//void motorControl_printMotorSpeeds();

//void motorControl_printPidConstants();

//void motorControl_showSpeedVsVelocityGraph(int wheelId);

void motorControl_startCalibrate(coord2 velocity);

float motorControl_gme(float M[3][3], int i, int j, int r, int c);

float motorControl_getSignedMinorDet(float M[3][3], int i, int j);

float motorControl_getDet(float M[3][3]);

void motorControl_invertMatrix(float M[3][3], float dest[3][3]);

float motorControl_getBodyFrameVectorSpinFactor(coord2 wheelBase, coord2 wheelDir);

void motorControl_calcBodyFrameVectors();

void motorControl_init(ros::NodeHandle nh);

void motorControl_driveMotorsAtSpeed(float motor1Speed, float motor2Speed, float motor3Speed);

coord3 matrix_mult(float M[3][3], coord3 v);

coord3 motorControl_translateBodyCoordinatesToMotorVelocities(coord3 v);

coord3 motorControl_translateMotorVelocitiesToBodyCoordinates(coord3 motorSpeeds);

coord3 motorControl_translateWorldCoordinatesToBodyCoordinates(coord3 robot, coord3 v);

coord3 motorControl_translateBodyCoordinatesToWorldCoordinates(coord3 robot, coord3 v);

void motorControl_moveRobotBodyCoordinates(coord3 v);

void motorControl_moveRobotWorldCoordinates(coord3 robot, coord3 v);

void motorControl_setOverride(bool val);

void motorControl_overrideForSpecifiedTime(double t);

void motorControl_addBodyVelocityToCurrentPosition(coord3 bodyVelocity);

void motorControl_updateCurrentPosition(bool print);

void motorControl_tick();

void motorControl_printMotorDiffs();

void motorControl_setUpdateMode(motorControl_statePredictionModes mode);

void motorControl_scaleMMatrixW(float scaleFactor);

void motorControl_scaleMMatrixXY(float scaleFactor);

void motorControl_printCoord3(const char* str, coord3 v);

void motorControl_printCoord2(const char* str, coord2 v);

#endif /* MOTORCONTROL_H_ */
