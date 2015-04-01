#include "stdio.h"
#include "time.h"
#include "play.h"
#include "calibrate.h"

long calibrate_motor0sum;
long calibrate_motor1sum;
long calibrate_motor2sum;
float ratioSum;

#define CALIBRATE_TIMER 8
#define CALIBRATE_POWER 60
#define CALIBRATE_VELOCITY 20 //cm per sec
#define CALIBRATE_DRIVE_TIME 2000 //ms
#define CALIBRATE_WAIT_TO_MEASURE_TIME 1500 //ms
#define LATENCY_TIMER 4

static int calibrateState;
static int calibrateForwardBackward;
static int calibrateTriDirection;

coord3 startPosition;

void driveTriDirection(int triDirection, int sgn)
{
	if(calibrate_state == pidControl)
	{
		float pwr = sgn * CALIBRATE_POWER;
		if(triDirection == 0)
			motorControl_spinWheels(pwr, pwr, 0);
		else if(triDirection == 1)
			motorControl_spinWheels(0, -pwr, pwr);
		else if(triDirection == 2)
			motorControl_spinWheels(-pwr, 0, -pwr);
	}
	else if(calibrate_state == cameraDistance)
	{
		float vel = sgn * CALIBRATE_VELOCITY;
		if(triDirection == 0)
			motorControl_moveRobotBodyCoordinates((coord3){0, vel, 0});
		else if(triDirection == 1)
			motorControl_moveRobotBodyCoordinates((coord3){vel / 2, -vel / 2 * sqrtf(3), 0});
		else if(triDirection == 2)
			motorControl_moveRobotBodyCoordinates((coord3){-vel / 2, -vel / 2 * sqrtf(3), 0});
	}
}

void measure1(int a)
{
	if(a == 0)
		calibrate_motor0sum += labs(motorControl_readQuadratureEncoderRegister(0));
	else if(a == 1)
		calibrate_motor1sum += labs(motorControl_readQuadratureEncoderRegister(1));
	else if(a == 2)
		calibrate_motor2sum += labs(motorControl_readQuadratureEncoderRegister(2));
}

void measure2(int a, int b)
{
	measure1(a); measure1(b);
}

void measureTriDirection(int triDirection)
{
	if(calibrate_state == pidControl)
	{
		if(triDirection == 0)
			measure2(0, 1);
		else if(triDirection == 1)
			measure2(1, 2);
		else if(triDirection == 2)
			measure2(0, 2);
	}
	else if(calibrate_state == cameraDistance)
	{
		float desiredDistance = ((float)(CALIBRATE_VELOCITY * CALIBRATE_DRIVE_TIME)) / 1000;
		float cameraDistance = utility_dist3(startPosition, robot1cameraPosition);
		float encoderDistance = utility_dist3(startPosition, robot1currentPosition);
		float ratio = encoderDistance / cameraDistance;
		ratioSum += ratio;
		printf("Desired: %.2f, camera: %.2f, encoder: %.2f\n", desiredDistance, cameraDistance, encoderDistance);
		printf("Ratio: %.2f\n", ratio);
	}
}

void continueCalibrate()
{
	if(calibrateState == 0 && calibrateTriDirection < 3)
	{
		startTimer(CALIBRATE_TIMER);
		if(calibrate_state == pidControl)
			motorControl_resetAllQuadratureEncoderCounters();
		robot1currentPosition = robot1cameraPosition;
		startPosition = robot1cameraPosition;
		if(calibrateForwardBackward == 0)
			driveTriDirection(calibrateTriDirection, 1);
		else
			driveTriDirection(calibrateTriDirection, -1);
		calibrateState++;
	}
	else if(calibrateState == 1 && getTimerTime_ms(CALIBRATE_TIMER) > CALIBRATE_DRIVE_TIME)
	{
		startTimer(CALIBRATE_TIMER);
		motorControl_killMotors();
		calibrateState++;
	}
	else if(calibrateState == 2 && getTimerTime_ms(CALIBRATE_TIMER) > CALIBRATE_WAIT_TO_MEASURE_TIME)
	{
		measureTriDirection(calibrateTriDirection);
		calibrateState = 0;
		calibrateForwardBackward++;
		if(calibrateForwardBackward == 2)
		{
			calibrateForwardBackward = 0;
			calibrateTriDirection++;
		}
		if(calibrateTriDirection == 3)
		{
			if(calibrate_state == pidControl)
			{
				//TODO: divide by CALIBRATE_DRIVE_TIME
				long qpps0 = calibrate_motor0sum * 1000 / CALIBRATE_DRIVE_TIME * 128 / CALIBRATE_POWER / 4;
				long qpps1 = calibrate_motor1sum * 1000 / CALIBRATE_DRIVE_TIME * 128 / CALIBRATE_POWER / 4;
				long qpps2 = calibrate_motor2sum * 1000 / CALIBRATE_DRIVE_TIME * 128 / CALIBRATE_POWER / 4;
				printf("results: %ld %ld %ld\n", qpps0, qpps1, qpps2);
				setMotorPidConstants(0, (pidq){250000, 130000, 400000, qpps0});
				setMotorPidConstants(1, (pidq){250000, 130000, 400000, qpps1});
				setMotorPidConstants(2, (pidq){250000, 130000, 400000, qpps2});
			}
			else if(calibrate_state == cameraDistance)
			{
				pulsesPerRadian = pulsesPerRadian * ratioSum / 6;
				printf("Ratio: %.2f, New pulsesPerRadian %.2f\n", ratioSum / 6, pulsesPerRadian);
				motorControl_statePredictionMode = constantlyUpdate;
			}
		}
	}
}

int latencyState;

void startMeasureLatency()
{
	calibrate_state = cameraLatency;
	latencyState = 0;
}

void measureLatency()
{
	if(latencyState == 0)
	{
		startPosition = robot1cameraPosition;
		motorControl_moveRobotWorldCoordinates(robot1cameraPosition, (coord3) { 30, 0, 0 });
		latencyState = 1;
		startTimer(LATENCY_TIMER);
	}
	else if(latencyState == 1 && utility_dist3(robot1cameraPosition, startPosition) > 1)
	{
		motorControl_killMotors();
		cameraLatency_ms = getTimerTime_ms(LATENCY_TIMER);
		printf("Latency of camera: %f ms\n", cameraLatency_ms);
		latencyState = 2;
	}
}

int mMatrixDir;

void continueMCalibrate()
{
	if(calibrateState == 0 && calibrateTriDirection < 3)
	{
		startTimer(CALIBRATE_TIMER);
		if(mMatrixDir == 0)
			motorControl_driveMotorsAtSpeed(20, 0, 0);
		if(mMatrixDir == 1)
			motorControl_driveMotorsAtSpeed(-20, 0, 0);
		if(mMatrixDir == 2)
			motorControl_driveMotorsAtSpeed(0, 20, 0);
		if(mMatrixDir == 3)
			motorControl_driveMotorsAtSpeed(0, -20, 0);
		if(mMatrixDir == 4)
			motorControl_driveMotorsAtSpeed(0, 0, 20);
		if(mMatrixDir == 5)
			motorControl_driveMotorsAtSpeed(0, 0, -20);
		calibrateState++;
		motorControl_setUpdateMode(printMotorDiffs);
	}
	else if(calibrateState == 1 && getTimerTime_ms(CALIBRATE_TIMER) > 3000)
	{
		motorControl_setUpdateMode(constantlyUpdate);
		motorControl_killMotors();
		calibrateState++;
	}
}

void calibrate_MMatrix(int dir)
{
	mMatrixDir = dir;
	calibrate_state = calibrateMMatrix;
	calibrateState = 0;
}

void calibrate_stop()
{
	calibrate_state = notCalibrating;
}

void startCalibrate(calibrate_states state)
{
	calibrate_state = state;
	calibrateState = 0;
	calibrateForwardBackward = 0;
	calibrateTriDirection = 0;
	calibrate_motor0sum = 0;
	calibrate_motor1sum = 0;
	calibrate_motor2sum = 0;
	ratioSum = 0;
	if(calibrate_state == cameraDistance)
		motorControl_statePredictionMode = manuallyUpdate;
}

void calibrate_tick()
{
	switch (calibrate_state)
	{
	case cameraLatency:
		measureLatency();
		break;
	case pidControl:
	case cameraDistance:
		continueCalibrate();
		break;
	case calibrateMMatrix:
		continueMCalibrate();
		break;
	}
}
