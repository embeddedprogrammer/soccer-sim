#include "stdio.h"
#include "time.h"
#include "play.h"
#include "calibrate.h"
#include "control.h"
#include "motorControl.h"

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

void calibrate_driveTriDirection(int triDirection, int sgn)
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

void calibrate_measure1(int a)
{
	if(a == 0)
		calibrate_motor0sum += labs(motorControl_readQuadratureEncoderRegister(0));
	else if(a == 1)
		calibrate_motor1sum += labs(motorControl_readQuadratureEncoderRegister(1));
	else if(a == 2)
		calibrate_motor2sum += labs(motorControl_readQuadratureEncoderRegister(2));
}

void calibrate_measure2(int a, int b)
{
	calibrate_measure1(a); calibrate_measure1(b);
}

void calibrate_measureTriDirection(int triDirection)
{
	if(calibrate_state == pidControl)
	{
		if(triDirection == 0)
			calibrate_measure2(0, 1);
		else if(triDirection == 1)
			calibrate_measure2(1, 2);
		else if(triDirection == 2)
			calibrate_measure2(0, 2);
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

void calibrate_continueCalibrate()
{
	if(calibrateState == 0 && calibrateTriDirection < 3)
	{
		utility_startTimer(CALIBRATE_TIMER);
		if(calibrate_state == pidControl)
			motorControl_resetAllQuadratureEncoderCounters();
		robot1currentPosition = robot1cameraPosition;
		startPosition = robot1cameraPosition;
		if(calibrateForwardBackward == 0)
			calibrate_driveTriDirection(calibrateTriDirection, 1);
		else
			calibrate_driveTriDirection(calibrateTriDirection, -1);
		calibrateState++;
	}
	else if(calibrateState == 1 && utility_getTimerTime_ms(CALIBRATE_TIMER) > CALIBRATE_DRIVE_TIME)
	{
		utility_startTimer(CALIBRATE_TIMER);
		motorControl_killMotors();
		calibrateState++;
	}
	else if(calibrateState == 2 && utility_getTimerTime_ms(CALIBRATE_TIMER) > CALIBRATE_WAIT_TO_MEASURE_TIME)
	{
		calibrate_measureTriDirection(calibrateTriDirection);
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
				//setMotorPidConstants(0, (pidq){250000, 130000, 400000, qpps0});
				//setMotorPidConstants(1, (pidq){250000, 130000, 400000, qpps1});
				//setMotorPidConstants(2, (pidq){250000, 130000, 400000, qpps2});
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

void calibrate_startMeasureLatency()
{
	calibrate_state = cameraLatency;
	latencyState = 0;
}

void calibrate_measureLatency()
{
	if(latencyState == 0)
	{
		startPosition = robot1cameraPosition;
		motorControl_moveRobotBodyCoordinates((coord3){0, 30, 0});
		latencyState = 1;
		utility_startTimer(LATENCY_TIMER);
	}
	else if(latencyState == 1 && utility_dist3(robot1cameraPosition, startPosition) > .5)
	{
		motorControl_killMotors();
		cameraLatency_ms = utility_getTimerTime_ms(LATENCY_TIMER) - 170;
		printf("Latency of camera: %f ms\n", cameraLatency_ms);
		latencyState = 2;
	}
//	printf("Dist: %f\n", utility_dist3(robot1cameraPosition, startPosition));
}

int mMatrixDir;

void continueMCalibrate()
{
	if(calibrateState == 0 && calibrateTriDirection < 3)
	{
		utility_startTimer(CALIBRATE_TIMER);
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
	else if(calibrateState == 1 && utility_getTimerTime_ms(CALIBRATE_TIMER) > 3000)
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

void calibrate_startCalibrate(calibrate_states state)
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


float calibrateSpeed = 0;

#define CALIBRATE_DRIVE_TIME2 1000

void calibrate_startSpeedCalibrate()
{
	calibrate_state = state_calibrateSpeed;
	utility_startTimer(CALIBRATE_TIMER);
	motorControl_statePredictionMode = manuallyUpdate;
	startPosition = robot1currentPosition;
	calibrateSpeed = 0;
}

void continueSpeedCalibrate()
{
	if(utility_getTimerTime_ms(CALIBRATE_TIMER) > CALIBRATE_DRIVE_TIME2)
	{
		float theActualSpeed = utility_dist3(startPosition, robot1currentPosition) / CALIBRATE_DRIVE_TIME2 * 1000;
		printf("Desired speed %f Actual speed: %f\n", calibrateSpeed, theActualSpeed);

		calibrateSpeed += 5;
		if(calibrateSpeed <= 25)
		{
			motorControl_moveRobotBodyCoordinates((coord3){0, calibrateSpeed, 0});
			startPosition = robot1currentPosition;
			utility_startTimer(CALIBRATE_TIMER);
		}
		else
		{
			motorControl_killMotors();
			calibrate_state = notCalibrating;
		}
	}
	else
	{
//		motorControl_moveRobotBodyCoordinates((coord3){0, calibrateSpeed, 0});
	}
}

#define CALIBRATE_XY_SCALE_DRIVE_TIME 1000
#define CALIBRATE_XY_SCALE_DRIVE_SPEED 20
#define CALIBRATE_XY_SCALE_WAIT_TO_MEASURE 1500
int calibrateXYScaleState;

void calibrate_startXYScaleCalibrate()
{
	printf("start xy\n");
	calibrate_state = state_XYScaleCalibrate;
	utility_startTimer(CALIBRATE_TIMER);
	calibrateXYScaleState = 0;
	motorControl_statePredictionMode = manuallyUpdate;
}

void continueXYScaleCalibrate()
{
	if(calibrateXYScaleState == 0)
	{
		printf("continue xy\n");
		motorControl_moveRobotBodyCoordinates((coord3){0, CALIBRATE_XY_SCALE_DRIVE_SPEED, 0});
		robot1currentPosition = robot1cameraPositionAverage;
		startPosition = robot1cameraPositionAverage;
		utility_startTimer(CALIBRATE_TIMER);
		calibrateXYScaleState++;
	}
	else if(calibrateXYScaleState == 1 && utility_getTimerTime_ms(CALIBRATE_TIMER) > CALIBRATE_XY_SCALE_DRIVE_TIME)
	{
		motorControl_killMotors();
		utility_startTimer(CALIBRATE_TIMER);
		calibrateXYScaleState++;
	}
	else if(calibrateXYScaleState == 2 && utility_getTimerTime_ms(CALIBRATE_TIMER) > CALIBRATE_XY_SCALE_WAIT_TO_MEASURE)
	{
		float encoderDistance = utility_dist3(startPosition, robot1currentPosition);
		float actualDistance = utility_dist3(startPosition, robot1cameraPositionAverage);
		float scaleFactor = encoderDistance / actualDistance;
		printf("Encoder distance %f Actual distance: %f Scale factor %f\n", encoderDistance, actualDistance, scaleFactor);
		motorControl_scaleMMatrixXY(scaleFactor);
		calibrateXYScaleState++;
		calibrate_state = notCalibrating;
	}
}

#define CALIBRATE_W_SCALE_DRIVE_TIME 2000
#define CALIBRATE_W_SCALE_DRIVE_SPEED 1
#define CALIBRATE_W_SCALE_WAIT_TO_MEASURE 1500
int calibrateWScaleState;

void calibrate_startWScaleCalibrate()
{
	calibrate_state = state_WScaleCalibrate;
	utility_startTimer(CALIBRATE_TIMER);
	calibrateWScaleState = 0;
	motorControl_statePredictionMode = manuallyUpdate;
}

void continueWScaleCalibrate()
{
	if(calibrateWScaleState == 0)
	{
		motorControl_moveRobotBodyCoordinates((coord3){0, 0, CALIBRATE_W_SCALE_DRIVE_SPEED});
		robot1currentPosition = robot1cameraPositionAverage;
		startPosition = robot1cameraPositionAverage;
		utility_startTimer(CALIBRATE_TIMER);
		calibrateWScaleState++;
	}
	else if(calibrateWScaleState == 1 && utility_getTimerTime_ms(CALIBRATE_TIMER) > CALIBRATE_W_SCALE_DRIVE_TIME)
	{
		motorControl_killMotors();
		utility_startTimer(CALIBRATE_TIMER);
		calibrateWScaleState++;
	}
	else if(calibrateWScaleState == 2 && utility_getTimerTime_ms(CALIBRATE_TIMER) > CALIBRATE_W_SCALE_WAIT_TO_MEASURE)
	{
		float encoderSpin = utility_angleMod(robot1currentPosition.w - startPosition.w);
		float actualSpin = utility_angleMod(robot1cameraPositionAverage.w - startPosition.w);
		float scaleFactor = encoderSpin / actualSpin;
		printf("Desired spin %f Actual spin: %f Scale factor %f\n", encoderSpin, actualSpin, scaleFactor);
		motorControl_scaleMMatrixW(scaleFactor);
		calibrateWScaleState++;
		calibrate_state = notCalibrating;
	}
}


#define ROBO_CLAW_JITTER_ACCEL 1
float roboclawErrConstantSpeed = -5;
float roboclawErrJitterSpeed;
int roboclawErrDir;
int roboclawErrState;
int roboclawErrWaitCycles;
#define ROBO_CLAW_WAIT_CYCLE_TICKS 1

void calibrate_startRoboclawErrCalibrate()
{
	roboclawErrState = 0;
	roboclawErrDir = 1;
	roboclawErrJitterSpeed = 0;
	calibrate_state = state_RoboclawErr;
//	motorControl_resetRoboErr();
	roboclawErrConstantSpeed += 5;
//	printf("Calibrate roboclaw error with constant speed %f plus jitter and %d wait cycles\n", roboclawErrConstantSpeed, ROBO_CLAW_WAIT_CYCLE_TICKS);
	printf("Calibrate roboclaw error with random speeds\n");

	P_MAX_VEL_ACC = 0;
	P_MAX_SPIN_ACC = 0;
}

float getRand(float maxNum)
{
	return ((float)rand()) * maxNum / RAND_MAX;
}

void calibrate_continueRoboclawErrCalibrate()
{
	if(roboclawErrState == 0)
	{
		roboclawErrWaitCycles++;
		// if(motorControl_roboErr())
		// {
		// 	printf("Error in cmc. Jittering stopped at %f speed\n", roboclawErrJitterSpeed);
		// 	roboclawErrState = 1;
		// 	motorControl_killMotors();
		// }
		if(roboclawErrWaitCycles >= ROBO_CLAW_WAIT_CYCLE_TICKS)
		{
			roboclawErrWaitCycles = 0;
			if(DEBUG_PRINT)
				printf("Jitter speed: %f\n", roboclawErrJitterSpeed);
			roboclawErrDir *= -1;
			roboclawErrJitterSpeed += ((float)ROBO_CLAW_JITTER_ACCEL) / TICKS_PER_SECOND * ROBO_CLAW_WAIT_CYCLE_TICKS;
			P_MAX_VEL_ACC = roboclawErrJitterSpeed;
			P_MAX_SPIN_ACC = 0;

			motorControl_moveRobotWorldCoordinates(robot1currentPosition, (coord3){getRand(20) * roboclawErrDir, getRand(20) * roboclawErrDir, getRand(5) * roboclawErrDir});
//			motorControl_moveRobotBodyCoordinates((coord3){0, roboclawErrConstantSpeed + roboclawErrJitterSpeed * roboclawErrDir, 0});
		}

	}
//	else if(roboclawErrState == 1)
//	{
//		utility_startTimer(CALIBRATE_TIMER);
//		roboclawErrState = 2;
//
//		utility_getTimerTime_ms(CALIBRATE_TIMER)
//
//	}
//	else if(roboclawErrState == 2)
//	{
//		if(utility_getTimerTime_ms(CALIBRATE_TIMER) > 200)
//		{
//			roboclawErrState = 3;
//
//		}
//
//	}
}

void calibrate_tick()
{
	switch (calibrate_state)
	{
	case notCalibrating:
		break;
	case cameraLatency:
		calibrate_measureLatency();
		break;
	case pidControl:
	case cameraDistance:
		calibrate_continueCalibrate();
		break;
	case calibrateMMatrix:
		continueMCalibrate();
		break;
	case state_calibrateSpeed:
		continueSpeedCalibrate();
		break;
	case state_XYScaleCalibrate:
		continueXYScaleCalibrate();
		break;
	case state_WScaleCalibrate:
		continueWScaleCalibrate();
		break;
	case state_RoboclawErr:
		calibrate_continueRoboclawErrCalibrate();
		break;
	}
}
