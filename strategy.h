#include "play.h"
#include "stdio.h"
#include "time.h"

#define KEY_LEFT  1113937
#define KEY_UP    1113938
#define KEY_RIGHT 1113939
#define KEY_DOWN  1113940
#define KEY_ESC   1048603
#define KEY_SPACE 1048608
#define KEY_DEL 1114111
#define KEY_a 1048673
#define KEY_A 1179713
#define KEY_1 1048625

#define KEY_ENTER 1048586
#define MODIFIER_SHIFT (1 << 16)
#define MODIFIER_CTRL (1 << 18)
#define MODIFIER_ALT (1 << 19)
#define MOVE_TIMER 2
#define END_UPDATE_CAMERA_WAIT_TIMER 3
#define LATENCY_TIMER 4
#define CAMERA_WAIT_DIST 10 //pixels
#define MOVEMENT_TIME 1000 //ms


enum operatingStates
	{idle, move, calibrate} operatingState;

enum strategy_states
	{testGoToPoint, testPlayRushGoal, testPlayDefense, testStrategySwitch} strategy_state;

enum calibrate_states
	{cameraLatency, pidControl, cameraDistance} calibrate_state;

bool stopRobot = true;

coord2 ball;
double currentTime;

coord3 lastCameraUpdateCoords;
float moveTime = 0;

void enterStrategyState(strategy_states newState)
{
	printf("strategy_state = newState (%d)\n", newState);
	strategy_state = newState;
	operatingState = move;
}

void receiveCoords(coord3 pRobot1, coord3 pRobot2, coord2 pBall, double t)
{
	manualTimerTicks++;

	robot1cameraPosition = pRobot1;
	robot2cameraPosition = pRobot2;
	ball = pBall;
	currentTime = t;
}

long motor0sum;
long motor1sum;
long motor2sum;
float ratioSum;

#define CALIBRATE_TIMER 8
#define CALIBRATE_POWER 60
#define CALIBRATE_VELOCITY 20 //cm per sec
#define CALIBRATE_DRIVE_TIME 2000 //ms
#define CALIBRATE_WAIT_TO_MEASURE_TIME 1500 //ms

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
			spinWheels(pwr, pwr, 0);
		else if(triDirection == 1)
			spinWheels(0, -pwr, pwr);
		else if(triDirection == 2)
			spinWheels(-pwr, 0, -pwr);
	}
	else if(calibrate_state == cameraDistance)
	{
		float vel = sgn * CALIBRATE_VELOCITY;
		if(triDirection == 0)
			moveRobotBodyCoordinates((coord3){0, vel, 0});
		else if(triDirection == 1)
			moveRobotBodyCoordinates((coord3){vel / 2, -vel / 2 * sqrtf(3), 0});
		else if(triDirection == 2)
			moveRobotBodyCoordinates((coord3){-vel / 2, -vel / 2 * sqrtf(3), 0});
	}
}

void measure1(int a)
{
	if(a == 0)
		motor0sum += labs(readQuadratureEncoderRegister(0));
	else if(a == 1)
		motor1sum += labs(readQuadratureEncoderRegister(1));
	else if(a == 2)
		motor2sum += labs(readQuadratureEncoderRegister(2));
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

void startCalibrate(calibrate_states state)
{
	printf("start calibrate\n");
	operatingState = calibrate;
	calibrate_state = state;
	calibrateState = 0;
	calibrateForwardBackward = 0;
	calibrateTriDirection = 0;
	motor0sum = 0;
	motor1sum = 0;
	motor2sum = 0;
	ratioSum = 0;
}

void continueCalibrate()
{
	if(calibrateState == 0 && calibrateTriDirection < 3)
	{
		startTimer(CALIBRATE_TIMER);
		if(calibrate_state == pidControl)
			resetAllQuadratureEncoderCounters();
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
		killMotors();
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
				long qpps0 = motor0sum * 1000 / CALIBRATE_DRIVE_TIME * 128 / CALIBRATE_POWER / 4;
				long qpps1 = motor1sum * 1000 / CALIBRATE_DRIVE_TIME * 128 / CALIBRATE_POWER / 4;
				long qpps2 = motor2sum * 1000 / CALIBRATE_DRIVE_TIME * 128 / CALIBRATE_POWER / 4;
				printf("results: %ld %ld %ld\n", qpps0, qpps1, qpps2);
				setMotorPidConstants(0, (pidq){250000, 130000, 400000, qpps0});
				setMotorPidConstants(1, (pidq){250000, 130000, 400000, qpps1});
				setMotorPidConstants(2, (pidq){250000, 130000, 400000, qpps2});
			}
			else if(calibrate_state == cameraDistance)
			{
				pulsesPerRadian = pulsesPerRadian * ratioSum / 6;
				printf("Ratio: %.2f, New pulsesPerRadian %.2f\n", ratioSum / 6, pulsesPerRadian);
			}
		}
	}
}

int latencyState;

void startMeasureLatency()
{
	operatingState = calibrate;
	calibrate_state = cameraLatency;
	latencyState = 0;
}


void measureLatency()
{
	if(latencyState == 0)
	{
		startPosition = robot1cameraPosition;
		moveRobotWorldCoordinates(robot1cameraPosition, (coord3) { 30, 0, 0 });
		latencyState = 1;
		startTimer(LATENCY_TIMER);
	}
	else if(latencyState == 1 && utility_dist3(robot1cameraPosition, startPosition) > 1)
	{
		killMotors();
		cameraLatency_ms = getTimerTime_ms(LATENCY_TIMER);
		printf("Latency of camera: %f ms\n", cameraLatency_ms);
		latencyState = 2;
	}
}

void strategy_tick()
{
	float ballAngle;
	coord2 ccc;
	coord2 vector;

	// Current state actions
	switch (operatingState)
	{
	case idle:
		//do nothing
		break;
	case move:
		switch (strategy_state)
		{
		case testGoToPoint:
			//skill_goToPoint(robot1currentPosition, ball);
			ballAngle = utility_getAngle(utility_3to2(robot1currentPosition), ball);
			skill_goToPoint(robot1currentPosition, (coord3){ball.x, ball.y, ballAngle});
			break;
		case testStrategySwitch:
			if(utility_dist(ball, utility_3to2(robot1currentPosition)) < utility_dist(ball, utility_3to2(robot2cameraPosition)))
				play_rushGoal(robot1currentPosition, ball);
			else
				play_blockGoal(robot1currentPosition, ball);
			break;
		case testPlayRushGoal:
			play_rushGoal(robot1currentPosition, ball);
			break;
		case testPlayDefense:
			play_blockGoal(robot1currentPosition, ball);
			break;
		}
		break;
	case calibrate:
		switch (calibrate_state)
		{
		case cameraLatency:
			measureLatency();
			break;
		case pidControl:
		case cameraDistance:
			continueCalibrate();
			break;
		}
		break;

//		continueCalibrate();
		break;
	}
//	printf("coords: [%f %f %f], [%f %f]\n", robot1.x, robot1.y, robot1.w, ball.x,
//			ball.y);
}

void pressKey(int key)
{
	//printf("key press: %d\n", key);
	//if(key == KEY_a + ('r' - 'a'))
	switch (key)
	{
	case KEY_ESC:
		operatingState = idle;
		printf("operatingState = idle\n");
		killMotors();
		break;
	case KEY_ENTER:
		operatingState = move;
		break;
	case KEY_SPACE:
		if(operatingState != idle)
		{
			killMotors();
			operatingState = idle;
			printf("operatingState = idle\n");
		}
		else
		{
			operatingState = move;
			printf("operatingState = move\n");
		}
		//killMotors();
		break;
	case KEY_a:
		spinWheel(0, 127);
		spinWheel(1, 127);
		spinWheel(2, 127);
		break;
	case KEY_a + ('b' - 'a'):
		break;
	case KEY_a + ('c' - 'a'):
		startCalibrate(pidControl);
		break;
	case KEY_a + ('d' - 'a'):
		enterStrategyState(testPlayDefense);
		break;
	case KEY_a + ('e' - 'a'):
		updateCurrentPosition(true);
		printf("Position now: %f %f %f\n", robot1currentPosition.x,
				robot1currentPosition.y, robot1currentPosition.w);
		//
		//		showSpeedVsVelocityGraph(0);
		//		showSpeedVsVelocityGraph(1);
		//		showSpeedVsVelocityGraph(2);
		//driveMotorWithSignedSpeed(0, 100);
		break;
	case KEY_a + ('f' - 'a'):
		driveMotorWithSignedSpeed(0, 0);
		break;
	case KEY_a + ('g' - 'a'):
		enterStrategyState(testPlayRushGoal);
		break;
	case KEY_a + ('h' - 'a'):
		robot1currentPosition = robot1cameraPosition;
		break;
	case KEY_a + ('i' - 'a'):
		updateCurrentPosition(false);
		break;
	case KEY_a + ('j' - 'a'):
		printMotorSpeeds();
		break;
	case KEY_a + ('k' - 'a'):
		printPidConstants();
		break;
	case KEY_a + ('l' - 'a'):
		setMotorPidConstants(0,
				(pidq ) { 250000, 130000, 400000, motor0sum / 4 });
		setMotorPidConstants(1,
				(pidq ) { 250000, 130000, 400000, motor1sum / 4 });
		setMotorPidConstants(2,
				(pidq ) { 250000, 130000, 400000, motor2sum / 4 });
		break;
	case KEY_a + ('m' - 'a'):
		printMotorDiffs();
		break;
	case KEY_a + ('n' - 'a'):
		break;
	case KEY_a + ('o' - 'a'):
		break;
	case KEY_a + ('p' - 'a'):
		enterStrategyState(testGoToPoint);
		break;
	case KEY_a + ('q' - 'a'):
		break;
	case KEY_a + ('r' - 'a'):
		emptyBuffer();
		break;
	case KEY_a + ('s' - 'a'):
		startTimer(7);
		printf("Start timer. clock: %f\n", getTime_ms());
		break;
	case KEY_a + ('t' - 'a'):
		printf("Reset encoders\n");
		resetQuadratureEncoderCounters(0);
		break;
	case KEY_a + ('u' - 'a'):
		printf("Encoder: %ld\n", readQuadratureEncoderRegister(0));
		break;
	case KEY_a + ('v' - 'a'):
		break;
	case KEY_a + ('w' - 'a'):
		break;
	case KEY_a + ('x' - 'a'):
		startCalibrate(cameraDistance);
		//startMeasureLatency();
		break;
	case KEY_a + ('y' - 'a'):
		break;
	case KEY_a + ('z' - 'a'):
		startMeasureLatency();
		break;
	}
	if ((key & (~MODIFIER_CTRL)) >= KEY_LEFT
			&& (key & (~MODIFIER_CTRL)) <= KEY_DOWN)
	{
		setOverride(false);
		switch (key)
		{
		case KEY_LEFT:
			moveRobotWorldCoordinates(robot1currentPosition, (coord3) { -50, 0, 0 });
			break;
		case KEY_UP:
			moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 0, 50, 0 });
			break;
		case KEY_RIGHT:
			moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 50, 0, 0 });
			break;
		case KEY_DOWN:
			moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 0, -50, 0 });
			break;
		case KEY_LEFT | MODIFIER_CTRL:
			moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 0, 0, 2 });
			break;
		case KEY_RIGHT | MODIFIER_CTRL:
			moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 0, 0, -2 });
			break;
		}
		overrideForSpecifiedTime(750);
	}
//	printf("key press: %d\n", key);
}
