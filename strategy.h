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
	{testGoToPoint, testPlayRushGoal, testPlayDefense} strategy_state;

enum calibrate_states
	{cameraLatency, pidControl} calibrate_state;

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

void driveTriDirection(int triDirection, int pwr)
{
	if(triDirection == 0)
		spinWheels(pwr, pwr, 0);
	else if(triDirection == 1)
		spinWheels(0, -pwr, pwr);
	else if(triDirection == 2)
		spinWheels(-pwr, 0, -pwr);
}

long motor0sum;
long motor1sum;
long motor2sum;

void measure1(int a)
{
	if(a == 0)
		motor0sum += labs(readQuadratureEncoderRegister(0));
	else if(a == 1)
		motor1sum += labs(readQuadratureEncoderRegister(0));
	else if(a == 2)
		motor2sum += labs(readQuadratureEncoderRegister(0));
}

void measure2(int a, int b)
{
	measure1(a); measure1(b);
}

void measureTriDirection(int triDirection)
{
	if(triDirection == 0)
		measure2(0, 1);
	else if(triDirection == 1)
		measure2(1, 2);
	else if(triDirection == 2)
		measure2(0, 2);
}

#define CALIBRATE_TIMER 8
#define CALIBRATE_POWER 64
#define CALIBRATE_DRIVE_TIME 1000 //ms
#define CALIBRATE_WAIT_TO_MEASURE_TIME 500 //ms

static int calibrateState;
static int calibrateForwardBackward;
static int calibrateTriDirection;

void startPidCalibrate()
{
	printf("start calibrate\n");
	operatingState = calibrate;
	calibrate_state = pidControl;
	calibrateState = 0;
	calibrateForwardBackward = 0;
	calibrateTriDirection = 0;
	motor0sum = 0;
	motor1sum = 0;
	motor2sum = 0;
}

void continueCalibrate()
{
	if(calibrateState == 0 && calibrateTriDirection < 3)
	{
		startTimer(CALIBRATE_TIMER);
		resetAllQuadratureEncoderCounters();
		if(calibrateForwardBackward == 0)
			driveTriDirection(calibrateTriDirection, CALIBRATE_POWER);
		else
			driveTriDirection(calibrateTriDirection, -CALIBRATE_POWER);
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
			printf("results: %ld %ld %ld\n", motor0sum, motor1sum, motor2sum);
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
	static coord3 startPosition;
	if(latencyState == 0)
	{
		startPosition = robot1cameraPosition;
		moveRobotWorldCoordinates(robot1cameraPosition, (coord3) { 10, 0, 0 });
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
			skill_goToPoint(robot1currentPosition, ball);
			break;
		case testPlayRushGoal:
			play_rushGoal(robot1currentPosition, ball);
			break;
		case testPlayDefense:
			play_followBallOnLine(robot1currentPosition, ball, 410);
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
	case KEY_a + ('p' - 'a'):
		enterStrategyState(testGoToPoint);
		break;
	case KEY_a + ('g' - 'a'):
		enterStrategyState(testPlayRushGoal);
		break;
	case KEY_a + ('d' - 'a'):
		enterStrategyState(testPlayDefense);
		break;
	case KEY_a + ('s' - 'a'):
		startTimer(7);
		printf("Start timer. clock: %f\n", getTime_ms());
		break;
	case KEY_a + ('w' - 'a'):
		printf("Time: %f. clock: %f\n", getTimerTime_ms(7), getTime_ms());
		break;
	case KEY_a + ('z' - 'a'):
		printf("Time: %f. clock: %f\n", getTimerTime_ms(7), getTime_ms());
		sleep_ms(1000);
		printf("Time: %f. clock: %f\n", getTimerTime_ms(7), getTime_ms());
		sleep_ms(1000);
		printf("Time: %f. clock: %f\n", getTimerTime_ms(7), getTime_ms());
		break;
	case KEY_a + ('j' - 'a'):
		printMotorSpeeds();
		break;
	case KEY_a + ('k' - 'a'):
		printPidConstants();
		break;
	case KEY_a + ('l' - 'a'):
		setMotorPidConstants(0, (pidq){250000, 130000, 400000, 180000});
		setMotorPidConstants(1, (pidq){250000, 130000, 400000, 180000});
		setMotorPidConstants(2, (pidq){250000, 130000, 400000, 180000});
		break;
	case KEY_a + ('c' - 'a'):
		startPidCalibrate();
		//startMeasureLatency();
		break;
	case KEY_a + ('e' - 'a'):
		updateCurrentPosition(true);
		printf("Position now: %f %f %f\n", robot1currentPosition.x, robot1currentPosition.y, robot1currentPosition.w);
//
//		showSpeedVsVelocityGraph(0);
//		showSpeedVsVelocityGraph(1);
//		showSpeedVsVelocityGraph(2);
		//driveMotorWithSignedSpeed(0, 100);
		break;
	case KEY_a + ('f' - 'a'):
		driveMotorWithSignedSpeed(0, 0);
		break;
	case KEY_a + ('r' - 'a'):
		emptyBuffer();
		break;
	case KEY_a + ('t' - 'a'):
		printf("Reset encoders\n");
		resetQuadratureEncoderCounters(0);
		break;
	case KEY_a + ('u' - 'a'):
		printf("Encoder: %ld\n", readQuadratureEncoderRegister(0));
		break;
	}
	if ((key & (~MODIFIER_CTRL)) >= KEY_LEFT
			&& (key & (~MODIFIER_CTRL)) <= KEY_DOWN)
	{
		switch (key)
		{
		case KEY_LEFT:
			moveRobotWorldCoordinates(robot1currentPosition, (coord3) { -10, 0, 0 });
			break;
		case KEY_UP:
			moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 0, 10, 0 });
			break;
		case KEY_RIGHT:
			moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 10, 0, 0 });
			break;
		case KEY_DOWN:
			moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 0, -10, 0 });
			break;
		case KEY_LEFT | MODIFIER_CTRL:
			moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 0, 0, .5 });
			break;
		case KEY_RIGHT | MODIFIER_CTRL:
			moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 0, 0, -.5 });
			break;
		}
		overrideForSpecifiedTime(250);
	}
//	printf("key press: %d\n", key);
}
