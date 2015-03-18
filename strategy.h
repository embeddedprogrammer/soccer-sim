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
#define CAMERA_WAIT_DIST 10 //pixels
#define MOVEMENT_TIME 1000 //ms

enum operatingStates
	{idle, move} operatingState;

enum strategy_states
	{testGoToPoint, testPlayRushGoal, testPlayDefense} strategy_state;


bool stopRobot = true;

coord3 robot1, robot2;
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

	robot1 = pRobot1;
	robot2 = pRobot2;
	ball = pBall;
	currentTime = t;
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
		float d = 0;
		switch (strategy_state)
		{
		case testGoToPoint:
			d = skill_goToPoint(robot1, ball);
			break;
		case testPlayRushGoal:
			d = play_rushGoal(robot1, ball);
			break;
		case testPlayDefense:
			d = play_followBallOnLine(robot1, ball, 410);
			break;
		}
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
	case KEY_a + ('e' - 'a'):
		showSpeedVsVelocityGraph(0);
		showSpeedVsVelocityGraph(1);
		showSpeedVsVelocityGraph(2);
		//driveMotorWithSignedSpeed(0, 100);
		break;
	case KEY_a + ('f' - 'a'):
		driveMotorWithSignedSpeed(0, 0);
		break;
	case KEY_a + ('r' - 'a'):
		emptyBuffer();
		break;
	}
	if ((key & (~MODIFIER_CTRL)) >= KEY_LEFT
			&& (key & (~MODIFIER_CTRL)) <= KEY_DOWN)
	{
		switch (key)
		{
		case KEY_LEFT:
			moveRobotWorldCoordinates(robot1, (coord3 ) { -10, 0, 0 });
			break;
		case KEY_UP:
			moveRobotWorldCoordinates(robot1, (coord3 ) { 0, 10, 0 });
			break;
		case KEY_RIGHT:
			moveRobotWorldCoordinates(robot1, (coord3 ) { 10, 0, 0 });
			break;
		case KEY_DOWN:
			moveRobotWorldCoordinates(robot1, (coord3 ) { 0, -10, 0 });
			break;
		case KEY_LEFT | MODIFIER_CTRL:
			moveRobotWorldCoordinates(robot1, (coord3 ) { 0, 0, .5 });
			break;
		case KEY_RIGHT | MODIFIER_CTRL:
			moveRobotWorldCoordinates(robot1, (coord3 ) { 0, 0, -.5 });
			break;
		}
		overrideForSpecifiedTime(250);
	}
//	printf("key press: %d\n", key);
}
