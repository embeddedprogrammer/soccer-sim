#include "calibrate.h"
#include "stdio.h"
#include "time.h"
#include "strategy.h"
#include "skill.h"
#include "play.h"
#include "stdbool.h"

#include "motorControl.h"


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
	{idle, move, calibrate} operatingState;

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
	robot1cameraPosition = pRobot1;
	robot2cameraPosition = pRobot2;
	ball = pBall;
	currentTime = t;
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
		case strategy_none:
			break;
		case testGoToPoint:
			//skill_goToPoint(robot1currentPosition, ball);
			ballAngle = utility_getAngle(utility_3to2(robot1currentPosition), ball);
			skill_goToPoint((coord3){ball.x, ball.y, ballAngle});
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
	}
}

void pressKey(int key)
{
	//printf("key press: %d\n", key);
	//if(key == KEY_a + ('r' - 'a'))
	switch (key)
	{
	case KEY_ESC:
		operatingState = idle;
		calibrate_stop();
		skill_stop();
		printf("operatingState = idle\n");
		motorControl_killMotors();
		break;
	case KEY_ENTER:
		operatingState = move;
		break;
	case KEY_SPACE:
		if(operatingState != idle)
		{
			motorControl_killMotors();
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
		motorControl_spinWheel(0, 127);
		motorControl_spinWheel(1, 127);
		motorControl_spinWheel(2, 127);
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
		motorControl_updateCurrentPosition(true);
		printf("Position now: %f %f %f\n", robot1currentPosition.x,
				robot1currentPosition.y, robot1currentPosition.w);
		//
		//		showSpeedVsVelocityGraph(0);
		//		showSpeedVsVelocityGraph(1);
		//		showSpeedVsVelocityGraph(2);
		//motorControl_driveMotorWithSignedSpeed(0, 100);
		break;
	case KEY_a + ('f' - 'a'):
		motorControl_driveMotorWithSignedSpeed(0, 0);
		break;
	case KEY_a + ('g' - 'a'):
		enterStrategyState(testPlayRushGoal);
		break;
	case KEY_a + ('h' - 'a'):
		robot1currentPosition = robot1cameraPosition;
		break;
	case KEY_a + ('i' - 'a'):
		motorControl_updateCurrentPosition(false);
		break;
	case KEY_a + ('j' - 'a'):
		calibrate_MMatrix(0);
		//motorControl_printMotorSpeeds();
		break;
	case KEY_a + ('k' - 'a'):
		calibrate_MMatrix(1);
		//motorControl_printPidConstants();
		break;
	case KEY_a + ('l' - 'a'):
		calibrate_MMatrix(2);
		break;
	case KEY_a + ('m' - 'a'):
		calibrate_MMatrix(3);
		//motorControl_printMotorDiffs();
		break;
	case KEY_a + ('n' - 'a'):
		calibrate_MMatrix(4);
		//motorControl_setUpdateMode(manuallyUpdate);
		break;
	case KEY_a + ('o' - 'a'):
		calibrate_MMatrix(5);
		//motorControl_setUpdateMode(constantlyUpdate);
		break;
	case KEY_a + ('p' - 'a'):
		enterStrategyState(testGoToPoint);
		break;
	case KEY_a + ('q' - 'a'):
		motorControl_printMotorDiffs();
		break;
	case KEY_a + ('r' - 'a'):
		motorControl_emptyBuffer();
		break;
	case KEY_a + ('s' - 'a'):
		startTimer(7);
		printf("Start timer. clock: %f\n", getTime_ms());
		break;
	case KEY_a + ('t' - 'a'):
		printf("Reset encoders\n");
		motorControl_resetQuadratureEncoderCounters(0);
		break;
	case KEY_a + ('u' - 'a'):
		printf("Encoder: %ld\n", motorControl_readQuadratureEncoderRegister(0));
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
		coord3 userControlledPoint = robot1currentPosition;
		switch (key)
		{
		case KEY_LEFT:
			userControlledPoint.x -= 20;
			break;
		case KEY_UP:
			userControlledPoint.y += 20;
			break;
		case KEY_RIGHT:
			userControlledPoint.x += 20;
			break;
		case KEY_DOWN:
			userControlledPoint.y -= 20;
			break;
		case KEY_LEFT | MODIFIER_CTRL:
			userControlledPoint.w += .5;
			break;
		case KEY_RIGHT | MODIFIER_CTRL:
			userControlledPoint.w -= .5;
			break;
		}
		skill_goToPoint(userControlledPoint);

//		case KEY_LEFT:
//			motorControl_moveRobotWorldCoordinates(robot1currentPosition, (coord3) { -50, 0, 0 });
//			break;
//		case KEY_UP:
//			motorControl_moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 0, 50, 0 });
//			break;
//		case KEY_RIGHT:
//			motorControl_moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 50, 0, 0 });
//			break;
//		case KEY_DOWN:
//			motorControl_moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 0, -50, 0 });
//			break;
//		case KEY_LEFT | MODIFIER_CTRL:
//			motorControl_moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 0, 0, 2 });
//			break;
//		case KEY_RIGHT | MODIFIER_CTRL:
//			motorControl_moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 0, 0, -2 });
//			break;
//		}
//		motorControl_overrideForSpecifiedTime(750);
	}
//	printf("key press: %d\n", key);
}
