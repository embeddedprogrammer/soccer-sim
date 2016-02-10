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

bool stopRobot = true;

double currentTime;

#define KALMAN_SAMPLES 5
coord3 recentCoords[KALMAN_SAMPLES];

int ballOffFieldTicks;

void control_receiveCoords(coord3 pRobot1, coord3 pRobot2, coord2 pBall, double t)
{
	robot1cameraPositionAverage = (coord3){0, 0, 0};
	for(int i = KALMAN_SAMPLES - 1; i > 0; i--)
	{
		recentCoords[i] = recentCoords[i - 1];
	}
	recentCoords[0] = pRobot1;
	for(int i = 0; i < KALMAN_SAMPLES; i++)
	{
		robot1cameraPositionAverage.x += recentCoords[i].x / KALMAN_SAMPLES;
		robot1cameraPositionAverage.y += recentCoords[i].y / KALMAN_SAMPLES;
		robot1cameraPositionAverage.w += recentCoords[i].w / KALMAN_SAMPLES;
	}
	robot1cameraPosition = pRobot1;
	robot2cameraPosition = pRobot2;
	if(isnan(pBall.x))
	{
		ballOffFieldTicks++;
		if(ballOffFieldTicks > 15)
			ball = pBall;
	}
	else
	{
		ball = pBall;
		ballOffFieldTicks = 0;
	}
//	printf("%.2f %.2f %.2f\n", ball.x, ball.y, t);
	currentTime = t;
}

void control_stopSMs()
{
	calibrate_stop();
	skill_stop();
	play_stop();
	strategy_stop();
}

void control_pressKey(int key)
{
	//printf("key press: %d\n", key);
	//if(key == KEY_a + ('r' - 'a'))
	switch (key)
	{
	case KEY_ESC:
		control_stopSMs();
		motorControl_killMotors();
		printf("operatingState = idle\n");
		break;
	case KEY_ENTER:
		break;
	case KEY_SPACE:
		break;
	case KEY_a:
		motorControl_spinWheel(0, 127);
		motorControl_spinWheel(1, 127);
		motorControl_spinWheel(2, 127);
		break;
	case KEY_a + ('b' - 'a'):
		motorControl_setUpdateMode(manuallyUpdate);
		break;
	case KEY_a + ('c' - 'a'):
		calibrate_startCalibrate(pidControl);
		break;
	case KEY_a + ('d' - 'a'):
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
		P_CONTROL_K_XY = 3;
		P_MAX_VELOCITY = 50;
		P_MAX_VEL_ACC = 10;

		P_CONTROL_K_W = 3;
		P_MAX_SPIN = 1.5;
		P_MAX_SPIN_ACC = .5;
		//motorControl_driveMotorWithSignedSpeed(0, 0);
		break;
	case KEY_a + ('g' - 'a'):
		play_rushGoal();
		//enterStrategyState(testPlayRushGoal);
		break;
	case KEY_a + ('h' - 'a'):
		robot1currentPosition = robot1cameraPosition;
		break;
	case KEY_a + ('i' - 'a'):
		motorControl_updateCurrentPosition(false);
		break;
	case KEY_a + ('j' - 'a'):
		motorControl_resetAllQuadratureEncoderCounters();
		//calibrate_MMatrix(0);
		//motorControl_printMotorSpeeds();
		break;
	case KEY_a + ('k' - 'a'):
		//setMotorPidConstants(0, (pidq){250000, 130000, 400000, 83000});
		//setMotorPidConstants(1, (pidq){250000, 130000, 400000, 86000});
		//setMotorPidConstants(2, (pidq){250000, 130000, 400000, 66000});
//		calibrate_MMatrix(1);
		//motorControl_printPidConstants();
		break;
	case KEY_a + ('l' - 'a'):
		skill_circleBall(1);
		//calibrate_MMatrix(2);
		break;
	case KEY_a + ('m' - 'a'):
		calibrate_MMatrix(3);
		//motorControl_printMotorDiffs();
		break;
	case KEY_a + ('n' - 'a'):
		calibrate_startWScaleCalibrate();
		break;
	case KEY_a + ('o' - 'a'):
		calibrate_startXYScaleCalibrate();
		break;
	case KEY_a + ('p' - 'a'):
		//printf("Rand #: %f\n", ((float)rand()) / RAND_MAX);

		calibrate_startRoboclawErrCalibrate();
		break;
	case KEY_a + ('q' - 'a'):
		motorControl_printMotorDiffs();
		break;
	case KEY_a + ('r' - 'a'):
		control_stopSMs();
		skill_goToPoint((coord3){P_START.x, P_START.y, 0});
		//motorControl_emptyBuffer();
		break;
	case KEY_a + ('s' - 'a'):
		strategy_bestStrategy();
//		startTimer(7);
//		printf("Start timer. clock: %f\n", getTime_ms());
		break;
	case KEY_a + ('t' - 'a'):
		printf("Reset encoders\n");
		motorControl_resetAllQuadratureEncoderCounters();
		break;
	case KEY_a + ('u' - 'a'):
		printf("Encoder: %ld\n", motorControl_readQuadratureEncoderRegister(0));
		break;
	case KEY_a + ('v' - 'a'):
		calibrate_startSpeedCalibrate();
		break;
	case KEY_a + ('w' - 'a'):
		//motorControl_printPidConstants();
		break;
	case KEY_a + ('x' - 'a'):
		calibrate_startCalibrate(cameraDistance);
		break;
	case KEY_a + ('y' - 'a'):
		motorControl_setUpdateMode(constantlyUpdate);
		break;
	case KEY_a + ('z' - 'a'):
		calibrate_startMeasureLatency();
		break;
	}
	if ((key & (~MODIFIER_CTRL)) >= KEY_LEFT
			&& (key & (~MODIFIER_CTRL)) <= KEY_DOWN)
	{
//		coord3 userControlledPoint = robot1currentPosition;
//		switch (key)
//		{
//		case KEY_LEFT:
//			userControlledPoint.x -= 20;
//			break;
//		case KEY_UP:
//			userControlledPoint.y += 20;
//			break;
//		case KEY_RIGHT:
//			userControlledPoint.x += 20;
//			break;
//		case KEY_DOWN:
//			userControlledPoint.y -= 20;
//			break;
//		case KEY_LEFT | MODIFIER_CTRL:
//			userControlledPoint.w += .5;
//			break;
//		case KEY_RIGHT | MODIFIER_CTRL:
//			userControlledPoint.w -= .5;
//			break;
//		}
//		skill_goToPoint(userControlledPoint);
		motorControl_setOverride(false);
		switch (key)
		{
		case KEY_LEFT:
			motorControl_moveRobotWorldCoordinates(robot1currentPosition, (coord3) { -40, 0, 0 });
			break;
		case KEY_UP:
			motorControl_moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 0, 40, 0 });
			break;
		case KEY_RIGHT:
			motorControl_moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 40, 0, 0 });
			break;
		case KEY_DOWN:
			motorControl_moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 0, -40, 0 }); //50
			break;
		case KEY_LEFT | MODIFIER_CTRL:
			motorControl_moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 0, 0, 3 }); //2
			break;
		case KEY_RIGHT | MODIFIER_CTRL:
			motorControl_moveRobotWorldCoordinates(robot1currentPosition, (coord3) { 0, 0, -3 });
			break;
		}
		motorControl_overrideForSpecifiedTime(750);
	}
//	printf("key press: %d\n", key);
}
