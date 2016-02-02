#include "skill.h"
#include "play.h"

enum play_states {playState_idle, playState_rushGoal, playState_blockGoal} play_state;

enum rushGoal_states {rushGoal_start, rushGoal_state_getBall, rushGoal_state_pushToGoal}
	rushGoal_state;

void play_stop()
{
	play_state = playState_idle;
}

void play_rushGoal()
{
//	printf("Rush goal\n");
	play_state = playState_rushGoal;
	rushGoal_state = rushGoal_start;
}

void play_blockGoal()
{
	play_state = playState_blockGoal;
}

void play_continue_rushGoal()
{
	float ballAngle = utility_getAngle(utility_3to2(robot1currentPosition), ball);
	float goalAngle = utility_getAngle(ball, P_GOAL);
	float angleDiff = utility_angleMod(goalAngle - ballAngle);

	coord2 vectorToBall = utility_subVector(ball, utility_3to2(robot1currentPosition));
	coord2 vectorToGoal = utility_subVector(P_GOAL, utility_3to2(robot1currentPosition));
	float distToLine = utility_findDistanceToLine(ball, utility_3to2(robot1currentPosition), vectorToGoal);
	float ballBetweenRobotAndGoal = utility_dotProduct(vectorToGoal, vectorToBall);
	//printf("Dist to line: %f Dot Product: %f\n", distToLine, ballBetweenRobotAndGoal);
	switch (rushGoal_state)
	{
	case rushGoal_start:
		skill_fetchBall();
		rushGoal_state = rushGoal_state_getBall;
		break;
	case rushGoal_state_getBall:
		if(distToLine < 10 && ballBetweenRobotAndGoal > 0)
		{
			rushGoal_state = rushGoal_state_pushToGoal;
//			printf("dribble ball\n");
			skill_dribbleBall();
		}
		break;
	case rushGoal_state_pushToGoal:
		if(distToLine > 10 || ballBetweenRobotAndGoal < 0)
		{
			skill_fetchBall();
			rushGoal_state = rushGoal_state_getBall;
//			printf("fetch ball\n");
		}
		break;
	}
}

//void play_rushGoal(coord3 robot, coord2 ball)
//{
//	// normal vector from ball to goal
//	coord2 n = utility_unitVector(utility_getVector(ball, P_GOAL));
//
//	// compute position 10cm behind ball, but aligned with goal.
//	coord2 position = utility_addVector(ball, utility_multVector(n, -10));
//
//	float goalAngle = utility_getAngle(utility_3to2(robot1currentPosition), P_GOAL);
//	if(utility_dist(position, utility_3to2(robot)) < 20) //10.5
//	{
//		//printf("Go to goal\n");
//		skill_goToPoint((coord3){P_GOAL.x, P_GOAL.y, goalAngle});
//	}
//	else
//	{
////		//printf("Go behind ball\n");
////		if (fabs(robot.y - ball.y) > 10)
//			skill_goToPoint((coord3){position.x, position.y, goalAngle});
////		else
////		{
////			if(robot.y > ball.y)
////				skill_goToPoint(robot, (coord3){position.x - 10, position.y + 10, 0});
////			else
////				skill_goToPoint(robot, (coord3){position.x - 10, position.y - 10, 0});
////		}
//	}
//}

void play_continueBlockGoal()
{
	coord3 point = (coord3){-P_GOAL.x + 20, ball.y, 0};
	if(point.y > P_GOAL.y + P_GOAL_WIDTH / 2)
		point.y = P_GOAL.y + P_GOAL_WIDTH / 2;
	if(point.y < P_GOAL.y - P_GOAL_WIDTH / 2)
		point.y = P_GOAL.y - P_GOAL_WIDTH / 2;

	skill_goToPoint(point);
}

void play_tick()
{
	switch (play_state)
	{
	case playState_idle:
		break;
	case playState_rushGoal:
		play_continue_rushGoal();
		break;
	case playState_blockGoal:
		play_continueBlockGoal();
		break;
	}
}
