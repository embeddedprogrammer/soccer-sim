#include "skill.h"
#include "play.h"

void play_rushGoal(coord3 robot, coord2 ball)
{
	// normal vector from ball to goal
	coord2 n = utility_unitVector(utility_getVector(ball, P_GOAL));

	// compute position 10cm behind ball, but aligned with goal.
	coord2 position = utility_addVector(ball, utility_multVector(n, -10));

	float goalAngle = utility_getAngle(utility_3to2(robot1currentPosition), P_GOAL);
	if(utility_dist(position, utility_3to2(robot)) < 20) //10.5
	{
		//printf("Go to goal\n");
		skill_goToPoint((coord3){P_GOAL.x, P_GOAL.y, goalAngle});
	}
	else
	{
//		//printf("Go behind ball\n");
//		if (fabs(robot.y - ball.y) > 10)
			skill_goToPoint((coord3){position.x, position.y, goalAngle});
//		else
//		{
//			if(robot.y > ball.y)
//				skill_goToPoint(robot, (coord3){position.x - 10, position.y + 10, 0});
//			else
//				skill_goToPoint(robot, (coord3){position.x - 10, position.y - 10, 0});
//		}
	}
}

void play_blockGoal(coord3 robot, coord2 ball)
{
	coord3 point = (coord3){-P_GOAL.x + 20, ball.y, 0};
	if(point.y > P_GOAL.y + P_GOAL_WIDTH / 2)
		point.y = P_GOAL.y + P_GOAL_WIDTH / 2;
	if(point.y < P_GOAL.y - P_GOAL_WIDTH / 2)
		point.y = P_GOAL.y - P_GOAL_WIDTH / 2;

	skill_goToPoint(point);
}
