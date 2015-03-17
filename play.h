#include "skill.h"

float play_rushGoal(coord3 robot, coord2 ball)
{
	// normal vector from ball to goal
	coord2 n = utility_unitVector(utility_getVector(ball, P_GOAL));

	// compute position 10cm behind ball, but aligned with goal.
	coord2 position = utility_addVector(ball, utility_multVector(n, -10));

	if(utility_dist(position, utility_3to2(robot)) < 20) //10.5
	{
		//printf("Go to goal\n");
		return skill_goToPoint(robot, P_GOAL);
	}
	else
	{
		//printf("Go behind ball\n");
		return skill_goToPoint(robot, position);
	}
}

float play_followBallOnLine(coord3 robot, coord2 ball, float xPos)
{
	 return skill_goToPoint(robot, (coord2){xPos, ball.y});
/*	// control x position to stay on current line
	 float vx = -P_CONTROL_K_XY*(robot.x - xPos);

	 // control y position to match the ball's y-position
	 float vy = -P_CONTROL_K_XY*(robot.y - ball.y);

	 // control angle to -pi/2
	 float theta_d = utility_getAngle(utility_3to2(robot), P_GOAL);
	 float omega = -P_CONTROL_K_W*(robot.w - theta_d);

	moveRobotWorldCoordinates(robot, (coord3){vx, vy, omega});*/
}
