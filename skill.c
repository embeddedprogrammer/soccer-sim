#include "skill.h"

#include "motorControl.h"

coord3 pointToGoTo;
enum skill_states {skill_none, skill_goingToPoint, skillState_circleBall,
	skillState_fetchBall, skillState_dribbleBall} skill_state;

void skill_goToPoint(coord3 point)
{
	skill_state = skill_goingToPoint;
	pointToGoTo = point;
}

void skill_continueGoToPoint(coord3 robot, coord3 point)
{
	coord2 diff = utility_getVector(utility_3to2(robot), utility_3to2(point));
	coord2 velocity = utility_multVector(diff, P_CONTROL_K_XY);

	if(utility_dist1(velocity) > P_MAX_VELOCITY)
		velocity = utility_vectorWithLength(velocity, P_MAX_VELOCITY);

	// face goal
	float omega = -P_CONTROL_K_W * utility_angleMod(robot.w - point.w);
	if(fabs(omega) > P_MAX_SPIN)
		omega = utility_fsign(omega) * P_MAX_SPIN;

//	if(DEBUG_PRINT)
//	{
//		printf("Angle: %f goal: %f omega: %f\n", robot1currentPosition.w, point.w, omega);
//
//		//printf("distance: %f, velocity: %f, omega: %f\n", utility_dist1(diff), utility_dist1(velocity), omega);
//	}

	motorControl_moveRobotWorldCoordinates(robot, (coord3){velocity.x, velocity.y, omega});
}

int skillDir;

void skill_continueCircularBallMotion(float tangentialVelocity, float perpendicularVelocity, bool turnToBounce)
{
	coord2 vecBallToRobot = utility_getVector(ball, utility_3to2(robot1currentPosition));
	coord2 vecTangential = utility_rotate(vecBallToRobot, M_PI / 2);

	coord2 tangentialVelocityVector = utility_vectorWithLength(vecTangential, tangentialVelocity);
	coord2 perpendicularVelocityVector = utility_vectorWithLength(vecBallToRobot, perpendicularVelocity);
	coord2 totalVelocity = utility_addVector(tangentialVelocityVector, perpendicularVelocityVector);

	if(utility_dist1(totalVelocity) > P_MAX_VELOCITY)
		totalVelocity = utility_vectorWithLength(totalVelocity, P_MAX_VELOCITY);

	float desiredAngle;
//	if(!turnToBounce)
//	{
		desiredAngle = utility_getAngle(utility_3to2(robot1currentPosition), ball);
//	}
//	else
//	{
//		float angleModAmt = -tangentialVelocity / P_ROBOT_RADIUS;
//		if(fabs(angleModAmt) > M_PI/4)
//			angleModAmt = utility_fsign(angleModAmt) * M_PI/4;
//		printf("Angle mod2 %f\n", angleModAmt);
//		desiredAngle = utility_getAngle(utility_3to2(robot1currentPosition), P_GOAL) + angleModAmt;
//	}

	float omega = -P_CONTROL_K_W * utility_angleMod(robot1currentPosition.w - desiredAngle);
	if(fabs(omega) > P_MAX_SPIN)
		omega = utility_fsign(omega) * P_MAX_SPIN;

	motorControl_moveRobotWorldCoordinates(robot1currentPosition, (coord3){totalVelocity.x, totalVelocity.y, omega});
}

// dir = +1 (CW) or -1 (CCW).
void skill_circleBall(int dir)
{
	printf("start circle ball\n");
	skill_state = skillState_circleBall;
	skillDir = dir;
}

// dir = +1 (CW) or -1 (CCW).
void skill_fetchBall()
{
	printf("start fetch ball\n");
	skill_state = skillState_fetchBall;
}

void skill_dribbleBall()
{
	printf("start dribble ball\n");
	skill_state = skillState_dribbleBall;
}

void skill_continueCircleBall()
{
	float dist = utility_dist(ball, utility_3to2(robot1currentPosition));
	float desiredDist = 20;
	float tangentialVelocity = 10;
	float perpendicularVelocity = (desiredDist - dist); // factor of 1 for now.

	skill_continueCircularBallMotion(tangentialVelocity * skillDir, perpendicularVelocity, false);
}

void skill_continueFetchBall()
{
	float ballAngle = utility_getAngle(utility_3to2(robot1currentPosition), ball);
	float goalAngle = utility_getAngle(ball, P_GOAL);
	float angleDiff = utility_angleMod(goalAngle - ballAngle);

	float dist = utility_dist(ball, utility_3to2(robot1currentPosition));

//	if(dist < 100)
//	{
		float desiredDist = 20;
		float tangentialVelocity = (angleDiff * dist) * P_CONTROL_K_XY;
		float perpendicularVelocity = (desiredDist - dist) * P_CONTROL_K_XY;

		skill_continueCircularBallMotion(tangentialVelocity, perpendicularVelocity, false);
//	}
//	else
//	{
//		skill_continueGoToPoint(robot1currentPosition, (coord3){ball.x, ball.y, ballAngle});
//	}
}

void skill_continueDribbleBall()
{
	float ballAngle = utility_getAngle(utility_3to2(robot1currentPosition),	ball);
	float goalAngle = utility_getAngle(ball, P_GOAL);
	float angleDiff = utility_angleMod(goalAngle - ballAngle);

	coord2 vectorToBall = utility_subVector(ball, utility_3to2(robot1currentPosition));
	coord2 vectorToGoal = utility_subVector(P_GOAL, utility_3to2(robot1currentPosition));
	float distToLine = utility_findDistanceToLine(ball, utility_3to2(robot1currentPosition), vectorToGoal);


	float dist = utility_dist(ball, utility_3to2(robot1currentPosition));
	float desiredDist = -15;
	float tangentialVelocity = utility_fsign(angleDiff) * distToLine * P_CONTROL_K_XY; //Modified code
	float perpendicularVelocity = (desiredDist - dist) * P_CONTROL_K_XY;

	skill_continueCircularBallMotion(tangentialVelocity, perpendicularVelocity, true);
}

void skill_stop()
{
	skill_state = skill_none;
}

void skill_tick()
{
	switch (skill_state)
	{
	case skill_goingToPoint:
		skill_continueGoToPoint(robot1currentPosition, pointToGoTo);
		break;
	case skillState_circleBall:
		skill_continueCircleBall();
		break;
	case skillState_fetchBall:
		skill_continueFetchBall();
		break;
	case skillState_dribbleBall:
		skill_continueDribbleBall();
		break;
	}
}



