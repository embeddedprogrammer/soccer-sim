#include "skill.h"

#include "motorControl.h"

coord3 pointToGoTo;
enum skill_states {skill_none, skill_goingToPoint} skill_state;

void skill_goToPoint(coord3 point)
{
	skill_state = skill_goingToPoint;
	pointToGoTo = point;
}

void skill_continueGoToPoint(coord3 robot)
{
	coord2 diff = utility_getVector(utility_3to2(robot), utility_3to2(pointToGoTo));
	coord2 velocity = utility_multVector(diff, P_CONTROL_K_XY);

	if(utility_dist1(velocity) > P_MAX_VELOCITY)
		velocity = utility_vectorWithLength(velocity, P_MAX_VELOCITY);

	// face goal
	float omega = -P_CONTROL_K_W * utility_angleMod(robot.w - pointToGoTo.w);
	if(fabs(omega) > P_MAX_SPIN)
		omega = fsign(omega) * P_MAX_SPIN;

	if(DEBUG_PRINT)
	{
		printf("Angle: %f goal: %f omega: %f\n", robot1currentPosition.w, pointToGoTo.w, omega);

		//printf("distance: %f, velocity: %f, omega: %f\n", utility_dist1(diff), utility_dist1(velocity), omega);
	}

	motorControl_moveRobotWorldCoordinates(robot, (coord3){velocity.x, velocity.y, omega});
}

void skill_stop()
{
	skill_state = skill_none;
}

void skill_tick()
{
	if(skill_state = skill_goingToPoint)
	{
		skill_continueGoToPoint(robot1currentPosition);
	}
}



