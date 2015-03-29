#include "motorControl.h"

//######################################################################
//                    Skills
//######################################################################

void skill_goToPoint(coord3 robot, coord3 point)
{
	coord2 diff = utility_getVector(utility_3to2(robot), utility_3to2(point));
	coord2 velocity = utility_multVector(diff, P_CONTROL_K_XY);

	if(utility_dist1(velocity) > P_MAX_VELOCITY)
		velocity = utility_vectorWithLength(velocity, P_MAX_VELOCITY);

	// face goal
	float omega = -P_CONTROL_K_W * utility_angleMod(robot.w - point.w);
	if(fabs(omega) > P_MAX_SPIN)
		omega = fsign(omega) * P_MAX_SPIN;

	if(DEBUG_PRINT)
	{
		printf("Angle: %f goal: %f omega: %f\n", robot1currentPosition.w, point.w, omega);

		//printf("distance: %f, velocity: %f, omega: %f\n", utility_dist1(diff), utility_dist1(velocity), omega);
	}

	moveRobotWorldCoordinates(robot, (coord3){velocity.x, velocity.y, omega});
}



