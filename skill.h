#include "motorControl.h"

//######################################################################
//                    Skills
//######################################################################

float skill_goToPoint(coord3 robot, coord2 point)
{
	coord2 velocity = utility_multVector(utility_getVector(utility_3to2(robot), point), P_CONTROL_K_XY);
	if(utility_dist1(velocity) > P_MAX_VELOCITY)
		velocity = utility_vectorWithLength(velocity, P_MAX_VELOCITY);

	// face goal
	float theta_d = utility_getAngle(utility_3to2(robot), P_GOAL);
	float omega = -P_CONTROL_K_W*(robot.w - theta_d);

	moveRobotWorldCoordinates(robot, (coord3){velocity.x, velocity.y, 0}); //omega
	//P_MAX_SPIN
	return utility_dist(utility_3to2(robot), point);
}



