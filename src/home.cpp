#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "walle/SoccerPoses.h"
#include "walle/Num.h"
#include "stdio.h"
#include "motorControl.h"

using namespace std;
using namespace geometry_msgs;

#define ROBOT_MAX_VXY .5
#define ROBOT_MAX_OMEGA 2*M_PI
#define CONTROL_K_XY 5
#define CONTROL_K_OMEGA 2
#define FIELD_WIDTH 3.048  // in meters
#define FIELD_HEIGHT 1.524 
#define ROBOT_RADIUS 0.10

Vector2d goal;

void param_init()
{
	goal << FIELD_WIDTH/2, 0;
}

int utility_sgn(double val)
{
	return (0 < val) - (val < 0);
}

// utility - saturate velocity
// 	Saturate the commanded velocity .
Vector3d utility_saturateVelocity(Vector3d v)
{
	if(fabs(v(0)) > ROBOT_MAX_VXY)
		v(0) = utility_sgn(v(0)) * ROBOT_MAX_VXY;
	if(fabs(v(1)) > ROBOT_MAX_VXY)
		v(1) = utility_sgn(v(1)) * ROBOT_MAX_VXY;
	if(fabs(v(2)) > ROBOT_MAX_OMEGA)
		v(2) = utility_sgn(v(2)) * ROBOT_MAX_OMEGA;
	cout << "V:" << v << endl;
	return v;
}

double utility_vecLength(Vector2d v)
{
	return sqrt(v(0)*v(0) + v(1)*v(1));
}

Vector2d utility_unitVector(Vector2d v)
{
	return v / utility_vecLength(v);
}

RobotPose utility_toRobotPose(Pose2D robot)
{
	Vector2d pos;
	pos << robot.x, robot.y;
	return (RobotPose){pos, robot.theta};
}

Vector2d utility_toBallPose(Pose2D ball)
{
	Vector2d pos;
	pos << ball.x, ball.y;
	return pos;
}

// skill - follow ball on line
//   Follows the y-position of the ball, while maintaining x-position at x_pos. 
//   Angle always faces the goal.
void skill_followBallOnLine(RobotPose robot, Vector2d ball, double x_pos)
{
	// control x position to stay on current line
	double vx = CONTROL_K_XY * (x_pos - robot.pos(0));

	// control y position to match the ball's y-position
	double vy = CONTROL_K_XY * (ball(1) - robot.pos(1));

	// control angle to face the goal
	Vector2d dirGoal = goal - robot.pos;
	double theta_d = atan2(dirGoal(1), dirGoal(0));
	double omega = -CONTROL_K_OMEGA * (robot.theta - theta_d); 
	
	// Output velocities to motors	
	Vector3d v;
	v << vx, vy, omega;
	v = utility_saturateVelocity(v);
	motorControl_moveRobotWorldVelocities(robot, v);
}

// skill - go to point
//   Travels towards a point. Angle always faces the goal.
void skill_goToPoint(RobotPose robot, Vector2d point)
{
	Vector2d dirPoint = point - robot.pos;
	Vector2d vxy = dirPoint * CONTROL_K_XY;

	// control angle to face the goal
	Vector2d dirGoal = goal - robot.pos;
	double theta_d = atan2(dirGoal(1), dirGoal(0));
	double omega = -CONTROL_K_OMEGA * (robot.theta - theta_d); 

	// Output velocities to motors
	Vector3d v;
	v << vxy, omega;
	v = utility_saturateVelocity(v);
	motorControl_moveRobotWorldVelocities(robot, v);
}

// play - rush goal
//   - go to position behind ball
//   - if ball is between robot and goal, go to goal
// NOTE:  This is a play because it is built on skills, and not control
// commands.  Skills are built on control commands.  A strategy would employ
// plays at a lower level.  For example, switching between offense and
// defense would be a strategy.
void play_rushGoal(RobotPose robot, Vector2d ball)
{
	// normal vector from ball to goal
	Vector2d n = utility_unitVector(goal - ball);

	// compute position 10cm behind ball, but aligned with goal.
	Vector2d position = ball - 0.2*n;

	if(utility_vecLength(position - robot.pos) < .21)
		skill_goToPoint(robot, goal);
	else
		skill_goToPoint(robot, position);
}

void visionCallback(const walle::SoccerPoses& msg)
{
	// robot #1 positions itself behind ball and rushes the goal.
	play_rushGoal(utility_toRobotPose(msg.home1), utility_toBallPose(msg.ball));
 
	// robot #2 stays on line, following the ball, facing the goal
	//skill_followBallOnLine(utility_toRobotPose(msg.home2), utility_toBallPose(msg.ball), -2 * FIELD_WIDTH / 3);
}

int main(int argc, char **argv)
{
	param_init();
	ros::init(argc, argv, "home");
	ros::NodeHandle nh;
	motorControl_init(nh, "robot");
	ros::Subscriber sub = nh.subscribe("/vision", 1, visionCallback);

	ros::spin();
	motorControl_killMotors();
	return 0;
}


