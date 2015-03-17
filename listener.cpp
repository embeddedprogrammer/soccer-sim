#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "strategy.h"
#include "beginner_tutorials/Num.h"
#include "time.h"
#include "stdio.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const beginner_tutorials::Num::ConstPtr& msg)
{
	receiveCoords((coord3){msg->x0, msg->y0, msg->w0}, (coord3){msg->x1, msg->y1, msg->w1}, (coord2){msg->xb, msg->yb});
}

void commandCallback(const std_msgs::Int32::ConstPtr& msg)
{
	pressKey(msg->data);
}

int main(int argc, char **argv)
{
	motorControl_init();

	ros::init(argc, argv, "listener");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("chatter", 10, chatterCallback);

	ros::Subscriber sub2 = n.subscribe("command", 10, commandCallback);

	ros::spin();

	printf("Exiting\n");

	killMotors();
	close(serial_fd);
	return 0;
}
