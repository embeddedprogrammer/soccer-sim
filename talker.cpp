#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "beginner_tutorials/Num.h"
#include "vision.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
 
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

void feedbackCallback(const beginner_tutorials::Num::ConstPtr& msg)
{
	currentx = msg->x0;
	currenty = msg->y0;
	currentw = msg->w0;
}

double getTime_s()
{
	struct timeval tp;
	gettimeofday(&tp, NULL);
	return ((double) tp.tv_sec + (double) tp.tv_usec * 1e-6);
}

double timerStartTime[10];

double getTime_ms()
{
	struct timeval tp;
	gettimeofday(&tp, NULL);
	return ((double) tp.tv_sec * 1e3 + (double) tp.tv_usec * 1e-3);
}

void startTimer(int timerId)
{
	timerStartTime[timerId] = getTime_ms();
}

double getTimerTime_ms(int timerId)
{
	return (getTime_ms() - timerStartTime[timerId]);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");
	vision_init();

	ros::NodeHandle n;

//	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Publisher chatter_pub = n.advertise<beginner_tutorials::Num>("chatter", 5);
	ros::Publisher commands_pub = n.advertise<std_msgs::Int32>("command", 5);
	ros::Subscriber sub = n.subscribe("feedback", 10, feedbackCallback);

	int loopRate = 30;
	
	ros::Rate loop_rate(loopRate);
	int waitTime = 1;// 000 / loopRate;
	printf("%d", waitTime);

	int count = 0;
	while (ros::ok())
	{
		startTimer(0);
		visionCoords coords = vision_getCoordinates();
//		cout << "time to excecute: " << getTimerTime_ms(0) << endl;
	
		beginner_tutorials::Num msg;
		msg.x0 = coords.robot1.x;
		msg.y0 = coords.robot1.y;
		msg.w0 = coords.robot1.w;
		msg.x1 = coords.robot2.x;
		msg.y1 = coords.robot2.y;
		msg.w1 = coords.robot2.w;
		msg.x2 = 0;
		msg.y2 = 0;
		msg.w2 = 0;
		msg.x3 = 0;
		msg.y3 = 0;
		msg.w3 = 0;
		msg.xb = coords.ball.x;
		msg.yb = coords.ball.y;
		msg.t = coords.t;
		msg.tsys = getTime_s();
		chatter_pub.publish(msg);
		
		int k = waitKey(waitTime);
		keyPress(k);
		if(k != -1)
		{
			std_msgs::Int32 msg2;
			msg2.data = k;
			commands_pub.publish(msg2);
		}
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
}
