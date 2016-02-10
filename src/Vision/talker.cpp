#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "walle/Num.h"
#include "vision.h"
#include <sstream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
 
void feedbackCallback(const walle::Num::ConstPtr& msg)
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

ros::Publisher chatter_pub;
ros::Publisher commands_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

		startTimer(0);
		visionCoords coords = vision_getCoordinates(frame);
//		cout << "time to excecute: " << getTimerTime_ms(0) << endl;
	
		walle::Num msg;
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
		
		int k = waitKey(30);
		keyPress(k);
		if(k != -1)
		{
			std_msgs::Int32 msg2;
			msg2.data = k;
			commands_pub.publish(msg2);
		}
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char **argv)
{
	vision_init();

	ros::init(argc, argv, "talker");
	ros::NodeHandle nh;
	//cv::startWindowThread();

	//Subscribe to topics
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub = it.subscribe("/camera1/image_raw", 1, imageCallback);

	chatter_pub = nh.advertise<walle::Num>("chatter", 5);
	commands_pub = nh.advertise<std_msgs::Int32>("command", 5);
	ros::Subscriber feedback_sub = nh.subscribe("feedback", 10, feedbackCallback);

	ros::spin();
	//cv::destroyWindow("view");
	return 0;
}