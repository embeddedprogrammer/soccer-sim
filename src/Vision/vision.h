#include <cmath>
#include <sstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.h>
#include <highgui.h>
#include <time.h>

#include <fstream>
//#include <opencv2/calib3d/calib3d.hpp>


using namespace std;
using namespace cv;

#define FIELD_WIDTH 267
#define FIELD_HEIGHT 361 //in cm

// Values for IP camera
int RED_C[] = {155, 0, 170, 179, 255, 255}; //Red
int BLUE_C[] = {90, 178, 214, 123, 255, 255}; //Blue
int GREEN_C[] = {47, 167, 170, 95, 255, 255}; //Green
int PURPLE_C[] = {101, 84, 219, 116, 255, 255}; //Purple
int YELLOW_C[] = {25, 0, 170, 45, 255, 255}; //yellow

// Values for USB camera
//int RED_C[] = {165, 104, 189, 10, 255, 255}; //Red
//int BLUE_C[] = {86, 82, 174, 106, 255, 255}; //Blue
//int GREEN_C[] = {72, 50, 169, 85, 255, 255}; //Green
//int PURPLE_C[] = {120, 20, 174, 137, 255, 255}; //Purple
//int YELLOW_C[] = {0, 0, 176, 36, 176, 255}; //yellow

int playerC[6];
int opponentC[6];
int ballC[6];

#define DEBUG_CYCLE_COUNT 100

#define KEY_SPACE 1048608
#define KEY_DEL 1114111
#define KEY_1 1048625

#define KEY_ESC 27
#define KEY_ZERO 48
#define KEY_a 97
#define KEY_A 65
#define KEY_LEFT 589649
#define KEY_RIGHT 589651
#define KEY_UP    65362
#define KEY_DOWN  65364

//#define KEY_LEFT  1113937
//#define KEY_UP    1113938
//#define KEY_RIGHT 1113939
//#define KEY_DOWN  1113940
//#define KEY_ESC 1048603
//#define KEY_A 1179713
//#define KEY_ZERO 1048624
//#define KEY_a 1048673

#define KEY_1 1048625
#define KEY_ENTER 1048586
#define MODIFIER_SHIFT (1 << 16)
#define MODIFIER_CTRL (1 << 18)
#define MODIFIER_ALT (1 << 19)

Mat imgHSV;
Mat imgOriginal;

double alpha = 1.0; /**< Simple contrast control [1.0-3.0]*/
int beta = 0;  /**< Simple brightness control [0-100]*/

bool DEBUG_PRINT;
bool erodeAndDilate = true;
int imageShown = 0;
int debugCount = 0;

//VideoCapture cap;

typedef struct {float x, y;} coord2;
typedef struct {float x, y, w;} coord3;
typedef struct {coord3 robot1, robot2; coord2 ball; double t, tSys;} visionCoords;

int swapMultiplier = 1;

void createImageWindow()
{
	namedWindow("Original", WINDOW_NORMAL);
	moveWindow("Original", 1070, 0);
	resizeWindow("Original", 640, 480);
	//resizeWindow("Original", 1280, 960);
}

void setBallColors(int val[])
{
	for(int i = 0; i < 6; i++)
		ballC[i] = val[i];
}

void setPlayerColors(int val[])
{
	for(int i = 0; i < 6; i++)
		playerC[i] = val[i];
}

void setOpponentColors(int val[])
{
	for(int i = 0; i < 6; i++)
		opponentC[i] = val[i];
}

//std::ifstream myFile ("imagefifo",std::ifstream::binary);
//std::vector<char> imageArray ;


void vision_init()
{
	setPlayerColors(RED_C);
	setOpponentColors(BLUE_C);
	setBallColors(YELLOW_C);
	createImageWindow();

//	system("curl -s http://192.168.1.90/mjpg/video.mjpg > imagefifo &");
// our camera - http://192.168.1.10:8080/stream?topic=/image&dummy=param.mjpg

//	cap.open("http://192.168.1.90/mjpg/video.mjpg");
//	cap.open("http://192.168.1.10:8080/stream?topic=/image&dummy=param.mjpg");
	// cap.open(0);

	// if ( !cap.isOpened() )
	// {
	// 	cout << "Cannot open the web cam" << endl;
	// }
	// cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	// cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
}

void createTrackbar(int val[])
{
	cvDestroyWindow("Control");
	namedWindow("Control", WINDOW_NORMAL); //CV_WINDOW_AUTOSIZE); //create a window called "Control"

	createTrackbar("LowH", "Control", &val[0], 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &val[3], 179);

	createTrackbar("LowS", "Control", &val[1], 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &val[4], 255);

	createTrackbar("LowV", "Control", &val[2], 255);//Value (0 - 255)
	createTrackbar("HighV", "Control", &val[5], 255);

	moveWindow("Control", 0, 0);
	resizeWindow("Control", 290, 290);
}

Mat thresholdImage(Mat imgHSV, int val[], int size)
{
	//InputArray min, InputArray max
	Mat imgThresholded;

	if(val[0] > val[3])
	{
		Mat th1, th2;
		inRange(imgHSV, Scalar(0, val[1], val[2]), Scalar(val[3], val[4], val[5]), th1);
		inRange(imgHSV, Scalar(val[0], val[1], val[2]), Scalar(179, val[4], val[5]), th2);
		addWeighted(th1, 1, th2, 1, 0.0, imgThresholded);
	}
	else
	{
		inRange(imgHSV, Scalar(val[0], val[1], val[2]), Scalar(val[3], val[4], val[5]), imgThresholded);
	}

	//morphological opening (removes small objects from the foreground)----------------------------------------------
	if(erodeAndDilate)
	{
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(size, size)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(size, size)));
	}

	return imgThresholded;
}

bool centerClick = false;
coord2 centerOfField = (coord2){0, 0};

coord2 screenToFieldCoords(coord2 screen)
{
//	return (coord2){
//		(screen.x - imgOriginal.cols / 2) * swapMultiplier * FIELD_WIDTH / imgOriginal.cols,
//		-(screen.y - imgOriginal.rows / 2) * swapMultiplier * FIELD_HEIGHT / imgOriginal.rows};
	return (coord2){
		(screen.x - imgOriginal.cols / 2) * FIELD_WIDTH / imgOriginal.cols * swapMultiplier - centerOfField.x,
		-(screen.y - imgOriginal.rows / 2) * FIELD_WIDTH / imgOriginal.cols * swapMultiplier - centerOfField.y};
}

coord2 fieldToScreenCoords(coord2 field)
{
//	return (coord2){
//		(field.x / swapMultiplier / FIELD_WIDTH * imgOriginal.cols) + imgOriginal.cols / 2,
//		(-field.y / swapMultiplier / FIELD_HEIGHT * imgOriginal.rows) + imgOriginal.rows / 2};
	return (coord2){
		((field.x + centerOfField.x) / swapMultiplier / FIELD_WIDTH * imgOriginal.cols) + imgOriginal.cols / 2,
		(-(field.y + centerOfField.y) / swapMultiplier / FIELD_WIDTH * imgOriginal.cols) + imgOriginal.rows / 2};
}

coord2 getCenter(Mat imgOriginal, Moments moment, const string& text)
{
	double dM01 = moment.m01;
	double dM10 = moment.m10;
	double dArea = moment.m00;
	float x, y;

	if (dArea > 10)
	{
		float img_x = dM10 / dArea;
		float img_y = dM01 / dArea;
		putText(imgOriginal, text, cvPoint(img_x, img_y), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, 8, false);
//		cout << "putting text at " << img_x << ", " << img_y << endl;
		return screenToFieldCoords((coord2){img_x, img_y});
	}
	else
	{
		x = NAN;
		y = NAN;
		if(DEBUG_PRINT)
			cout << text << " NOT FOUND" << endl;
	}
	return (coord2){x, y};
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		coord2 fieldxy = screenToFieldCoords((coord2){x, y});
		if(centerClick)
		{
			centerClick = false;
			centerOfField.x = centerOfField.x + fieldxy.x;
			centerOfField.y = centerOfField.y + fieldxy.y;
			cout << "Recentered" << endl;
			
		}
		fieldxy = screenToFieldCoords((coord2){x, y});
		cout << "x: " << fieldxy.x
		   << ", y: " << fieldxy.y
		   << ", H: " << (int)imgHSV.at<Vec3b>(y,x)[0]
			<< ", S: " << (int)imgHSV.at<Vec3b>(y,x)[1]
			<< ", V: " << (int)imgHSV.at<Vec3b>(y,x)[2] << endl;
	}
}

coord3 getRobotCoords(Mat imgOriginal, Mat thresholdImage)
{
	coord2 big = {NAN, NAN};
	coord2 small = {NAN, NAN};

	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;

	//find contours of filtered image using openCV findContours function
	findContours(thresholdImage, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	if (hierarchy.size() > 0)
	{
		int numObjects = hierarchy.size();

		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects >= 2)
		{
			//Sort moments based on area
			int largestMomentsIndex[2] = {-1};
			double largestMomentsArea[2] = {-1};
			bool used[30] = {false};
			for(int i = 0; i < 2; i++)
			{
				int largestMomentIndex = 0;
				double largestMomentArea = 0;
				for(int j = 0; j < numObjects; j++)
				{
					double momentArea = moments((Mat)contours[j]).m00;
					if(momentArea > largestMomentArea && !used[j])
					{
						largestMomentIndex = j;
						largestMomentArea = momentArea;
					}
				}
				used[largestMomentIndex] = true;
				largestMomentsIndex[i] = largestMomentIndex;
				largestMomentsArea[i] = largestMomentArea;
			}

			Moments largestMoment = moments((Mat)contours[largestMomentsIndex[0]]);
			Moments smallestMoment = moments((Mat)contours[largestMomentsIndex[1]]);

			big = getCenter(imgOriginal, largestMoment, "Big");
			small = getCenter(imgOriginal, smallestMoment, "Small");
		}
		else
		{
			if(DEBUG_PRINT)
				cout << "More or less than 2 objects" << endl;
		}
	}

	// Geometry
	float angle;

	if (!isnan(big.x) && !isnan(small.x)){
		angle = atan2(small.y - big.y, small.x - big.x);
		//cout << "angle: " << angle * 180 /3.14 << endl;
	}
	else
	{
		angle = NAN;
		if(DEBUG_PRINT)
			cout << "CANNOT COMPUTE ANGLE" << endl;
	}

	return (coord3)(coord3){(big.x + small.x) / 2, (big.y + small.y) / 2, angle};
}

Mat combineThreshold(Mat threshold)
{
	Mat newImage = Mat::zeros(imgHSV.size(), imgHSV.type() );
	for( int y = 0; y < threshold.rows; y++ )
	{
		for( int x = 0; x < threshold.cols; x++ )
		{
			if(threshold.at<uchar>(y,x) == 255)
			{
				newImage.at<Vec3b>(y,x)[0] = saturate_cast<uchar>(imgOriginal.at<Vec3b>(y,x)[0]);
				newImage.at<Vec3b>(y,x)[1] = saturate_cast<uchar>(imgOriginal.at<Vec3b>(y,x)[1]);
				newImage.at<Vec3b>(y,x)[2] = saturate_cast<uchar>(imgOriginal.at<Vec3b>(y,x)[2]);
			}
			else
			{
				newImage.at<Vec3b>(y,x)[0] = saturate_cast<uchar>(imgOriginal.at<Vec3b>(y,x)[0]) / 3;
				newImage.at<Vec3b>(y,x)[1] = saturate_cast<uchar>(imgOriginal.at<Vec3b>(y,x)[1]) / 3;
				newImage.at<Vec3b>(y,x)[2] = saturate_cast<uchar>(imgOriginal.at<Vec3b>(y,x)[2]) / 3;
			}
		}
	}
	return newImage;
}

bool printxyCoords = false;

void keyPress(int key)
{
	int prevSwapMult = 0;
	switch(key)
	{
	case KEY_ZERO + 0:
		cout << "now showing original image" << endl;
		imageShown = 0;
		cvDestroyWindow("Control");
		break;
	case KEY_ZERO + 1:
		cout << "now showing player threshold of image" << endl;
		imageShown = 1;
		createTrackbar(playerC);
		break;
	case KEY_ZERO + 2:
		cout << "now showing opponent threshold of image" << endl;
		imageShown = 2;
		createTrackbar(opponentC);
		break;
	case KEY_ZERO + 3:
		cout << "now showing ball threshold of image" << endl;
		imageShown = 3;
		createTrackbar(ballC);
		break;
	case KEY_ZERO + 9:
		cout << "now showing no image" << endl;
		imageShown = 9;
		cvDestroyWindow("Control");
		break;
	case KEY_LEFT | MODIFIER_ALT:
		prevSwapMult = swapMultiplier;
		swapMultiplier = -1;
		centerOfField.x *= (swapMultiplier * prevSwapMult); 
		centerOfField.y *= (swapMultiplier * prevSwapMult); 
		cout << "now going towards goal on left of field" << endl;
		break;
	case KEY_RIGHT | MODIFIER_ALT:
		prevSwapMult = swapMultiplier;
		swapMultiplier = 1;
		centerOfField.x *= (swapMultiplier * prevSwapMult); 
		centerOfField.y *= (swapMultiplier * prevSwapMult); 
		cout << "now going towards goal on right of field" << endl;
		break;
	case (KEY_a + 'r' - 'a') | MODIFIER_CTRL:
		setPlayerColors(RED_C);
		cout << "PLAYER: RED" << endl;
		break;
	case (KEY_a + 'g' - 'a') | MODIFIER_CTRL:
		setPlayerColors(GREEN_C);
		cout << "PLAYER: GREEN" << endl;
		break;
	case (KEY_a + 'b' - 'a') | MODIFIER_CTRL:
		setPlayerColors(BLUE_C);
		cout << "PLAYER: BLUE" << endl;
		break;
	case (KEY_a + 'p' - 'a') | MODIFIER_CTRL:
		setPlayerColors(PURPLE_C);
		cout << "PLAYER: PURPLE" << endl;
		break;
	case (KEY_a + 'r' - 'a') | MODIFIER_ALT:
		setOpponentColors(RED_C);
		cout << "OPPONENT: RED" << endl;
		break;
	case (KEY_a + 'g' - 'a') | MODIFIER_ALT:
		setOpponentColors(GREEN_C);
		cout << "OPPONENT: GREEN" << endl;
		break;
	case (KEY_a + 'b' - 'a') | MODIFIER_ALT:
		setOpponentColors(BLUE_C);
		cout << "OPPONENT: BLUE" << endl;
		break;
	case (KEY_a + 'p' - 'a') | MODIFIER_ALT:
		setOpponentColors(PURPLE_C);
		cout << "OPPONENT: PURPLE" << endl;
		break;
	case (KEY_a + 'e' - 'a') | MODIFIER_ALT:
		erodeAndDilate = true;
		cout << "Erode and Dilate" << endl;
		break;
	case (KEY_a + 'd' - 'a') | MODIFIER_ALT:
		erodeAndDilate = false;
		cout << "Showing raw threshold" << endl;
		break;
	case (KEY_a + 'c' - 'a') | MODIFIER_ALT:
		centerClick = true;
		cout << "Click in the center to calibrate" << endl;
		break;
	case (KEY_a + 'j' - 'a'):
		printxyCoords = true;
		break;
	case (KEY_a + 'k' - 'a'):
		printxyCoords = true;
		break;
	case (KEY_a + 'l' - 'a'):
		printxyCoords = true;
		break;
	case (KEY_a + 'm' - 'a'):
		printxyCoords = true;
		break;
	case (KEY_a + 'n' - 'a'):
		printxyCoords = true;
		break;
	case (KEY_a + 'o' - 'a'):
		printxyCoords = true;
		break;
	case (KEY_ESC):
		printxyCoords = false;
		break;
  case -1:
    break;
  default:
    printf("%d\n", key);
	}
}


typedef struct {
    unsigned int sec;
    unsigned int nsec;
} Time;

// This is the ugly parsing image function
Time getNextImage(std::ifstream & myFile, std::vector<char> & imageArray)
{
	imageArray.clear();
	char buffer[4];
	Time timestamp;
	bool foundImage = false;
	while (!foundImage)
	{
		myFile.read(buffer, 1);
		if ((*buffer) == (char) 0xFF)
		{
			myFile.read(buffer, 1);
			if ((*buffer) == (char) 0xD8)
			{
				//printf("found start of image \n");
				imageArray.push_back((char) 0xFF);
				imageArray.push_back((char) 0xD8);
				while (1)
				{
					myFile.read(buffer, 1);
					imageArray.push_back(*buffer);
					if ((*buffer) == (char) 0xFF)
					{
						myFile.read(buffer, 1);
						imageArray.push_back(*buffer);
						if ((*buffer) == (char) 0xFE)
						{
							myFile.read(buffer, 4);
							imageArray.push_back(*buffer);
							imageArray.push_back(*(buffer + 1));
							imageArray.push_back(*(buffer + 2));
							imageArray.push_back(*(buffer + 3));
							if ((*(buffer + 3)) == (char) 0x01)
							{
								myFile.read(buffer, 4);
								unsigned int sec = 0;
								for (int i = 0; i < 4; i++)
								{
									imageArray.push_back(*(buffer + i));
									sec <<= 8;
									sec += *(unsigned char*) (void*) (buffer + i);
								}

								myFile.read(buffer, 1);
								unsigned int hundreds = 0;
								imageArray.push_back(*buffer);
								hundreds += *(unsigned char*) (void*) buffer;
								timestamp.sec = sec;
								timestamp.nsec = hundreds * 10000000;
							}

						}
						else if ((*buffer) == (char) 0xD9)
						{
							//printf("found end of image\n");
							foundImage = true;
							return timestamp;
						}
					}
				}
			}
		}
	}
	return Time { 0, 0 };
}

float currentx, currenty, currentw;

void arrowedLine(Mat img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int line_type=8, int shift=0, double tipLength=0.1)
{
    const double tipSize = norm(pt1-pt2)*tipLength; // Factor to normalize the size of the tip depending on the length of the arrow
    line(img, pt1, pt2, color, thickness, line_type, shift);
    const double angle = atan2( (double) pt1.y - pt2.y, (double) pt1.x - pt2.x );
    Point p(cvRound(pt2.x + tipSize * cos(angle + CV_PI / 4)),
    cvRound(pt2.y + tipSize * sin(angle + CV_PI / 4)));
    line(img, p, pt2, color, thickness, line_type, shift);
    p.x = cvRound(pt2.x + tipSize * cos(angle - CV_PI / 4));
    p.y = cvRound(pt2.y + tipSize * sin(angle - CV_PI / 4));
    line(img, p, pt2, color, thickness, line_type, shift);
}

void rotate(coord3* v, float theta)
{
	float xNew = v->x * cos(theta) - v->y * sin(theta);
	float yNew = v->x * sin(theta) + v->y * cos(theta);
	v->x = xNew;
	v->y = yNew;
}

coord3 motorControl_translateWorldCoordinatesToBodyCoordinates(coord3 robot, coord3 v)
{
	rotate(&v, -robot.w + M_PI/2);
	return v;
}

coord3 lastRobot1;

visionCoords vision_getCoordinates(Mat frame)
{
	imgOriginal = frame;

	DEBUG_PRINT = (debugCount == 0);
	debugCount = (debugCount + 1) % DEBUG_CYCLE_COUNT;

	// if (!bSuccess) //if not success, break loop
	// {
	// 	cout << "Cannot read a frame from video stream" << endl;
	// 	return (visionCoords){(coord3){0, 0, 0}, (coord3){0, 0, 0}, (coord2){0, 0}, 0, 0};
	// }

//	Time timestamp = getNextImage(myFile, imageArray);
//	imgOriginal = imdecode(imageArray,CV_LOAD_IMAGE_COLOR);

	DEBUG_PRINT = (debugCount == 0);
	debugCount = (debugCount + 1) % DEBUG_CYCLE_COUNT;

// //------------------------------------------UNDISTORT------------------------------
// 	Mat camera_matrix = (Mat_<double>(3,3) << 5.1314846582801022e+02, 0., 3.1558730401691332e+02, 0.,
// 	                      5.1394244582336182e+02, 2.6671553993693823e+02, 0., 0., 1.);

// 	Mat distortion_coeff = (Mat_<double>(1,5) << -3.6477907232256079e-01, 1.8476332607237927e-01,
// 	                         3.9894289612051551e-03, -3.3921806480293636e-04,
// 	                         -6.1411439476998939e-02);

// 	Mat new_camera_matrix = (Mat_<double>(3,3) << 5.1314846582801022e+02, 0., 3.1558730401691332e+02 + 50, 0.,
//             				5.1394244582336182e+02, 2.6671553993693823e+02 + 25, 0., 0., 1.);
// 	Mat map1, map2;
// 	Mat R = (Mat_<double>(3,3) << 1,0,0,0,1,0,0,0,1);
// 	Mat undistortedImg;
// 	initUndistortRectifyMap(camera_matrix, distortion_coeff, R, new_camera_matrix, Size(750, 530), CV_32FC1, map1, map2);

// 	remap(imgOriginal, undistortedImg, map1, map2, INTER_LINEAR, BORDER_CONSTANT, 0);
// 	imgOriginal = undistortedImg;

//	Mat undistortedImg;
//	undistort(imgOriginal, undistortedImg, camera_matrix, distortion_coeff);
//	imgOriginal = undistortedImg;
//------------------------------------------UNDISTORT----------------------------------

//	Rect myROI(140, 0, 940, 720);//Full Field
//	//Rect myROI(600,50, 450,300);//Upper Right Field
//	imgOriginal = imgOriginal(myROI);
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

	Mat playerI = thresholdImage(imgHSV, playerC, 2);
	Mat opponentI = thresholdImage(imgHSV, opponentC, 2);
	Mat ballI = thresholdImage(imgHSV, ballC, 2);

	coord2 result = fieldToScreenCoords((coord2){currentx, currenty});
	coord2 offset = fieldToScreenCoords((coord2){currentx + 5 * cos(currentw), currenty + 5 * sin(currentw)});
	circle(imgOriginal, Point(result.x, result.y), 3, Scalar(255, 0, 0));
	arrowedLine(imgOriginal, Point(result.x, result.y), Point(offset.x, offset.y), Scalar(255, 0, 0));

	if(imageShown == 0)
		imshow("Original", imgOriginal);
	else if(imageShown == 1)
		imshow("Original", combineThreshold(playerI));
	else if(imageShown == 2)
		imshow("Original", combineThreshold(opponentI));
	else if(imageShown == 3)
		imshow("Original", combineThreshold(ballI));
	setMouseCallback("Original", CallBackFunc, NULL);

	coord3 robot1 = getRobotCoords(imgOriginal, playerI);
	coord3 robot2 = getRobotCoords(imgOriginal, opponentI);
	coord2 ball = getCenter(imgOriginal, moments(ballI), "BALL");
	//cout << "t: " << timestamp.sec << endl;

	if(printxyCoords)
	{
		/*coord3 bodyCoord = motorControl_translateWorldCoordinatesToBodyCoordinates
			((coord3){(robot1.x + lastRobot1.x) / 2, (robot1.y + lastRobot1.y) / 2, (robot1.w + lastRobot1.w) / 2},
			(coord3){robot1.x - lastRobot1.x, robot1.y - lastRobot1.y, robot1.w - lastRobot1.w});
		cout << "bodyCoord " <<
			(bodyCoord.x) << " " <<
			(bodyCoord.y) << " " <<
			(bodyCoord.w) << " " << endl;*/

		cout << "robot " <<
			(robot1.x) << " " <<
			(robot1.y) << " " <<
			(robot1.w) << " " << endl;
	}

	lastRobot1 = robot1;

	return (visionCoords){robot1, robot2, ball, 0, 0};;
}
