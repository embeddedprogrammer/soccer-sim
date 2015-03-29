#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

typedef struct {
	float x, y, w;
} coord3;

typedef struct {
	float m0, m1, m2;
} motorVel3;

typedef struct {
	float x, y;
} coord2;

long manualTimerTicks = 0;
//typedef struct sofs
//{
//    coord3 r0,r1,r2,r3;
//    coord2 b;
//};

//sofs coord_new(float r0x,float r0y,float r0w,float r1x,float r1y,float r1w,float r2x,float r2y,float r2w,float r3x,float r3y,float r3w,float bx,float by)
//{
//    sofs coordinates;
//    coordinates.b.x=bx;
//    coordinates.b.y=by;
//
//    coordinates.r0.x=r0x;
//    coordinates.r0.y=r0y;
//    coordinates.r0.w=r0w;
//
//    coordinates.r1.x=r1x;
//    coordinates.r1.y=r1y;
//    coordinates.r1.w=r1w;
//
//    coordinates.r2.x=r2x;
//    coordinates.r2.y=r2y;
//    coordinates.r2.w=r2w;
//
//    coordinates.r3.x=r3x;
//    coordinates.r3.y=r3y;
//    coordinates.r3.w=r3w;
//    return coordinates;
//}

const coord2 P_GOAL = {130, 0};
const float P_GOAL_WIDTH = 75;
const float P_CONTROL_K_XY = 2; //divided cause in pixels.
const float P_MAX_VELOCITY = 30; // Need to measure

const float P_CONTROL_K_W = 3;
const float P_MAX_SPIN = 2; // Need to measure

void rotate(coord3* v, float theta)
{
	float xNew = v->x * cos(theta) - v->y * sin(theta);
	float yNew = v->x * sin(theta) + v->y * cos(theta);
	v->x = xNew;
	v->y = yNew;
}

coord2 utility_rotate(coord2 v, float theta)
{
	return (coord2){v.x * cos(theta) - v.y * sin(theta), v.x * sin(theta) + v.y * cos(theta)};
}

coord2 utility_3to2(coord3 v)
{
	return (coord2){v.x, v.y};
}

coord3 utility_2to3(coord2 v)
{
	return (coord3){v.x, v.y, 0};
}

float utility_dist1(coord2 v)
{
	return sqrt(v.x*v.x + v.y*v.y);
}

coord2 utility_addVector(coord2 p1, coord2 p2)
{
	return (coord2){p1.x + p2.x, p1.y + p2.y};
}

coord2 utility_subVector(coord2 p1, coord2 p2)
{
	return (coord2){p1.x - p2.x, p1.y - p2.y};
}

coord2 utility_multVector(coord2 v, float m)
{
	return (coord2){v.x * m, v.y * m};
}

coord3 utility_multVector3(coord3 v, float m)
{
	return (coord3){v.x * m, v.y * m, v.w * m};
}

coord3 utility_divVector3(coord3 v, float m)
{
	return (coord3){v.x / m, v.y / m, v.w / m};
}

coord3 utility_subVector3(coord3 v1, coord3 v2)
{
	return (coord3){v1.x - v2.x, v1.y - v2.y, v1.w - v2.w};
}

coord2 utility_getVector(coord2 p1, coord2 p2)
{
	return (coord2){p2.x - p1.x, p2.y - p1.y};
}

float utility_dist(coord2 p1, coord2 p2)
{
	return utility_dist1(utility_getVector(p1, p2));
}

float utility_dist3(coord3 r1, coord3 r2)
{
	return utility_dist((coord2){r1.x, r1.y}, (coord2){r2.x, r2.y});
}

coord2 utility_unitVector(coord2 v)
{
	float d = utility_dist1(v);
	return (coord2){v.x / d, v.y / d};
}

coord2 utility_vectorWithLength(coord2 v, float l)
{
	float d = utility_dist1(v);
	return (coord2){v.x / d * l, v.y / d * l};
}

float utility_angleMod(float angle)
{
	while(angle < 0)
		angle += 2*M_PI;
	return fmod(angle + M_PI, (2*M_PI)) - M_PI;
}

float utility_getAngle1(coord2 v)
{
	return atan2f(v.y, v.x);
}

float utility_getAngle(coord2 p1, coord2 p2)
{
	return utility_getAngle1(utility_getVector(p1, p2));
}

float utility_dotProduct(coord2 v1, coord2 v2)
{
	return v1.x * v2.x + v1.y * v2.y;
}

void sleep_ms(long ms)
{
	long startTime = clock();
	while(clock() - startTime < (ms * 1000)); //CLOCKS_PER_SEC/1000
}

double timerStartTime[10];

double getTime_ms()
{
	struct timeval tp;
	gettimeofday(&tp, NULL);
	return ((double) tp.tv_sec *1e3 + (double) tp.tv_usec * 1e-3);
	//return tp.tv_sec*1000 + tp.tv_usec/1000;
}

double getTime_s()
{
	struct timeval tp;
	gettimeofday(&tp, NULL);
	return ((double) tp.tv_sec + (double) tp.tv_usec * 1e-6);
}

void startTimer(int timerId)
{
	//timerStartTime[timerId] = manualTimerTicks;
	timerStartTime[timerId] = getTime_ms();
	//timerStartTime[timerId] = clock();
}

//long getTimerTime_us(int timerId)
//{
//	return clock() - timerStartTime[timerId];
//}

double getTimerTime_ms(int timerId)
{
	//return (clock() - timerStartTime[timerId]) / 1000;
	//return (manualTimerTicks - timerStartTime[timerId]) * 1000 / 20;
	return (getTime_ms() - timerStartTime[timerId]);
}

double getTimerTime_s(int timerId)
{
	//return (clock() - timerStartTime[timerId]) / 1000;
	//return (manualTimerTicks - timerStartTime[timerId]) * 1000 / 20;
	return (getTime_ms() - timerStartTime[timerId]) / 1000;
}

int sign(int x)
{
	if(x < 0)
		return -1;
	if(x > 0)
		return 1;
	else
		return 0;
}

float fsign(float x)
{
	if(x < 0)
		return -1;
	if(x > 0)
		return 1;
	else
		return 0;
}

