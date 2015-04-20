/*
 * utility.h
 *
 *  Created on: Mar 28, 2015
 *      Author: newuser
 */

#ifndef UTILITY_H_
#define UTILITY_H_


typedef struct {
	float x, y, w;
} coord3;

typedef struct {
	float m0, m1, m2;
} motorVel3;

typedef struct {
	float x, y;
} coord2;

const coord2 P_GOAL = {130, 0};
const coord2 P_START = {-45, 0};

const float P_GOAL_WIDTH = 75;
float P_CONTROL_K_XY = 2;
float P_MAX_VELOCITY = 40;
float P_MAX_VEL_ACC = 5;

float P_CONTROL_K_W = 3;
float P_MAX_SPIN = 1.2;
float P_MAX_SPIN_ACC = .5;

#define P_FIELD_WIDTH 115
#define P_FIELD_HEIGHT 75

void utility_rotate2(coord3* v, float theta);

coord2 utility_rotate(coord2 v, float theta);

coord2 utility_3to2(coord3 v);

coord3 utility_2to3(coord2 v);

float utility_dist1(coord2 v);

coord2 utility_addVector(coord2 p1, coord2 p2);

coord3 utility_addVector3(coord3 v1, coord3 v2);

coord2 utility_subVector(coord2 p1, coord2 p2);

coord2 utility_multVector(coord2 v, float m);

coord3 utility_multVector3(coord3 v, float m);

coord3 utility_divVector3(coord3 v, float m);

coord3 utility_subVector3(coord3 v1, coord3 v2);

coord2 utility_getVector(coord2 p1, coord2 p2);

float utility_dist(coord2 p1, coord2 p2);

float utility_dist3(coord3 r1, coord3 r2);

coord2 utility_unitVector(coord2 v);

coord2 utility_vectorWithLength(coord2 v, float l);

float utility_angleMod(float angle);

float utility_getAngle1(coord2 v);

float utility_getAngle(coord2 p1, coord2 p2);

float utility_dotProduct(coord2 v1, coord2 v2);

void utility_sleep_ms(long ms);

double utility_getTime_ms();

double utility_getTime_s();

void utility_startTimer(int timerId);

double utility_getTimerTime_ms(int timerId);

double utility_getTimerTime_s(int timerId);

void utility_setTime_s(double timeInSeconds);

int utility_sign(int x);

float utility_fsign(float x);

float utility_find_intersection(float x_pos, coord2 ball, coord2 ballV);

coord2 utility_unit_vector_at_angle(float angle);

coord2 utility_projection(coord2 p1, coord2 p2);

float utility_findDistanceToLine(coord2 nearbyPoint, coord2 linePoint, coord2 lineVelocity);

coord2 utility_findClosestPointOnLine(coord2 nearbyPoint, coord2 linePoint, coord2 lineVelocity);

#endif /* UTILITY_H_ */
