#ifndef CALIBRATE_H_
#define CALIBRATE_H_

enum calibrate_states
	{notCalibrating, cameraLatency, pidControl, cameraDistance, calibrateMMatrix} calibrate_state;

void driveTriDirection(int triDirection, int sgn);

void measure1(int a);

void measure2(int a, int b);

void measureTriDirection(int triDirection);

void startCalibrate(calibrate_states state);

void continueCalibrate();

void startMeasureLatency();

void measureLatency();

void calibrate_stop();

void calibrate_MMatrix(int dir);

#endif /* CALIBRATE_H_ */


