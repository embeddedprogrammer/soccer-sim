#ifndef CALIBRATE_H_
#define CALIBRATE_H_

enum calibrate_states
	{notCalibrating, cameraLatency, pidControl, cameraDistance, calibrateMMatrix,
	state_calibrateSpeed, state_XYScaleCalibrate, state_WScaleCalibrate, state_RoboclawErr} calibrate_state;

void calibrate_driveTriDirection(int triDirection, int sgn);

void calibrate_measure1(int a);

void calibrate_measure2(int a, int b);

void calibrate_measureTriDirection(int triDirection);

void calibrate_startCalibrate(calibrate_states state);

void calibrate_continueCalibrate();

void calibrate_startMeasureLatency();

void calibrate_measureLatency();

void calibrate_stop();

void calibrate_MMatrix(int dir);

void calibrate_startSpeedCalibrate();

void calibrate_startWScaleCalibrate();

void calibrate_startXYScaleCalibrate();

void calibrate_startRoboclawErrCalibrate();

#endif /* CALIBRATE_H_ */


