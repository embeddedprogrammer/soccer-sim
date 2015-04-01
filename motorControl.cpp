#include "motorControl.h"
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <deque>
#include "utility.h"

#ifndef M_PI
#define M_PI 3.1415926535
#endif

using namespace std;

void printMatrix(const char* str, float M[3][3])
{
	printf("%s:\n%f %f %f\n%f %f %f\n%f %f %f\n", str, M[0][0], M[0][1], M[0][2], M[1][0], M[1][1], M[1][2], M[2][0], M[2][1], M[2][2]);
}

void printCoord3(const char* str, coord3 v)
{
	printf("%s: %.2f %.2f %.2f\n", str, v.x, v.y, v.w);
}

void printCharArray(char command[], int size)
{
	for(int i = 0; i < size; i++)
	{
		printf(" %d", ((int)((unsigned char)command[i])));
	}
	printf("\n");
}

const int wheelAddress[] = {128, 129, 128};
const int wheelRoboclawPosition[] = {1, 1, 0};

int motorControl_calcChecksum(char command[], int size)
{
	int total = 0;
	for(int i = 0; i < size; i++)
	{
		total += ((unsigned char)command[i]);
	}
	unsigned char checksum = total & 0x7f;
	return checksum;
}

int motorControl_calcChecksum2(char command[], int commandSize, char result[], int resultSize)
{
	int total = motorControl_calcChecksum(command, commandSize) + motorControl_calcChecksum(result, resultSize);
	unsigned char checksum = total & 0x7f;
	return checksum;
}

void motorControl_serial_sendMessage(char command[], int size)
{
	write(serial_fd, command, size);
//	printf("command: ");
//	printCharArray(command, size);
}

int motorControl_serial_readMessage(char result[], int size)
{
	int len;
	for(int i = 0; i < 100; i++)
	{
		sleep_ms(2);
		len = read(serial_fd, result, size);
		if(len > 0)
			break;
	}
	if(len != size)
		printf("Error reading from serial port. Only received %d out of %d characters\n", len, size);
	return len;
}

long motorControl_arrayToLong(char array[], int startPos)
{
	return  (long)((((unsigned long)array[startPos + 0]) << 24) |
			(((unsigned long)array[startPos + 1]) << 16) |
			(((unsigned long)array[startPos + 2]) << 8) |
			(((unsigned long)array[startPos + 3])));
}

void motorControl_longToArray(char array[], int startPos, long value)
{
	array[startPos + 0] = (char)((value >> 24) & 0xff);
	array[startPos + 1] = (char)((value >> 16) & 0xff);
	array[startPos + 2] = (char)((value >> 8) & 0xff);
	array[startPos + 3] = (char)((value) & 0xff);
}

void motorControl_emptyBuffer()
{
	char result[2];

	int len;
	int i;
	for(i = 0; i < 100; i++)
	{
		sleep_ms(1);
		if(read(serial_fd, result, 1) != 0)
		{
			if(i == 0)
				printf("Data in buffer: ");
			printf("%d ", ((int)((unsigned char)result[0])));
		}
		else
			break;
	}
	if(i == 0)
		printf("Buffer already empty.\n");
	else
		printf("\n");
}

coord3 lastMotorPositions;

long motorControl_readQuadratureEncoderRegister(int wheelId)
{
	char command[2] = {wheelAddress[wheelId], 16 + wheelRoboclawPosition[wheelId]};
	motorControl_serial_sendMessage(command, 2);

	char result[8];
	int len = motorControl_serial_readMessage(result, 6);

	if(len < 6 || motorControl_calcChecksum2(command, 2, result, 5) != result[5])
	{
		printf("Error: Checksum does not match\n");
		return (long)((wheelId == 0) ? lastMotorPositions.x :
				((wheelId == 1) ? lastMotorPositions.y : lastMotorPositions.w));
	}

	return motorControl_arrayToLong(result, 0);
}

coord3 readQuadratureEncoders()
{
	return (coord3){
		motorControl_readQuadratureEncoderRegister(0),
		motorControl_readQuadratureEncoderRegister(1),
		motorControl_readQuadratureEncoderRegister(2)};
}

void motorControl_printQuadratureEncoderRegisters()
{
	printCoord3("Encoder counters", readQuadratureEncoders());
}


//(Roboclaw should be 0 or 1)
void motorControl_resetQuadratureEncoderCounters(int roboclaw)
{
	char command[3];
	command[0] = 128 + roboclaw;
	command[1] = 20;
	command[2] = motorControl_calcChecksum(command, 2);
	motorControl_serial_sendMessage(command, 3);
}

void motorControl_resetAllQuadratureEncoderCounters()
{
	motorControl_resetQuadratureEncoderCounters(0);
	motorControl_resetQuadratureEncoderCounters(1);
}

long motorControl_readMotorSpeed(int wheelId)
{
	char command[2] = {wheelAddress[wheelId], 18 + wheelRoboclawPosition[wheelId]};
	motorControl_serial_sendMessage(command, 2);

	char result[8];
	motorControl_serial_readMessage(result, 6);

	if(motorControl_calcChecksum2(command, 2, result, 5) != result[5])
		printf("Error: Checksum does not match\n");

	return motorControl_arrayToLong(result, 0);
}

pidq motorControl_readMotorPidConstants(int wheelId)
{
	char command[2] = {wheelAddress[wheelId], 55 + wheelRoboclawPosition[wheelId]};
	motorControl_serial_sendMessage(command, 2);

	char result[20];
	int len = motorControl_serial_readMessage(result, 17);

//	printf("Data received: ");
//	printCharArray(result, 17);

	if(motorControl_calcChecksum2(command, 2, result, 16) != result[16])
		printf("Error: Checksum does not match\n");

	return (pidq){motorControl_arrayToLong(result, 0), motorControl_arrayToLong(result, 4), motorControl_arrayToLong(result, 8), motorControl_arrayToLong(result, 12)};
}

void setMotorPidConstants(int wheelId, pidq val)
{
	char command[19];
	command[0] = wheelAddress[wheelId];
	command[1] = 28 + wheelRoboclawPosition[wheelId];
	motorControl_longToArray(command, 2, val.d);
	motorControl_longToArray(command, 6, val.p);
	motorControl_longToArray(command, 10, val.i);
	motorControl_longToArray(command, 14, val.q);
	command[18] = motorControl_calcChecksum(command, 18);
	motorControl_serial_sendMessage(command, 19);
}

//Drive Commands

//Accepts power values from -127 to +127.
void motorControl_spinWheel(int wheelId, int power)
{
	if(abs(power) > 127)
		power = 127 * sign(power);
	char command[] = {0, 0, 0, 0};
	command[0] = wheelAddress[wheelId];
	int motor = wheelRoboclawPosition[wheelId];
	if(power >= 0)
	{
		command[1] = 0 + motor*4;
		command[2] = power;
	}
	else
	{
		command[1] = 1 + motor*4;
		command[2] = -power;
	}
	command[3] = motorControl_calcChecksum(command, 3);
	//printCharArray(command);
	motorControl_serial_sendMessage(command, sizeof(command));
}

void motorControl_spinWheels(int power0, int power1, int power2)
{
	motorControl_spinWheel(0, -power0);
	motorControl_spinWheel(1, power1);
	motorControl_spinWheel(2, power2);
}

void motorControl_killMotors()
{
	motorControl_spinWheel(0, 0);
	motorControl_spinWheel(1, 0);
	motorControl_spinWheel(2, 0);
	currentBodyVelocity = (coord3){0, 0, 0};
}

void motorControl_driveMotorWithSignedSpeed(int wheelId, long qSpeed)
{
	if(qSpeed == 0) //zero will let wheel spin freely, so we'll move it at a small amount.
		qSpeed = 1;
	char command[7];
	command[0] = wheelAddress[wheelId];
	command[1] = 35 + wheelRoboclawPosition[wheelId];
	motorControl_longToArray(command, 2, qSpeed);
	command[6] = motorControl_calcChecksum(command, 6);
	motorControl_serial_sendMessage(command, 7);
}

void motorControl_printMotorSpeeds()
{
	printf("Motor speeds: %ld, %ld, %ld\n", motorControl_readMotorSpeed(0), motorControl_readMotorSpeed(1), motorControl_readMotorSpeed(2));
}

void motorControl_printPidConstants()
{
	for(int w = 0; w < 3; w++)
	{
		pidq result = motorControl_readMotorPidConstants(w);
		printf("Motor %d: P: %ld I: %ld D: %ld QPPS: %ld\n", w, result.p, result.i, result.d, result.q);
	}
}

void motorControl_showSpeedVsVelocityGraph(int wheelId)
{
	printf("Show power vs. velocity graph for wheel %d\n", wheelId + 1);
	printf("Desired\tActual\n");
	for(int i = 0; i < 127; i++)
	{
//		long speed = 10 * i;
//		driveMotorWithSignedSpeed(wheelId, speed);
		int speed = i;
		motorControl_spinWheel(wheelId, speed);
		sleep_ms(100);
		long actual = motorControl_readMotorSpeed(wheelId);
		printf("%d\t%ld\n", speed, actual);
	}
	motorControl_spinWheel(wheelId, 0);
}

//Two things to calibrate

void motorControl_startCalibrate(coord2 velocity)
{
//	for(int i = 0; i < 127; i++)
//	{
//		driveMotorWithSignedSpeed(wheelId, speed);
//		int speed = i;
//		spinWheel(wheelId, speed);
//		sleep_ms(100);
//		long actual = readMotorSpeed(wheelId);
//		printf("%d\t%ld\n", speed, actual);
//	}
//	spinWheel(wheelId, 0);
}

// Body Frame Calculations
float motorControl_gme(float M[3][3], int i, int j, int r, int c)
{
    return M[(i + r + 1) % 3][(j + c + 1) % 3];
}

float motorControl_getSignedMinorDet(float M[3][3], int i, int j)
{
    return motorControl_gme(M, i, j, 0, 0) * motorControl_gme(M, i, j, 1, 1) - motorControl_gme(M, i, j, 1, 0) * motorControl_gme(M, i, j, 0, 1);
}

float motorControl_getDet(float M[3][3])
{
    return M[0][0] * motorControl_getSignedMinorDet(M, 0, 0) +
        M[0][1] * motorControl_getSignedMinorDet(M, 0, 1) +
        M[0][2] * motorControl_getSignedMinorDet(M, 0, 2);
}

void motorControl_invertMatrix(float M[3][3], float dest[3][3])
{
    float d = motorControl_getDet(M);
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
        	dest[j][i] = motorControl_getSignedMinorDet(M, i, j) / d;
}

coord2 WHEEL_BASE[] = {(coord2){-4.120, 1.895}, (coord2){4.249, 1.843}, (coord2){-0.142, -3.814}};
coord2 WHEEL_DIR[] = {(coord2){-0.540, -0.842}, (coord2){-0.376, 0.926}, (coord2){0.997, 0.080}};

float motorControl_getBodyFrameVectorSpinFactor(coord2 wheelBase, coord2 wheelDir)
{
    return utility_dotProduct(
    		utility_unitVector(utility_rotate(wheelBase, M_PI / 2)), wheelDir)
    		* utility_dist1(wheelBase);
}

//void calcBodyFrameVectors()
//{
//	M[0][0] = WHEEL_DIR[0].x / P_WHEEL_RADIUS;
//	M[0][1] = WHEEL_DIR[0].y / P_WHEEL_RADIUS;
//	M[0][2] = getBodyFrameVectorSpinFactor(WHEEL_BASE[0], WHEEL_DIR[0]) / P_WHEEL_RADIUS;
//	M[1][0] = WHEEL_DIR[1].x / P_WHEEL_RADIUS;
//	M[1][1] = WHEEL_DIR[1].y / P_WHEEL_RADIUS;
//	M[1][2] = getBodyFrameVectorSpinFactor(WHEEL_BASE[1], WHEEL_DIR[1]) / P_WHEEL_RADIUS;
//	M[2][0] = WHEEL_DIR[2].x / P_WHEEL_RADIUS;
//	M[2][1] = WHEEL_DIR[2].y / P_WHEEL_RADIUS;
//	M[2][2] = getBodyFrameVectorSpinFactor(WHEEL_BASE[2], WHEEL_DIR[2]) / P_WHEEL_RADIUS;
//	invertMatrix(M, M_inverse);
//	printMatrix("M Matrix", M);
//	printMatrix("Inverted matrix", M_inverse);
//}

//float M_matlab_inv[3][3] = {{-0.381, -0.51, 1.200}, {-1.232, 1.19, 0.070}, {0.121, 1.07, 0.102}};

void motorControl_calcBodyFrameVectors()
{
	M[0][0] = cos(M_PI/180*WHEEL_ANGLES[0])  / P_WHEEL_RADIUS;
	M[0][1] = -sin(M_PI/180*WHEEL_ANGLES[0]) / P_WHEEL_RADIUS;
	M[0][2] = P_ROBOT_RADIUS				 / P_WHEEL_RADIUS;
	M[1][0] = cos(M_PI/180*WHEEL_ANGLES[1])  / P_WHEEL_RADIUS;
	M[1][1] = -sin(M_PI/180*WHEEL_ANGLES[1]) / P_WHEEL_RADIUS;
	M[1][2] = P_ROBOT_RADIUS				 / P_WHEEL_RADIUS;
	M[2][0] = cos(M_PI/180*WHEEL_ANGLES[2])  / P_WHEEL_RADIUS;
	M[2][1] = -sin(M_PI/180*WHEEL_ANGLES[2]) / P_WHEEL_RADIUS;
	M[2][2] = P_ROBOT_RADIUS				 / P_WHEEL_RADIUS;
//	motorControl_invertMatrix(M_matlab_inv, M);
	motorControl_invertMatrix(M, M_inverse);
	printMatrix("M Matrix", M);
	printMatrix("Inverted matrix", M_inverse);
}

void motorControl_init()
{
	motorControl_calcBodyFrameVectors();
	serial_fd = open("/dev/ttySAC0", O_RDWR);
}

void motorControl_driveMotorsAtSpeed(float motor1Speed, float motor2Speed, float motor3Speed)
{
	long motor1pulsesPerSecond = motor1Speed * pulsesPerRadian;
	long motor2pulsesPerSecond = motor2Speed * pulsesPerRadian;
	long motor3pulsesPerSecond = motor3Speed * pulsesPerRadian;
	motorControl_driveMotorWithSignedSpeed(0, motor1pulsesPerSecond);
	motorControl_driveMotorWithSignedSpeed(1, motor2pulsesPerSecond);
	motorControl_driveMotorWithSignedSpeed(2, motor3pulsesPerSecond);
}

coord3 matrix_mult(float M[3][3], coord3 v)
{
	return (coord3){
		M[0][0]*v.x + M[0][1]*v.y + M[0][2]*v.w,
		M[1][0]*v.x + M[1][1]*v.y + M[1][2]*v.w,
		M[2][0]*v.x + M[2][1]*v.y + M[2][2]*v.w};
}

coord3 motorControl_translateBodyCoordinatesToMotorVelocities(coord3 v)
{
	return matrix_mult(M, v);
}

coord3 motorControl_translateMotorVelocitiesToBodyCoordinates(coord3 motorSpeeds)
{
	return matrix_mult(M_inverse, motorSpeeds);
}

coord3 motorControl_translateWorldCoordinatesToBodyCoordinates(coord3 robot, coord3 v)
{
	rotate(&v, -robot.w + M_PI/2);
	return v;
}

coord3 motorControl_translateBodyCoordinatesToWorldCoordinates(coord3 robot, coord3 v)
{
	rotate(&v, robot.w - M_PI/2);
	return v;
}

void motorControl_moveRobotBodyCoordinates(coord3 v) //int[]* worldVelocities, int[]* wheelVelocities)
{
	currentBodyVelocity = v;
	coord3 motorSpeeds = motorControl_translateBodyCoordinatesToMotorVelocities(v);
//	printCoord3("Body velocities", v);
//	printCoord3("Motor speeds", motorSpeeds);
//	printCoord3("Reverse calc", translateMotorVelocitiesToBodyCoordinates(motorSpeeds));
	motorControl_driveMotorsAtSpeed(motorSpeeds.x, motorSpeeds.y, motorSpeeds.w);
}

void motorControl_moveRobotWorldCoordinates(coord3 robot, coord3 v)
{
	if(!override)
	{
		if(isnan(v.x))
			v.x = 0;
		if(isnan(v.y))
			v.y = 0;
		if(isnan(v.w))
			v.w = 0;
		v = motorControl_translateWorldCoordinatesToBodyCoordinates(robot, v);
		motorControl_moveRobotBodyCoordinates(v);
	}
}



//Tick and control

void motorControl_setOverride(bool val)
{
	override = val;
}

void motorControl_overrideForSpecifiedTime(double t)
{
	//printf("Start timer\n");
	startTimer(MOTOR_CONTROL_OVERRIDE_TIMER);
	timeToOverride = t;
	motorControl_setOverride(true);
}

std::deque<coord3> pastVelocities;

void motorControl_setUpdateMode(motorControl_statePredictionModes mode)
{
	motorControl_statePredictionMode = mode;
}

void motorControl_addBodyVelocityToCurrentPosition(coord3 bodyVelocity)
{
	coord3 worldVelocity = motorControl_translateBodyCoordinatesToWorldCoordinates(robot1currentPosition, bodyVelocity);
	robot1currentPosition.x = robot1currentPosition.x + worldVelocity.x;
	robot1currentPosition.y = robot1currentPosition.y + worldVelocity.y;
	robot1currentPosition.w = robot1currentPosition.w + worldVelocity.w;
}

void motorControl_updateCurrentPosition(bool print)
{
	coord3 currentMotorPositions = readQuadratureEncoders();
	coord3 motorPulses = utility_subVector3(currentMotorPositions, lastMotorPositions);
	coord3 motorRadians = utility_divVector3(motorPulses, pulsesPerRadian);
	lastMotorPositions = currentMotorPositions;
	coord3 bodyCoords = motorControl_translateMotorVelocitiesToBodyCoordinates(motorRadians);

	if(motorControl_statePredictionMode == constantlyUpdate)
	{
		robot1currentPosition = robot1cameraPosition;
		int ticksBehind = (int)(cameraLatency_ms / 1000 * TICKS_PER_SECOND + .5);
		for(int i = 100 - ticksBehind; i < 100 && i < pastVelocities.size(); i++)
		{
			motorControl_addBodyVelocityToCurrentPosition(pastVelocities.at(i));
		}
		pastVelocities.push_back((coord3)bodyCoords);
		if(pastVelocities.size() > 100)
			pastVelocities.pop_front();
	}
	else if(motorControl_statePredictionMode == manuallyUpdate)
	{
		motorControl_addBodyVelocityToCurrentPosition(bodyCoords);
	}
	else if(motorControl_statePredictionMode == printMotorDiffs)
	{
		printCoord3("RadianDiff", motorRadians);
	}
//printf("Time to excecute: %f\n", getTimerTime_ms(9)); //100 us
}



void motorControl_tick()
{
	if(override && getTimerTime_ms(MOTOR_CONTROL_OVERRIDE_TIMER) > timeToOverride)
	{
		override = false;
		motorControl_killMotors();
	}
	motorControl_updateCurrentPosition(false);
}

coord3 lastMotorPositionsTest;
void motorControl_printMotorDiffs()
{
	coord3 currentMotorPositions = readQuadratureEncoders();
	coord3 motorPulses = utility_subVector3(currentMotorPositions, lastMotorPositionsTest);
	coord3 motorPulsesPerRadian = utility_divVector3(motorPulses, 10 * 2 * M_PI);
	lastMotorPositionsTest = currentMotorPositions;
	printCoord3("Pulses Per Radian (with 10 rotations)", motorPulsesPerRadian);
}
