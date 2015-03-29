#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <iostream>
#include "utility.h"

using namespace std;

typedef struct {
	long p, i, d, q;
} pidq;

#ifndef M_PI
#define M_PI 3.1415926535
#endif

double cameraLatency_ms = 580;
//float pulsesPerRadian = (19456 / 2 / M_PI);
float pulsesPerRadian = (198100 / 10 / 2 / M_PI); //Accurate to .5%

// Everything is in centimeters.
#define P_WHEEL_RADIUS 2.7 //Measured
#define P_ROBOT_RADIUS 8.4 //Measured
#define P_KICKER_DIST 6.35 //Measured
const float WHEEL_ANGLES[] = {120, 240, 0};
float M[3][3];
float M_inverse[3][3];
float wheelPowerNeededPerVelocityUnit = 20;
int serial_fd;

#define MOTOR_CONTROL_OVERRIDE_TIMER 7
#define TICKS_PER_SECOND 25
bool override = false;
double timeToOverride;
coord3 currentBodyVelocity;
coord3 robot1currentPosition;
coord3 robot1cameraPosition, robot2cameraPosition;
bool DEBUG_PRINT;

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
		cout << " " << ((int)((unsigned char)command[i]));
	}
	cout << endl;
}

const int wheelAddress[] = {128, 129, 128};
const int wheelRoboclawPosition[] = {1, 1, 0};

int calcChecksum(char command[], int size)
{
	int total = 0;
	for(int i = 0; i < size; i++)
	{
		total += ((unsigned char)command[i]);
	}
	unsigned char checksum = total & 0x7f;
	return checksum;
}

int calcChecksum2(char command[], int commandSize, char result[], int resultSize)
{
	int total = calcChecksum(command, commandSize) + calcChecksum(result, resultSize);
	unsigned char checksum = total & 0x7f;
	return checksum;
}

void serial_sendMessage(char command[], int size)
{
	write(serial_fd, command, size);
//	printf("command: ");
//	printCharArray(command, size);
}

int serial_readMessage(char result[], int size)
{
	int len;
	for(int i = 0; i < 100; i++)
	{
		sleep_ms(1);
		len = read(serial_fd, result, size);
		if(len > 0)
			break;
	}
	if(len != size)
		printf("Error reading from serial port. Only received %d out of %d characters\n", len, size);
	return len;
}

long arrayToLong(char array[], int startPos)
{
	return  (long)((((unsigned long)array[startPos + 0]) << 24) |
			(((unsigned long)array[startPos + 1]) << 16) |
			(((unsigned long)array[startPos + 2]) << 8) |
			(((unsigned long)array[startPos + 3])));
}

void longToArray(char array[], int startPos, long value)
{
	array[startPos + 0] = (char)((value >> 24) & 0xff);
	array[startPos + 1] = (char)((value >> 16) & 0xff);
	array[startPos + 2] = (char)((value >> 8) & 0xff);
	array[startPos + 3] = (char)((value) & 0xff);
}

void emptyBuffer()
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

long readQuadratureEncoderRegister(int wheelId)
{
	char command[2] = {wheelAddress[wheelId], 16 + wheelRoboclawPosition[wheelId]};
	serial_sendMessage(command, 2);

	char result[8];
	serial_readMessage(result, 6);

	if(calcChecksum2(command, 2, result, 5) != result[5])
		printf("Error: Checksum does not match\n");

	return arrayToLong(result, 0);
}

coord3 readQuadratureEncoders()
{
	return (coord3){
		readQuadratureEncoderRegister(0),
		readQuadratureEncoderRegister(1),
		readQuadratureEncoderRegister(2)};
}

void printQuadratureEncoderRegisters()
{
	printCoord3("Encoder counters", readQuadratureEncoders());
}


//(Roboclaw should be 0 or 1)
void resetQuadratureEncoderCounters(int roboclaw)
{
	char command[3];
	command[0] = 128 + roboclaw;
	command[1] = 20;
	command[2] = calcChecksum(command, 2);
	serial_sendMessage(command, 3);
}

void resetAllQuadratureEncoderCounters()
{
	resetQuadratureEncoderCounters(0);
	resetQuadratureEncoderCounters(1);
}

long readMotorSpeed(int wheelId)
{
	char command[2] = {wheelAddress[wheelId], 18 + wheelRoboclawPosition[wheelId]};
	serial_sendMessage(command, 2);

	char result[8];
	serial_readMessage(result, 6);

	if(calcChecksum2(command, 2, result, 5) != result[5])
		printf("Error: Checksum does not match\n");

	return arrayToLong(result, 0);
}

pidq readMotorPidConstants(int wheelId)
{
	char command[2] = {wheelAddress[wheelId], 55 + wheelRoboclawPosition[wheelId]};
	serial_sendMessage(command, 2);

	char result[20];
	int len = serial_readMessage(result, 17);

//	printf("Data received: ");
//	printCharArray(result, 17);

	if(calcChecksum2(command, 2, result, 16) != result[16])
		printf("Error: Checksum does not match\n");

	return (pidq){arrayToLong(result, 0), arrayToLong(result, 4), arrayToLong(result, 8), arrayToLong(result, 12)};
}

void setMotorPidConstants(int wheelId, pidq val)
{
	char command[19];
	command[0] = wheelAddress[wheelId];
	command[1] = 28 + wheelRoboclawPosition[wheelId];
	longToArray(command, 2, val.d);
	longToArray(command, 6, val.p);
	longToArray(command, 10, val.i);
	longToArray(command, 14, val.q);
	command[18] = calcChecksum(command, 18);
	serial_sendMessage(command, 19);
}

//Drive Commands

//Accepts power values from -127 to +127.
void spinWheel(int wheelId, int power)
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
	command[3] = calcChecksum(command, 3);
	//printCharArray(command);
	serial_sendMessage(command, sizeof(command));
}

void spinWheels(int power0, int power1, int power2)
{
	spinWheel(0, -power0);
	spinWheel(1, power1);
	spinWheel(2, power2);
}

void killMotors()
{
	spinWheel(0, 0);
	spinWheel(1, 0);
	spinWheel(2, 0);
	currentBodyVelocity = (coord3){0, 0, 0};
}

void driveMotorWithSignedSpeed(int wheelId, long qSpeed)
{
	char command[7];
	command[0] = wheelAddress[wheelId];
	command[1] = 35 + wheelRoboclawPosition[wheelId];
	longToArray(command, 2, qSpeed);
	command[6] = calcChecksum(command, 6);
	serial_sendMessage(command, 7);
}

void printMotorSpeeds()
{
	printf("Motor speeds: %ld, %ld, %ld\n", readMotorSpeed(0), readMotorSpeed(1), readMotorSpeed(2));
}

void printPidConstants()
{
	for(int w = 0; w < 3; w++)
	{
		pidq result = readMotorPidConstants(w);
		printf("Motor %d: P: %ld I: %ld D: %ld QPPS: %ld\n", w, result.p, result.i, result.d, result.q);
	}
}

void showSpeedVsVelocityGraph(int wheelId)
{
	printf("Show power vs. velocity graph for wheel %d\n", wheelId + 1);
	printf("Desired\tActual\n");
	for(int i = 0; i < 127; i++)
	{
//		long speed = 10 * i;
//		driveMotorWithSignedSpeed(wheelId, speed);
		int speed = i;
		spinWheel(wheelId, speed);
		sleep_ms(100);
		long actual = readMotorSpeed(wheelId);
		printf("%d\t%ld\n", speed, actual);
	}
	spinWheel(wheelId, 0);
}

//Two things to calibrate

void startCalibrate(coord2 velocity)
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
float gme(float M[3][3], int i, int j, int r, int c)
{
    return M[(i + r + 1) % 3][(j + c + 1) % 3];
}

float getSignedMinorDet(float M[3][3], int i, int j)
{
    return gme(M, i, j, 0, 0) * gme(M, i, j, 1, 1) - gme(M, i, j, 1, 0) * gme(M, i, j, 0, 1);
}

float getDet(float M[3][3])
{
    return M[0][0] * getSignedMinorDet(M, 0, 0) +
        M[0][1] * getSignedMinorDet(M, 0, 1) +
        M[0][2] * getSignedMinorDet(M, 0, 2);
}

void invertMatrix(float M[3][3], float dest[3][3])
{
    float d = getDet(M);
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
        	dest[j][i] = getSignedMinorDet(M, i, j) / d;
}

coord2 WHEEL_BASE[] = {(coord2){-4.120, 1.895}, (coord2){4.249, 1.843}, (coord2){-0.142, -3.814}};
coord2 WHEEL_DIR[] = {(coord2){-0.540, -0.842}, (coord2){-0.376, 0.926}, (coord2){0.997, 0.080}};

float getBodyFrameVectorSpinFactor(coord2 wheelBase, coord2 wheelDir)
{
    return utility_dotProduct(
    		utility_unitVector(utility_rotate(wheelBase, M_PI / 2)), wheelDir)
    		* utility_dist1(wheelBase);
}

void calcBodyFrameVectors()
{
	M[0][0] = WHEEL_DIR[0].x / P_WHEEL_RADIUS;
	M[0][1] = WHEEL_DIR[0].y / P_WHEEL_RADIUS;
	M[0][2] = getBodyFrameVectorSpinFactor(WHEEL_BASE[0], WHEEL_DIR[0]) / P_WHEEL_RADIUS;
	M[1][0] = WHEEL_DIR[1].x / P_WHEEL_RADIUS;
	M[1][1] = WHEEL_DIR[1].y / P_WHEEL_RADIUS;
	M[1][2] = getBodyFrameVectorSpinFactor(WHEEL_BASE[1], WHEEL_DIR[1]) / P_WHEEL_RADIUS;
	M[2][0] = WHEEL_DIR[2].x / P_WHEEL_RADIUS;
	M[2][1] = WHEEL_DIR[2].y / P_WHEEL_RADIUS;
	M[2][2] = getBodyFrameVectorSpinFactor(WHEEL_BASE[2], WHEEL_DIR[2]) / P_WHEEL_RADIUS;
	invertMatrix(M, M_inverse);
	printMatrix("M Matrix", M);
	printMatrix("Inverted matrix", M_inverse);
}


//void calcBodyFrameVectors()
//{
//	M[0][0] = cos(M_PI/180*WHEEL_ANGLES[0])  / P_WHEEL_RADIUS;
//	M[0][1] = -sin(M_PI/180*WHEEL_ANGLES[0]) / P_WHEEL_RADIUS;
//	M[0][2] = P_ROBOT_RADIUS				 / P_WHEEL_RADIUS;
//	M[1][0] = cos(M_PI/180*WHEEL_ANGLES[1])  / P_WHEEL_RADIUS;
//	M[1][1] = -sin(M_PI/180*WHEEL_ANGLES[1]) / P_WHEEL_RADIUS;
//	M[1][2] = P_ROBOT_RADIUS				 / P_WHEEL_RADIUS;
//	M[2][0] = cos(M_PI/180*WHEEL_ANGLES[2])  / P_WHEEL_RADIUS;
//	M[2][1] = -sin(M_PI/180*WHEEL_ANGLES[2]) / P_WHEEL_RADIUS;
//	M[2][2] = P_ROBOT_RADIUS				 / P_WHEEL_RADIUS;
//	invertMatrix(M, M_inverse);
//	printMatrix("M Matrix", M);
//	printMatrix("Inverted matrix", M_inverse);
//}

void motorControl_init()
{
	calcBodyFrameVectors();
	serial_fd = open("/dev/ttySAC0", O_RDWR);
}

float motor1pulsesPerSecond;
float motor2pulsesPerSecond;
float motor3pulsesPerSecond;

void driveMotorsAtSpeed(float motor1Speed, float motor2Speed, float motor3Speed)
{
	motor1pulsesPerSecond = motor1Speed * pulsesPerRadian;
	motor2pulsesPerSecond = motor2Speed * pulsesPerRadian;
	motor3pulsesPerSecond = motor3Speed * pulsesPerRadian;
	driveMotorWithSignedSpeed(0, motor1pulsesPerSecond);
	driveMotorWithSignedSpeed(1, motor2pulsesPerSecond);
	driveMotorWithSignedSpeed(2, motor3pulsesPerSecond);
}

coord3 matrix_mult(float M[3][3], coord3 v)
{
	return (coord3){
		M[0][0]*v.x + M[0][1]*v.y + M[0][2]*v.w,
		M[1][0]*v.x + M[1][1]*v.y + M[1][2]*v.w,
		M[2][0]*v.x + M[2][1]*v.y + M[2][2]*v.w};
}

coord3 translateBodyCoordinatesToMotorVelocities(coord3 v)
{
	return matrix_mult(M, v);
}

coord3 translateMotorVelocitiesToBodyCoordinates(coord3 motorSpeeds)
{
	return matrix_mult(M_inverse, motorSpeeds);
}

coord3 translateWorldCoordinatesToBodyCoordinates(coord3 robot, coord3 v)
{
	rotate(&v, -robot.w + M_PI/2);
	return v;
}

coord3 translateBodyCoordinatesToWorldCoordinates(coord3 robot, coord3 v)
{
	rotate(&v, robot.w - M_PI/2);
	return v;
}

void moveRobotBodyCoordinates(coord3 v) //int[]* worldVelocities, int[]* wheelVelocities)
{
	currentBodyVelocity = v;
	coord3 motorSpeeds = translateBodyCoordinatesToMotorVelocities(v);
//	printCoord3("Body velocities", v);
//	printCoord3("Motor speeds", motorSpeeds);
//	printCoord3("Reverse calc", translateMotorVelocitiesToBodyCoordinates(motorSpeeds));
	driveMotorsAtSpeed(motorSpeeds.x, motorSpeeds.y, motorSpeeds.w);
}

void moveRobotWorldCoordinates(coord3 robot, coord3 v)
{
	if(!override)
	{
		if(isnan(v.x))
			v.x = 0;
		if(isnan(v.y))
			v.y = 0;
		if(isnan(v.w))
			v.w = 0;
		v = translateWorldCoordinatesToBodyCoordinates(robot, v);
		moveRobotBodyCoordinates(v);
	}
}



//Tick and control

void setOverride(bool val)
{
	override = val;
}

void overrideForSpecifiedTime(double t)
{
	//printf("Start timer\n");
	startTimer(MOTOR_CONTROL_OVERRIDE_TIMER);
	timeToOverride = t;
	setOverride(true);
}

std::deque<coord3> pastVelocities;

coord3 lastMotorPositions;

void updateCurrentPosition(bool print)
{
	coord3 currentMotorPositions = readQuadratureEncoders();
	coord3 motorPulses = utility_subVector3(currentMotorPositions, lastMotorPositions);
	coord3 motorRadians = utility_divVector3(motorPulses, pulsesPerRadian);
	lastMotorPositions = currentMotorPositions;
	coord3 bodyCoords = translateMotorVelocitiesToBodyCoordinates(motorRadians);
	coord3 worldCoords = translateBodyCoordinatesToWorldCoordinates(robot1currentPosition, bodyCoords);
	robot1currentPosition.x = robot1currentPosition.x + worldCoords.x;
	robot1currentPosition.y = robot1currentPosition.y + worldCoords.y;
	robot1currentPosition.w = robot1currentPosition.w + worldCoords.w;
	if(DEBUG_PRINT)
	{
//		printCoord3("Motors", motorRadians);
//		printCoord3("Body  ", bodyCoords);
//		printCoord3("-------- Position", robot1currentPosition);
	}



////	startTimer(9);
//	robot1currentPosition = robot1cameraPosition;
//	int ticksBehind = (int)(cameraLatency_ms / 1000 * TICKS_PER_SECOND + .5);
////	if(DEBUG_PRINT)
////		printf("Print out queue\n");
//	for(int i = 100 - ticksBehind; i < 100 && i < pastVelocities.size(); i++)
//	{
//		coord3 bodyVelocity = pastVelocities.at(i);
////		if(DEBUG_PRINT)
////		{
////			printf("  Current position: %f %f %f\n", robot1currentPosition.x, robot1currentPosition.y, robot1currentPosition.w);
////			printf("  Body velocity: %f %f %f\n", bodyVelocity.x, bodyVelocity.y, bodyVelocity.w);
////		}
//		coord3 worldVelocity = translateBodyCoordinatesToWorldCoordinates(robot1currentPosition, bodyVelocity);
////		if(DEBUG_PRINT)
////			printf("  worldVelocity: %f %f %f\n", worldVelocity.x, worldVelocity.y, worldVelocity.w);
//		robot1currentPosition.x = robot1currentPosition.x + worldVelocity.x / TICKS_PER_SECOND;
//		robot1currentPosition.y = robot1currentPosition.y + worldVelocity.y / TICKS_PER_SECOND;
//		robot1currentPosition.w = robot1currentPosition.w + worldVelocity.w / TICKS_PER_SECOND;
//	}
//	//printf("Time to excecute: %f\n", getTimerTime_ms(9)); //100 us
}

void motorControl_tick()
{
	if(override && getTimerTime_ms(MOTOR_CONTROL_OVERRIDE_TIMER) > timeToOverride)
	{
		override = false;
		killMotors();
		//printf("stop override at %f ms\n", getTimerTime_ms(MOTOR_CONTROL_OVERRIDE_TIMER));
	}

	//coord3 actualBodyVelocity = getBodyVelocity();

	pastVelocities.push_back((coord3)currentBodyVelocity);
	if(pastVelocities.size() > 100)
		pastVelocities.pop_front();

	updateCurrentPosition(false);
}

coord3 lastMotorPositionsTest;
void printMotorDiffs()
{
	coord3 currentMotorPositions = readQuadratureEncoders();
	coord3 motorPulses = utility_subVector3(currentMotorPositions, lastMotorPositionsTest);
	coord3 motorPulsesPerRadian = utility_divVector3(motorPulses, 10 * 2 * M_PI);
	lastMotorPositionsTest = currentMotorPositions;
	printCoord3("Pulses Per Radian (with 10 rotations)", motorPulsesPerRadian);
}
