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

void wheelTest();
void spinWheel(int wheelId, int velocity);
void serial_sendMessage(char command[], int size);
void straight();
void turnRight();
void turnLeft();
void moveRight();
void moveLeft();
void killMotors();
void SpinInPlaceC();
void SpinInPlaceCC();
void Box();

void printCharArray(char command[], int size)
{
	for(int i = 0; i < size; i++)
	{
		cout << " " << ((int)((unsigned char)command[i]));
	}
	cout << endl;
}

void wheelTest()
{
	int i;
	for(i = 0; i < 3; i++)
	{
		spinWheel(i, 127);
		sleep(1);
		spinWheel(i, -127);
		sleep(1);
		spinWheel(i, 0);
		sleep(1);
	}
}

#ifndef M_PI
#define M_PI 3.1415926535
#endif

// Everything is in centimeters.
#define P_WHEEL_RADIUS 2.7 //Measured
#define P_ROBOT_RADIUS 8.4 //Measured
#define P_KICKER_DIST 6.35 //Measured
const float WHEEL_ANGLES[] = {120, 240, 0};
float M[3][3]; // = {{1, 2, 3}, {0, 0, 0}, {0, 0, 0}};
float wheelPowerNeededPerVelocityUnit = 20;

void calcBodyFrameVectors();
void calculateWheelVelocities(float v[]);
void spinWheelAtVelocity(int wheelId, float velocity);
int sign(int x);
int serial_fd;

void motorControl_init()
{
	calcBodyFrameVectors();
	serial_fd = open("/dev/ttySAC0", O_RDWR);
}

void calcBodyFrameVectors()
{
	M[0][0] = cos(M_PI/180*WHEEL_ANGLES[0]) / P_WHEEL_RADIUS;
	M[0][1] = sin(M_PI/180*WHEEL_ANGLES[0]) / P_WHEEL_RADIUS;
	M[0][2] = P_ROBOT_RADIUS								/ P_WHEEL_RADIUS;
	M[1][0] = cos(M_PI/180*WHEEL_ANGLES[1]) / P_WHEEL_RADIUS;
	M[1][1] = sin(M_PI/180*WHEEL_ANGLES[1]) / P_WHEEL_RADIUS;
	M[1][2] = P_ROBOT_RADIUS								/ P_WHEEL_RADIUS;
	M[2][0] = cos(M_PI/180*WHEEL_ANGLES[2]) / P_WHEEL_RADIUS;
	M[2][1] = sin(M_PI/180*WHEEL_ANGLES[2]) / P_WHEEL_RADIUS;
	M[2][2] = P_ROBOT_RADIUS								/ P_WHEEL_RADIUS;
}

void moveRobotBodyCoordinates(coord3 v) //int[]* worldVelocities, int[]* wheelVelocities)
{
	spinWheelAtVelocity(0, M[0][0]*v.x - M[0][1]*v.y + M[0][2]*v.w);
	spinWheelAtVelocity(1, M[1][0]*v.x - M[1][1]*v.y + M[1][2]*v.w);
	spinWheelAtVelocity(2, M[2][0]*v.x - M[2][1]*v.y + M[2][2]*v.w);
}

void moveRobotWorldCoordinates(coord3 robot, coord3 v)
{
	//printCoord3(v);
	rotate(&v, -robot.w + M_PI/2);
	moveRobotBodyCoordinates(v);
}

// Spin wheel in cm/sec.
void spinWheelAtVelocity(int wheelId, float velocity)
{
	int power = (int)(velocity * wheelPowerNeededPerVelocityUnit);
	spinWheel(wheelId, power);
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

//Valid baud rates for ROBOClAW are 2400, 9600, 19200, and 38400
//void serial_setBaudRate(int baudRate)
//{
//	system("stty -F /dev/ttySAC0 9600"); // + baudRate);
//	printf("stty -F /dev/ttySAC0 %d\n", baudRate);
//}

void serial_sendMessage(char command[], int size)
{
	//startTimer();
	write(serial_fd, command, size);
	//cout << "command sent: ";
	//printCharArray(command, size);
	//printf("Time to send command %ld us\n", getTimerTime_us()); //Aprox 300 us
}

int serial_readMessage(char result[], int size)
{
	//For some reason we have to reopen connection to receive data
//	close(serial_fd);
//	serial_fd = open("/dev/ttySAC0", O_RDWR);

	//startTimer();
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

	//result[len] = '\0';
	//printf("Time to send command %ld us\n", getTimerTime_us()); //Aprox 300 us

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
	printf("Show power vs. velocity graph for wheel %d\n", wheelId);
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




//Other stuff

void straight()
{
	spinWheel(0,127);
	spinWheel(1,127);
}

void turnRight(){
	spinWheel(0,-15);
	spinWheel(1,30);
	spinWheel(2,30);
	sleep(1);
	killMotors();
}

void turnLeft(){
	spinWheel(0,43);
	//spinWheel(1,-30);
	//spinWheel(2,-30);
	sleep(1);
	killMotors();
}

void moveRight(){
	spinWheel(2,-127);
	spinWheel(1,25);
	spinWheel(0,-25);

}

void moveLeft(){
	spinWheel(2,127);
	spinWheel(1,-25);
	spinWheel(0,25);
}

void killMotors()
{
	spinWheel(0,0);
	spinWheel(1,0);
	spinWheel(2,0);
}

void SpinInPlaceC()
{
	spinWheel(0,-20);
	spinWheel(1,20);
	spinWheel(2,20);
	sleep(3);
	killMotors();
}

void SpinInPlaceCC()
{
	spinWheel(0,-80);
	spinWheel(1,80);
	spinWheel(2,80);
	sleep(3);
	killMotors();
}

void Box(){
	int i =0;
	for(i; i<4; i++)
	{
		straight();
		sleep(1);
		killMotors();
		turnLeft();
	}
}
