#ifndef ROBOT_ARM_CLIENT_H
#define ROBOT_ARM_CLIENT_H

#define WIN32_LEAN_AND_MEAN
#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES // for C++

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <iostream>
#include <time.h>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>
#include <cmath>


// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")


#define DEFAULT_BUFLEN 2048
//#define DEFAULT_PORT "30002"	//10HZ
#define DEFAULT_PORT "30003" // real time client 125HZ

struct RobotArmClient
{
	WSADATA wsaData;
	SOCKET ConnectSocket = INVALID_SOCKET;
	struct addrinfo *result;
	struct addrinfo *ptr;
	struct addrinfo	hints;
	char recvbuf[DEFAULT_BUFLEN];
	int iResult;
	int recvbuflen = DEFAULT_BUFLEN;

	//robot hand cartesion info
	double cur_cartesian_info_array[6];

	double dst_cartesian_info_array[6];

	double cur_joint_pos_array[6];

	double dst_joint_pos_array[6];

	double cur_tcp_speed_array[6];

	double start_xyz_[3];

	std::atomic<float> displacement_;

	std::atomic<double> distanceToDst_;

	std::atomic<float> tcp_speed_;

	double distanceToDstConfig_;

	std::vector<std::thread> URMsgHandler;

	std::mutex updateMutex;

	bool TCP_ALIVE;

	RobotArmClient();

	void startRecvTCP();
	
	//void closeTCP();

	void getCartesianInfo(double* array6);

	void getTCPSpeed(double* array6);

	void getCurJointPose(double* array6);

	void reverse8CharsToDouble(char* end, double* d);
	
	void getCartesionInfoFromURPackage(char* x_start_ptr);

	void getActualJointPosFromURPackage();

	void getActualTCPSpeedFromURPackage();

	int moveHandL(double* dst_cartesian_info, float acceleration, float speed);

	int moveHandJ(double* dst_joint_config, float speed, float acceleration, bool wait2dst);

	void printCartesianInfo(double* array6);

	int waitTillHandReachDstPose(double* dst_pose6);

	int waitTillHandReachDstConfig(double* dst_joint_config);

	double EuclideanDistance(double* pose6_1, double* pose6_2);

	void setDstCartesianInfo(double* array6);

	void setDstConfigInfo(double* array6);

	void getDistanceToDst(double& distance);

	void stopRecvTCP(void);

	void setStartPoseXYZ();

	void waitTillTCPMove();

	void rotateJointRelative(int id, double deg, float acceleration, float speed);

	void moveHandRelativeTranslate(double x, double y, double z, float acceleration, float speed);
	
};

#endif