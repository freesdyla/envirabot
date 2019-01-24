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
#include <iomanip>
#include <limits>
#include "utilities.h"

// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")


#define DEFAULT_BUFLEN 1 << 16
//#define DEFAULT_PORT "30002"	//10HZ
#define DEFAULT_PORT "30003" // real time client 125HZ

struct RobotArmClient
{
	WSADATA wsaData;
	SOCKET ConnectSocket = INVALID_SOCKET;
	struct addrinfo *result;
	struct addrinfo *ptr;
	struct addrinfo	hints;
	char *recvbuf;
	int iResult;
	const int recvbuflen = DEFAULT_BUFLEN;

	//robot hand cartesion info
	double cur_cartesian_info_array[6];

	double dst_cartesian_info_array[6];

	double cur_joint_pos_array[6];

	double dst_joint_pos_array[6];

	double cur_tcp_speed_array[6];

	double start_xyz_[3];

	double cur_ur_time_;

	std::vector<double> ur_timestamp_vec_;

	std::atomic<float> displacement_;

	std::atomic<double> distanceToDst_;

	std::atomic<UINT64> safety_mode_;

	std::atomic<double> tcp_speed_;

	std::atomic<double> rot_speed_;

	std::atomic<double> distanceToDstConfig_;

	std::vector<std::thread> URMsgHandler;

	std::mutex updateMutex;

	bool TCP_ALIVE;

	std::atomic<bool> recv_normal_ = false;

	struct timestamp_pose
	{
		LARGE_INTEGER timestamp;
		double pose[6];
	};

	std::vector<timestamp_pose> timestamp_pose_vec_;

	std::atomic<bool> record_poses_ = false;


	RobotArmClient();

	~RobotArmClient();

	void startRecvTCP();
	
	void getCartesianInfo(double* array6);

	void getTCPSpeed(double* array6);

	void getCurJointPose(double* array6);

	UINT64 getSafetyMode();

	void reverse8CharsToDouble(char* end, double* d);
	
	void getCartesionInfoFromURPackage();

	void getActualJointPosFromURPackage();

	void getActualTCPSpeedFromURPackage();

	void getTimeFromURPackage();

	int moveHandL(double* dst_cartesian_info, float acceleration, float speed, bool wait2dst = true);

	int moveHandJ(double* dst_joint_config, float speed, float acceleration, bool wait2dst = true);

	void printCartesianInfo(double* array6);

	int waitTillHandReachDstPose(double* dst_pose6);

	double waitTillRotationSpeedDropDownTo(double target_speed);

	int waitTillHandReachDstConfig(double* dst_joint_config);

	double EuclideanDistance(double* pose6_1, double* pose6_2);

	void setDstCartesianInfo(double* array6);

	void setDstConfigInfo(double* array6);

	void getDistanceToDst(double& distance);

	void stopRecvTCP(void);

	void setStartPoseXYZ();

	double waitTillTCPMove(double target_speed = 0.01);

	double waitTillRotate(double target_speed = 0.01);

	void rotateJointRelative(int id, double deg, float acceleration, float speed);

	void moveHandRelativeTranslate(double x, double y, double z, float acceleration, float speed);

	double configLInfNorm(double* config1, double* config2);

	void lineLightControl(bool turn_on);

	void probeCylinderControl(bool extend);

	void whiteBoardServoControl(bool extend);

	void laserScannerLightControl(bool on);

	void startOrStopRecordPose(bool start);

	std::vector<timestamp_pose> getTimestampPoses();

	int saveTimeStampPoses(std::string filename);

	int saveCurPose(std::string filename);
};

#endif