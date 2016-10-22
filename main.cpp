#define _CRT_SECURE_NO_WARNINGS

#include "VisionArmCombo.h"

#include <iostream>
#include "RobotArmClient.h" // need to be 1st due to winsock
#include "KeyenceLineProfiler.h"

#include <ctime>
#include "KinectThread.h"
#include "PathPlanner.h"



int main(int argc, char**argv)
{
	VisionArmCombo vac;

	vac.markerDetection();

	//vac.calibrateKinectRGBCamera();

	//vac.KinectRGBHandEyeCalibration();

	//vac.mapWorkspaceUsingKinectArm();

	//vac.extractLeafFromPointCloud();

	//vac.acquireLinesOnPlanes();	std::getchar(); return 0;

	//vac.readCloudAndPoseFromFile(); vac.lineScannerHandEyeCalibration(6);

	//vac.testLineScannerProbeCalibrationMatrix();

	//Eigen::Vector3d v; vac.calibrateToolCenterPoint(v, 6);

	std::getchar();  return 0;
}