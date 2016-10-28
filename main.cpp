#include "VisionArmCombo.h"
#include <iostream>
#include <ctime>

int main(int argc, char**argv)
{
	VisionArmCombo vac;
	
	//vac.calibrateToolCenterPoint(8); std::getchar(); return 0;

	//vac.testLineScannerProbeCalibrationMatrix(); std::getchar(); return 0;

	vac.scanAndProbeTest();	std::getchar(); return 0;

	/*int i = 1;
	while(true)
	{
		std::getchar();
		vac.sendRoboteqVar(1, i);
		i++;
		if (i > 3)
		{
			i = 1;
		}
	}*/

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