#include "VisionArmCombo.h"

#include <iostream>
#include <ctime>
#include <Windows.h>
#include "utilities.h"


int main(int argc, char**argv)
{
	VisionArmCombo vac;

	//vac.waitForChamberTimeOffset(1);

	//std::getchar();

	//std::cout << "send rover\n";	std::getchar();
	//for (int i = vac.cur_chamber_id_; i <= 8; i++)
		//vac.sendRoverToChamber(i);
	
	vac.sendRoverToChamber(vac.cur_chamber_id_);
	
	//vac.manualMapPotPositions(0, 0);

	//vac.hyperspectralCameraCalibration(); std::getchar(); exit(0);

	//vac.calibrateToolCenterPoint(4, PAM);

	//vac.calibrateThermalCamera(); vac.ThermalHandEyeCalibration();
	//vac.calibrateRGBCamera(30); vac.RGBHandEyeCalibration();
	//vac.calibrateIRCamera(30); vac.IRHandEyeCalibration();

	//vac.acquireLinesOnPlanes();

	//vac.readCloudAndPoseFromFile(); vac.lineScannerHandEyeCalibration(6); std::getchar();  return 0;

}