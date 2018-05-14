#include "VisionArmCombo.h"
#include "HyperspectralCamera.h"
#include <iostream>
#include <ctime>
#include <Windows.h>


int main(int argc, char**argv)
{
	VisionArmCombo vac;

#if 0
	while (true) {

		UINT64 safety_mode = vac.robot_arm_client_->getSafetyMode();

		std::cout << std::hex<<safety_mode << "\n";

		Sleep(1000);

	}
#endif

	//vac.acquireHyperspectralCalibrationData();
	
#if 0
	float dist;
	int marker_id;
	vac.markerDetection(RGB, dist, marker_id, 10.f, true);	
	std::cout << "dist: " << dist << " marker id" << marker_id<<"\n";
	std::getchar();
	return 0;
#endif

	// go to chamber test
	//while (1) {		vac.sendRoverToChamber(1);}
#if 1

	std::cout << "any key to go to chamber 1\n";
	std::getchar();

	for (int i = 0; i < 100; i++) {
		std::cout << "round " << i + 1 << "\n";
		if (vac.sendRoverToChamber(1) != SUCCESS) {
			std::cout << "error\n";
			break;
		}
	}

	std::cout << "end\n";
	std::getchar();
#endif


#if 0
	double array6[6] = {0.015, -0.538, 0.02, M_PI, 0., 0.};

	vac.robot_arm_client_->moveHandL(array6, 0.1, 0.1);

	double array3[3] = {-0.3, 0., 0.};
	vac.scanTranslateOnlyHyperspectral(array3, 0.1, 0.1);

	std::getchar();
#endif

	//vac.testRun();	std::getchar(); return 0;
	
	//vac.markerDetection();

	//vac.calibrateToolCenterPoint(4, PAM);

	//vac.calibrateThermalCamera(); vac.ThermalHandEyeCalibration();
	//vac.calibrateKinectRGBCamera(); vac.KinectRGBHandEyeCalibration();
	//vac.calibrateKinectIRCamera(); vac.KinectIRHandEyeCalibration();

	//vac.acquireLinesOnPlanes();

	//vac.readCloudAndPoseFromFile(); vac.lineScannerHandEyeCalibration(6); std::getchar();  return 0;

	// Enter chamber and stop at middle position
#if 0
	int num_plants = 7;
	vac.pot_process_status_.clear();
	vac.pot_process_status_.resize(num_plants);
	for (auto & s : vac.pot_process_status_) s = false;

	vac.plant_cluster_min_vec_.clear();
	vac.plant_cluster_min_vec_.resize(num_plants);
	vac.plant_cluster_max_vec_.clear();
	vac.plant_cluster_max_vec_.resize(num_plants);

	vac.mapWorkspaceUsingKinectArm(1, num_plants);

	std::cout << "move rover to 0 and hit enter\n";
	std::getchar();

	vac.mapWorkspaceUsingKinectArm(0, num_plants);

	std::cout << "Done\n";
#endif

	//vac.extractLeafFromPointCloud();

}