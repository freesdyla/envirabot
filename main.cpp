#include "VisionArmCombo.h"
#include "HyperspectralCamera.h"
#include <iostream>
#include <ctime>
#include <Windows.h>


int main(int argc, char**argv)
{
	VisionArmCombo vac;


#if 0
	double array6[6] = {0.015, -0.538, 0.02, M_PI, 0., 0.};

	vac.robot_arm_client_->moveHandL(array6, 0.1, 0.1);

	double array3[3] = {-0.3, 0., 0.};
	vac.scanTranslateOnlyHyperspectral(array3, 0.1, 0.1);

	std::getchar();
#endif

	//vac.testRun();	std::getchar(); return 0;

	//vac.acquireRGBStereoPair(); std::getchar(); return 0;
	
	//vac.setAndSaveParameters(); std::getchar();	return 0;
	
	//vac.calibrateToolCenterPoint(4, PAM); std::getchar(); //return 0;

	//vac.TCPCalibrationErrorAnalysis(8); std::getchar(); return 0;

//	vac.acquireLinesOnPlanes();	//return 0;
	
//	vac.readCloudAndPoseFromFile(); vac.lineScannerHandEyeCalibration(6); std::getchar();  return 0;

	//vac.calibrateGripperTip(8);

	//vac.testLineScannerProbeCalibrationMatrix(); std::getchar(); return 0;

	//vac.scanAndProbeTest();	std::getchar(); return 0;

//	vac.probePlateCenterTest();	std::getchar(); return 0;

	//vac.scanPotMultiAngle(); std::getchar(); return 0;


	//vac.markerDetection();

	//vac.calibrateKinectRGBCamera(); vac.KinectRGBHandEyeCalibration();
	vac.calibrateKinectIRCamera(); vac.KinectIRHandEyeCalibration();

	



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

	

	//vac.testLineScannerProbeCalibrationMatrix();

}