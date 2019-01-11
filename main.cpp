#include "VisionArmCombo.h"

#include <iostream>
#include <ctime>
#include <Windows.h>
#include "utilities.h"


int main(int argc, char**argv)
{
	VisionArmCombo vac;

	//vac.enterChargingMode();

	//for (int i = vac.cur_chamber_id_; i <= 8; i++) vac.sendRoverToChamber(i);
	
	vac.sendRoverToChamber(vac.cur_chamber_id_);

	//std::getchar();

#if 0
	Eigen::Matrix4d cam_pose;
	cam_pose.col(0) << 0, -1, 0, 0;
	cam_pose.col(1) << -1, 0, 0, 0;
	cam_pose.col(2) << 0, 0, -1, 0;
	cam_pose.col(3) << -0.1, -0.6, 0.3, 1.;

	cv::Vec6d start_scan_pose;
	vac.tiltLinearScan(cam_pose, start_scan_pose, 40, DIRECT_IMAGING);
	std::string file_path = "hs_1_" + vac.getCurrentDateTimeStr();
	vac.hypercam_->saveData(file_path);
	vac.robot_arm_client_->saveTimeStampPoses(file_path);
#endif
	
	//vac.manualMapPotPositions(0, 0);

	//vac.hyperspectralCameraCalibration(); std::getchar(); exit(0);

	//vac.calibrateToolCenterPoint(4, PAM);
	
	//vac.calibrateThermalCamera(); vac.ThermalHandEyeCalibration();
	//vac.calibrateRGBCamera(30); vac.RGBHandEyeCalibration();
	//vac.calibrateIRCamera(30); vac.IRHandEyeCalibration();

	//vac.acquireLinesOnPlanes();

	//vac.readCloudAndPoseFromFile(); vac.lineScannerHandEyeCalibration(6); std::getchar();  return 0;

}