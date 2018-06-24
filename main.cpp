#include "VisionArmCombo.h"

#include <iostream>
#include <ctime>
#include <Windows.h>
#include "utilities.h"


int main(int argc, char**argv)
{
	VisionArmCombo vac;

#if 0
	TOF_Swift camh;
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_(new pcl::visualization::PCLVisualizer);
	viewer_->addCoordinateSystem(0.1);
	VisionArmCombo::PointCloudT::Ptr cloud_(new VisionArmCombo::PointCloudT);

	camh.setPower(300);

	while (true) {

		camh.getPointCloud(cloud_);

		if (cloud_->size() > 10) {
			viewer_->removeAllPointClouds();
			viewer_->addPointCloud(cloud_, "cloud");
			viewer_->spinOnce(10);
		}

		cv::Mat ir = camh.getIR();
		cv::imshow("IR", ir); cv::waitKey(10);
	}

	std::getchar();

	camh.stop();
#endif
	
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

	//std::cout << "send rover\n";	std::getchar();
	vac.sendRoverToChamber(vac.cur_chamber_id_); std::getchar();

	// go to chamber test
	//while (1) {		vac.sendRoverToChamber(vac.cur_chamber_id);}
#if 0

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
	
	//vac.markerDetection();

	//vac.calibrateToolCenterPoint(4, PAM);

	//vac.calibrateThermalCamera(); vac.ThermalHandEyeCalibration();
	//vac.calibrateRGBCamera(30); vac.RGBHandEyeCalibration();
	//vac.calibrateIRCamera(30); vac.IRHandEyeCalibration();

	//vac.acquireLinesOnPlanes();

	//vac.readCloudAndPoseFromFile(); vac.lineScannerHandEyeCalibration(6); std::getchar();  return 0;

}