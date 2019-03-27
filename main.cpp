#include "VisionArmCombo.h"

#include <iostream>
#include <ctime>
#include <Windows.h>
#include "utilities.h"
#include "RoboteqDevice.h"

int main(int argc, char**argv)
{
	VisionArmCombo vac;

	vac.run();

	//vac.sendRoverToChamber(vac.cur_chamber_id_);
	//vac.sideViewImagingRoutinePerPlant(1, -0.3, 1);
	//vac.sideViewProbingRoutinePerPlant(1, -0.3, 1);

#if 0
	for (double y = -0.1; y <= 0.1; y += 0.05)
	{
		std::cout << "Y " << y << "\n";
		Eigen::Matrix4d pose;
		pose.col(0) << 0., 1., 0., 0.;
		pose.col(1) << 0., 0., 1., 0.;
		pose.col(2) << 1., 0., 0., 0.;
		pose.col(3) << 0.8, y, 0.5, 1.;
		pose.topLeftCorner<3, 3>() = pose.topLeftCorner<3, 3>()*Eigen::AngleAxisd(pcl::deg2rad(70.), Eigen::Vector3d::UnitX()).matrix()*Eigen::AngleAxisd(pcl::deg2rad(20.), Eigen::Vector3d::UnitZ()).matrix();
		std::vector<int> ik_sol_vec;
		vac.pp_.inverseKinematics(pose, ik_sol_vec, 1);
		//for (int i = 0; i < 8; i++)
		for(auto i:ik_sol_vec)
		{
			std::cout << "solution: " << i << std::endl;
			float sol_f[6];

			vac.double2float(vac.pp_.ik_sols_ + i * 6, sol_f);

			VisionArmCombo::ArmConfig config;
			config.setJointPos(sol_f[0], sol_f[1], sol_f[2], sol_f[3], sol_f[4], sol_f[5]);

			std::vector<PathPlanner::RefPoint> ref_points;
			std::vector<Eigen::Matrix3f> rot_mats;
			std::vector<PathPlanner::OBB> arm_obbs;
			vac.pp_.computeReferencePointsOnArm(config.joint_pos_f, ref_points, rot_mats);
			vac.pp_.getArmOBBModel(ref_points, rot_mats, arm_obbs); vac.addOBBArmModelToViewer(arm_obbs);
			vac.display();
		}
	}
#endif

	//RoboteqDevice bms;
	//bms.Connect();

	//int status;
	//string response;
	//status = bms.GetBatteryStatus(response);
	//Utilities::to_log_file("upload done");
	//std::cout << "done";std::getchar();

	//vac.manualMapPotPositions(0, 0);

	//vac.hyperspectralCameraCalibration(); std::getchar(); exit(0);

	//vac.calibrateToolCenterPoint(4, PAM);
	
	//vac.calibrateThermalCamera(); vac.ThermalHandEyeCalibration();
	//vac.calibrateRGBCamera(30); vac.RGBHandEyeCalibration();
	//vac.calibrateIRCamera(30); vac.IRHandEyeCalibration();

	//vac.acquireLinesOnPlanes();

	//vac.readCloudAndPoseFromFile(); vac.lineScannerHandEyeCalibration(6); std::getchar();  return 0;

}