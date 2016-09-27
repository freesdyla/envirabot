#define _CRT_SECURE_NO_WARNINGS
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include "RobotArmClient.h" // need to be 1st due to winsock
#include "KeyenceLineProfiler.h"
#include "VisionArmCombo.h"
#include <ctime>
#include "KinectThread.h"
#include "PathPlanner.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


int main(int argc, char**argv)
{
	VisionArmCombo vac;

	vac.mapWorkspaceUsingKinectArm();

	//vac.extractLeafFromPointCloud();

	//vac.acquireLinesOnPlanes();	std::getchar(); return 0;

	//vac.readCloudAndPoseFromFile(); vac.lineScannerHandEyeCalibration(6);

	//vac.testLineScannerProbeCalibrationMatrix();

	//Eigen::Vector3d v; vac.calibrateToolCenterPoint(v, 6);

	std::getchar();  return 0;

	return 0;
}