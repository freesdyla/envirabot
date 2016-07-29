#define _CRT_SECURE_NO_WARNINGS
#include <pcl\visualization\pcl_visualizer.h>
#include <pcl\point_cloud.h>
#include <pcl\point_types.h>
#include <pcl\kdtree\kdtree_flann.h>
#include <pcl\features\normal_3d.h>
#include <pcl\surface\gp3.h>
#include <pcl\filters\voxel_grid.h>
#include <pcl\filters\statistical_outlier_removal.h>
#include <iostream>
#include "RobotArmClient.h" // need to be 1st due to winsock
#include "KeyenceLineProfiler.h"
#include "VisionArmCombo.h"
#include <ctime>
#include "KinectThread.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

double magnitudeVec3(double * vec)
{
	return sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
}

void array6ToEigenMat4(double* array6, Eigen::Matrix4f & mat4)
{
	mat4 = Eigen::Matrix4f::Identity();
	double angle = magnitudeVec3(array6 + 3);
	Eigen::Vector3f axis(array6[3] / angle, array6[4] / angle, array6[5] / angle);
	Eigen::Matrix3f rotationMatrix;
	rotationMatrix = Eigen::AngleAxisf(angle, axis);
	mat4.block<3, 3>(0, 0) = rotationMatrix;
	mat4(0, 3) = array6[0]; mat4(1, 3) = array6[1]; mat4(2, 3) = array6[2];
}

void array6ToEigenMat4d(double* array6, Eigen::Matrix4d & mat4d)
{
	mat4d = Eigen::Matrix4d::Identity();
	double angle = magnitudeVec3(array6 + 3);
	Eigen::Vector3d axis(array6[3] / angle, array6[4] / angle, array6[5] / angle);
	Eigen::Matrix3d rotationMatrix;
	rotationMatrix = Eigen::AngleAxisd(angle, axis);
	mat4d.block<3, 3>(0, 0) = rotationMatrix;
	mat4d(0, 3) = array6[0]; mat4d(1, 3) = array6[1]; mat4d(2, 3) = array6[2];
}

void normalizeVec3(double* vec)
{
	double magnitude = magnitudeVec3(vec);

	for (int i = 0; i < 3; i++)
		vec[i] /= magnitude;
}

void array3ToEigenVec3(double* array3, Eigen::Vector3f &  vec3)
{
	for (int i = 0; i < 3; i++)
		vec3(i) = array3[i];
}

int main(int argc, char**argv)
{
	//pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZ>> SOR;


	VisionArmCombo vac;
	
	vac.mapWorkspaceUsingKinectArm();

	std::getchar();

	return 0;

	//vac.acquireCalibrationPointCloud();

	vac.readCloudAndPoseFromFile();

	std::cout << "calibration cloud size: " << vac.calibration_point_cloud_vec.size()<<"\n";

	for (auto pose : vac.hand_pose_vec_)
	{
		std::cout <<"hand pose \n"<< *pose << "\n";
	}

	vac.extractCalibrationPlaneCoefficientsSensorFrame();

	vac.calibrateScannerPoseSolver(1);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3d(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer3d->addCoordinateSystem(0.1);

	for (auto cloud : vac.calibration_point_cloud_vec)
	{
		viewer3d->removeAllPointClouds();
		viewer3d->addPointCloud(cloud, "cloud", 0);
		viewer3d->spin();
	}

	std::getchar();

	return 0;
	
	//RobotArmClient rac;
/*
	// calibrate tool point 
	int poseIdx = 0;
	
	const int numPoseNeeded = 4;

	double poseVec[numPoseNeeded][6];

	if (numPoseNeeded % 2 != 0 || numPoseNeeded < 4)
	{
		std::cout << "Num of poses needed wrong" << std::endl;
		return -1;
	}

	while (true)
	{
		
		std::cout << "Press Enter to save pose " << poseIdx << std::endl;
		std::getchar();

		//double pose[6] = { 0 };
		rac.getCartesianInfo(poseVec[poseIdx]);
		rac.printCartesianInfo(poseVec[poseIdx]);
	
		if (poseIdx == numPoseNeeded-1)
		{
			//std::cout << "size: "<<poseVec.size() << std::endl;
			Eigen::MatrixXd A(3 * numPoseNeeded / 2, 3);
			Eigen::VectorXd b(3 * numPoseNeeded / 2);

			for (int i = 0; i < numPoseNeeded; i+=2)
			{
				Eigen::Matrix4d T0;
				
				array6ToEigenMat4d(poseVec[i], T0);

				std::cout << "T0" << std::endl << T0 << std::endl;

				Eigen::Matrix4d T1;

				array6ToEigenMat4d(poseVec[i+1], T1);

				std::cout << "T1" << std::endl << T1 << std::endl;

				T0 = T0 - T1;

				std::cout << "T0-T1" << std::endl << T0 << std::endl;

				A.block<3, 3>(3 * (i / 2), 0) = T0.block<3, 3>(0, 0);
				b.block<3,1>(3 * (i / 2), 0) = T0.block<3, 1>(0, 3);
			}

			// Solve Ax=b
			std::cout << "A:" << std::endl << A << std::endl << "b:" << std::endl << b << std::endl;
			std::cout << "x:" << std::endl << A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b) << std::endl;

			break;
		}

		poseIdx++;
		
	}

	std::getchar();

	return 0;*/

	// move to start pose
	//double cartesian_start[6] = {-0.14898, -0.51984, 0.54554, 0.0271, -3.1445, 0.0966};

	/*rac.getCartesianInfo(cartesian_start);

	rac.printCartesianInfo(cartesian_start);

	Sleep(3000);

	rac.stopRecvTCP();

	std::getchar();*/

	// move to start pose
	//rac.moveHand(cartesian_start, 0.1);

	//rac.waitTillHandReachDstPose(cartesian_start);
	//Sleep(3000);

	// define end pose
	/*double cartesian_end[6];
	std::memcpy(cartesian_end, cartesian_start, 48);
	cartesian_end[0] -= 0.2;

	double distance = 0.;
	rac.getDistanceToDst(distance);

	//std::cout << "distance to dst: " << sqrt(distance)<< std::endl;

	KeyenceLineProfiler klf;

	klf.init();

	// move to end point
	rac.moveHand(cartesian_end, 0.05);

	klf.start(10);

	rac.waitTillHandReachDstPose(cartesian_end);

	klf.stop();

	rac.stopRecvTCP();

	//klf.saveToTxt("");

	// register point cloud
	PointCloudT::Ptr cloud(new PointCloudT);

	int num_profiles = klf.m_vecProfileData.size();

	std::cout << "num profiles: " << num_profiles<<std::endl;

	Eigen::Matrix4f startPose;
	Eigen::Matrix4f endPose;

	array6ToEigenMat4(cartesian_start, startPose);
	array6ToEigenMat4(cartesian_end, endPose);

	Eigen::Vector3f motionVector(cartesian_end[0] - cartesian_start[0], 
								 cartesian_end[1] - cartesian_start[1], 
								 cartesian_end[2] - cartesian_start[2]);

	std::cout << "motion vector:" << motionVector << std::endl;

	float magnitude = sqrt(motionVector.dot(motionVector));

	Eigen::Vector3f motionDelta = motionVector / (num_profiles - 1);

	std::cout << "motion delta:" << motionDelta << std::endl;


	for (int i = 0; i < num_profiles; i++)
	{
		for (int j = 0; j < 800; j++)
		{
			PointT point;

			point.z = klf.m_vecProfileData[i].m_pnProfileData[j] * (-1e-8f)+ j*motionDelta(2);

			if (abs(point.z) < 0.140f)
			{
				point.z += 0.3f;
				point.x = i*motionDelta(0);
				point.y = (float)(klf.m_profileInfo.lXStart + j*klf.m_profileInfo.lXPitch)*(-1e-8f) + j*motionDelta(1);

				point.r = (uint8_t)(255.f-255.f*(point.z+0.14f)/0.28f);
				point.g = point.r;
				point.b = 255;
				
				cloud->push_back(point);
			}
		}
	}

	std::cout << "point cloud size: " << cloud->size() << std::endl;

	klf.finalize();*/

	// Normal estimation*
/*	pcl::NormalEstimation<PointT, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.005);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);
	*/
	PointCloudT::Ptr cloud(new PointCloudT);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->addCoordinateSystem(0.1);

	viewer->addPointCloud(cloud, "cloud", 0);
	//viewer->addPolygonMesh(triangles, "mesh", 0);
	
	viewer->spin();

	std::getchar();

	return 0;
}