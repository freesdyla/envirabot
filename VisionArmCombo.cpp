#include "VisionArmCombo.h"

VisionArmCombo::VisionArmCombo()
{
	pre_point << 0, 0, 0;
	guessScannerToHand_ = Eigen::Matrix4f::Identity();
	Eigen::Matrix3f rot;
	rot = Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitZ());
	guessScannerToHand_.block<3,3>(0,0) = rot;
	guessScannerToHand_(0, 3) = -0.08f;	//x
	guessScannerToHand_(2, 3) = 0.095f;	//z

	kinect_cloud_.reset(new PointCloudT);
	kinect_cloud_->is_dense = true;

	// kinect
	cam2hand_kinect_ = Eigen::Matrix4f::Identity();
	cam2hand_kinect_(0, 3) = 0.0540247f;
	cam2hand_kinect_(1, 3) = 0.1026325f;
	cam2hand_kinect_(2, 3) = 0.0825227f;
}

void VisionArmCombo::pp_callback(const pcl::visualization::PointPickingEvent& event, void*)
{
	if (event.getPointIndex() == -1)
		return;
	Eigen::Vector3f current_point;
	event.getPoint(current_point[0], current_point[1], current_point[2]);

	std::cout << "current point:\n" << current_point << "\n";
	std::cout << "distance: " << (current_point - pre_point).norm() << "\n";
	pre_point = current_point;
}

double VisionArmCombo::magnitudeVec3(double * vec)
{
	return sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

void VisionArmCombo::array6ToEigenMat4d(double* array6, Eigen::Matrix4d & mat4d)
{
	mat4d = Eigen::Matrix4d::Identity();
	double angle = magnitudeVec3(array6 + 3);
	Eigen::Vector3d axis(array6[3] / angle, array6[4] / angle, array6[5] / angle);
	Eigen::Matrix3d rotationMatrix;
	rotationMatrix = Eigen::AngleAxisd(angle, axis);
	mat4d.block<3, 3>(0, 0) = rotationMatrix;
	mat4d(0, 3) = array6[0]; mat4d(1, 3) = array6[1]; mat4d(2, 3) = array6[2];
}

void VisionArmCombo::array6ToEigenMat4(double* array6, Eigen::Matrix4f & mat4)
{
	mat4 = Eigen::Matrix4f::Identity();
	double angle = magnitudeVec3(array6 + 3);
	Eigen::Vector3f axis(array6[3] / angle, array6[4] / angle, array6[5] / angle);
	Eigen::Matrix3f rotationMatrix;
	rotationMatrix = Eigen::AngleAxisf(angle, axis);
	mat4.block<3, 3>(0, 0) = rotationMatrix;
	mat4(0, 3) = array6[0]; mat4(1, 3) = array6[1]; mat4(2, 3) = array6[2];
}


void VisionArmCombo::initRobotArmClient()
{
	robot_arm_client_ = new RobotArmClient();
}

void VisionArmCombo::initLineProfiler()
{
	line_profiler_ = new KeyenceLineProfiler();
}

void VisionArmCombo::initKinectThread()
{
	kinect_thread_ = new KinectThread();
}

int VisionArmCombo::calibrateToolCenterPoint(Eigen::Vector3d & vec3d, int numPoseNeeded)
{
	if (robot_arm_client_ == NULL)
	{
		std::cout << "robot arm clinet not initialized" << std::endl;
		return -1;
	}

	int poseIdx = 0;

	std::vector<double*> poseVec;
	poseVec.resize(numPoseNeeded);

	for (int i = 0; i < numPoseNeeded; i++)
	{
		poseVec[i] = new double[6];
	}

	if (numPoseNeeded % 2 != 0 || numPoseNeeded < 4)
	{
		std::cout << "Num of poses needed wrong" << std::endl;
		return -1;
	}

	while (true)
	{
		std::cout << "Press Enter to save pose " << poseIdx << std::endl;
		std::getchar();

		robot_arm_client_->getCartesianInfo(poseVec[poseIdx]);
		robot_arm_client_->printCartesianInfo(poseVec[poseIdx]);

		if (poseIdx == numPoseNeeded - 1)
		{
			//std::cout << "size: "<<poseVec.size() << std::endl;
			Eigen::MatrixXd A(3 * numPoseNeeded / 2, 3);
			Eigen::VectorXd b(3 * numPoseNeeded / 2);

			for (int i = 0; i < numPoseNeeded; i += 2)
			{
				Eigen::Matrix4d T0;

				array6ToEigenMat4d(poseVec[i], T0);

				std::cout << "T0" << std::endl << T0 << std::endl;

				Eigen::Matrix4d T1;

				array6ToEigenMat4d(poseVec[i + 1], T1);

				std::cout << "T1" << std::endl << T1 << std::endl;

				T0 = T0 - T1;

				std::cout << "T0-T1" << std::endl << T0 << std::endl;

				A.block<3, 3>(3 * (i / 2), 0) = T0.block<3, 3>(0, 0);
				b.block<3, 1>(3 * (i / 2), 0) = T0.block<3, 1>(0, 3);
			}

			// Solve Ax=b
			std::cout << "A:" << std::endl << A << std::endl << "b:" << std::endl << b << std::endl;

			vec3d = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

			std::cout << "x:" << std::endl << vec3d << std::endl;

			break;
		}

		poseIdx++;
	}

	std::cout << "Press Enter" << std::endl;
	std::getchar();

	return 0;
}

/*
	assume robot arm already at start pose
	vec3d: motion vector in base frame
	cloud: PCL point cloud to save the data
*/
void VisionArmCombo::scanTranslateOnly(double * vec3d, PointCloudT::Ptr cloud)
{
	if (robot_arm_client_ == NULL || line_profiler_ == NULL)
	{
		std::cout << "robot arm or line profiler not initialized \n";
		return;
	}

	double curPoseD[6];
	robot_arm_client_->getCartesianInfo(curPoseD);

	robot_arm_client_->printCartesianInfo(curPoseD);

	double endPoseD[6];
	std::memcpy(endPoseD, curPoseD, 48);
	for (int i = 0; i < 3; i++) endPoseD[i] += vec3d[i];

	if (line_profiler_->device_initialized == false)
		line_profiler_->init();

	robot_arm_client_->moveHand(endPoseD, 0.05);
	line_profiler_->start(10);
	robot_arm_client_->waitTillHandReachDstPose(endPoseD);
	line_profiler_->stop();

	// register point cloud
	cloud->clear();

	int num_profiles = line_profiler_->m_vecProfileData.size();

	std::cout << "num profiles: " << num_profiles << std::endl;

	Eigen::Matrix4f startPose;
	Eigen::Matrix4f endPose;

	array6ToEigenMat4(curPoseD, startPose);
	array6ToEigenMat4(endPoseD, endPose);

	// express motion vector in base frame
	Eigen::Vector3f motionVector( endPoseD[0] - curPoseD[0],
							      endPoseD[1] - curPoseD[1],
								  endPoseD[2] - curPoseD[2] );

	std::cout << "motion vector in base frame:" << motionVector << std::endl;

	// transform motion vector in scanner frame
	/*Eigen::Matrix3f m;
	m = startPose.block<3, 3>(0,0);
	// A'*B' = (BA)' where A and B are rotation matrices. vec_senosr = Te2s*Tb2e*vec_base
	m = m*Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitZ());
	motionVector = m.transpose()*motionVector;*/

	motionVector = (guessScannerToHand_.block<3,3>(0,0)).transpose()*(startPose.block<3, 3>(0, 0)).transpose()*motionVector;

	std::cout << "motion vector in sensor frame:" << motionVector << std::endl;

	float magnitude = sqrt(motionVector.dot(motionVector));

	Eigen::Vector3f motionDelta = motionVector / (num_profiles - 1);

	std::cout << "motion delta:" << motionDelta << std::endl;

	std::cout << "x start: " << line_profiler_->m_profileInfo.lXStart << " pitch: " << line_profiler_->m_profileInfo.lXPitch << "\n";


	for (int i = 0; i < num_profiles; i++)
	{
		for (int j = 0; j < 800; j++)
		{
			PointT point;

			point.z = line_profiler_->m_vecProfileData[i].m_pnProfileData[j] * (-1e-8f);

			if (abs(point.z) < 0.140f)
			{
				point.z += 0.3f + i*motionDelta(2);
				point.y = i*motionDelta(1);
				point.x = (float)(line_profiler_->m_profileInfo.lXStart + j*line_profiler_->m_profileInfo.lXPitch)*(1e-8f) + i*motionDelta(0);

				point.r = (uint8_t)(255.f - 255.f*(point.z + 0.14f) / 0.28f);
				point.g = point.r;
				point.b = 255;

				cloud->push_back(point);
			}
		}
	}

	std::cout << "point cloud size: " << cloud->size() << std::endl;

	if (cloud->size() > 0)
	{
		std::string filename = getCurrentDateTimeStr() + ".pcd";

		pcl::io::savePCDFile(filename, *cloud, true);

		result_file_.open("ScannerRobotArmCalibrationFile.txt", std::ios::app);

		char buffer[100];

		sprintf(buffer, "%f,%f,%f,%f,%f,%f", curPoseD[0], curPoseD[1], curPoseD[2], curPoseD[3], curPoseD[4], curPoseD[5]);

		std::string startPoseStr(buffer);

		result_file_ << filename << "," << startPoseStr << std::endl;

		result_file_.close();
	}
}


void VisionArmCombo::readCloudAndPoseFromFile()
{
	std::ifstream file("ScannerRobotArmCalibrationFile.txt");
	
	if (file.is_open())
	{
		std::string line;

		calibration_point_cloud_vec.clear();

		while (std::getline(file, line))
		{
			std::size_t found = line.find_first_of(",");

			std::string front_part;

			double pose[6];

			if (found != std::string::npos && found > 0)
			{
				front_part = line.substr(0, found);
				std::cout << "file name: " << front_part << "\n";
				
				// trim line
				line = line.substr(found + 1, line.size()-1);

				// read point cloud
				PointCloudT::Ptr tmp_point_cloud (new PointCloudT);
				if(pcl::io::loadPCDFile(front_part, *tmp_point_cloud) == 0)
					calibration_point_cloud_vec.push_back(tmp_point_cloud);
				else
					std::cout<<"load pcd file "<<front_part<<" fail\n";
				
				//std::cout << line << "\n";	

				for (int i = 0; i < 5; i++)
				{
					found = line.find_first_of(",");

					if (found != std::string::npos)
					{
						front_part = line.substr(0, found);
						pose[i] = std::stod(front_part, 0);
						std::cout << i << " " << pose[i] << "\n";
						line = line.substr(found + 1, line.size() - 1);
					}
					else
					{
						std::cout << "wrong line\n";
						break;
					}
				}

				pose[5] = std::stod(line, 0);
				std::cout << "5 " << pose[5] << "\n";

				Eigen::Matrix4d *tmp_pose = new Eigen::Matrix4d;
				array6ToEigenMat4d(pose, *tmp_pose);
				hand_pose_vec_.push_back(tmp_pose);
			}
			else
			{
				std::cout << "wrong line\n";
				break;
			}				
		}

		file.close();
	}
	else std::cout << "Unable to open file";
}

std::string VisionArmCombo::getCurrentDateTimeStr()
{
	SYSTEMTIME st;

	GetSystemTime(&st);

	char currentTime[84] = "";

	std::sprintf(currentTime, "%d-%d-%d-%d-%d-%d-%d", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);

	return std::string(currentTime);
}

/*
	Solve scanner pose wrt robot hand assuming normalDistanceVec populated.
	Carlson, F. B., Johansson, R., & Robertsson, A. (2015, September). 
	Six DOF eye-to-hand calibration from 2D measurements using planar constraints. 
	In Intelligent Robots and Systems (IROS), 2015 IEEE/RSJ International Conference on (pp. 3628-3632). IEEE.
*/
void VisionArmCombo::calibrateScannerPoseSolver(int iterations)
{
	
	if (normalized_plane_coefficients_vec_sensor_frame_.size() != hand_pose_vec_.size() || hand_pose_vec_.size()==0)
	{
		std::cout << "calibration data wrong \n";
		return;
	}

	const int num_scan = hand_pose_vec_.size();
	std::vector<double> xs_vec; xs_vec.resize(num_scan);
	std::vector<double> ys_vec; ys_vec.resize(num_scan);
	std::vector<Eigen::Vector4d> noraml_distance_vec_base_frame;
	noraml_distance_vec_base_frame.resize(normalized_plane_coefficients_vec_sensor_frame_.size());

	// initialize scanner to hand matrix to identity
	curScannerToHand_ = guessScannerToHand_.cast<double>();

	std::cout << "initial scanner to hand:\n" << curScannerToHand_ << "\n";
	
	// find a point on the intersection of plane and xy-plane in sensor frame
	for (int i = 0; i < normalized_plane_coefficients_vec_sensor_frame_.size(); i++)
	{
		// plane unit normal + distance
		Eigen::Vector4d planeCo = *normalized_plane_coefficients_vec_sensor_frame_[i];
		double tmp_x = planeCo(3) / planeCo(0)*0.5;
		double tmp_y = planeCo(3) / planeCo(1)*0.5;

		// need to check isInf
		xs_vec[i] = tmp_x;
		ys_vec[i] = tmp_y;
	}
	
	// iterative solver
	for (int iter = 0; iter < iterations; iter++)
	{
		// transform plane normal distance to robot base frame. get n
		for (int i = 0; i < normalized_plane_coefficients_vec_sensor_frame_.size(); i++)
		{
			Eigen::Vector4d npc = *normalized_plane_coefficients_vec_sensor_frame_[i];
			Eigen::Vector4d plane_normal_distance_sensor_frame(npc(0)*npc(3), npc(1)*npc(3), npc(2)*npc(3), 1.0);
			Eigen::Vector3d unit_normal_base_frame = (*hand_pose_vec_[i]).block<3,3>(0,0)*curScannerToHand_.block<3,3>(0,0)*npc.block<3, 1>(0, 0);
			
			// transform the normal embedding distance back to base frame
			Eigen::Vector4d plane_normal_distance_base_frame = (*hand_pose_vec_[i]) * curScannerToHand_ * plane_normal_distance_sensor_frame;

			// project to unit normal in base frame
			double dot = abs(plane_normal_distance_base_frame.block<3, 1>(0, 0).transpose()*unit_normal_base_frame);

			noraml_distance_vec_base_frame[i].block<3,1>(0,0) = unit_normal_base_frame*dot;
		}

		// construct Ai = [n'*Rhand2base*xs	n'*Rhand2base*ys n'*Rhand2base] 1x9
		Eigen::MatrixXd A(num_scan, 9);

		for (int i = 0; i < noraml_distance_vec_base_frame.size(); i++)
		{
			Eigen::Vector3d n = noraml_distance_vec_base_frame[i].block<3,1>(0,0);
			Eigen::Matrix3d Rhand2base = (*hand_pose_vec_[i]).block<3,3>(0,0);

			A.block<1, 3>(i, 0) = n.transpose()*Rhand2base*xs_vec[i];
			A.block<1, 3>(i, 3) = n.transpose()*Rhand2base*ys_vec[i];
			A.block<1, 3>(i, 6) = n.transpose()*Rhand2base;
		}
		
		// construct Yi = ||n|| - n'*phand2base
		Eigen::VectorXd Y(num_scan);
		for (int i = 0; i < noraml_distance_vec_base_frame.size(); i++)
		{
			Eigen::Vector3d n = noraml_distance_vec_base_frame[i].block<3, 1>(0, 0);
			Y(i) = n.norm() - n.transpose()*(*hand_pose_vec_[i]).block<3, 1>(0, 3);
		}

		// solve w using SVD
		Eigen::VectorXd w(9);
		w = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);

		// get Rsensor2hand
		Eigen::Matrix3d tmp_rotation;
		Eigen::Vector3d rx = w.block<3, 1>(0, 0);
		Eigen::Vector3d ry = w.block<3, 1>(3, 0);
		tmp_rotation.block<3, 1>(0, 0) = rx;
		tmp_rotation.block<3, 1>(0, 1) = ry;
		tmp_rotation.block<3, 1>(0, 2) = rx.cross(ry);

		// Orthogonalize rotational matrix
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(tmp_rotation, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();
		Eigen::Matrix3d tmp = Eigen::Matrix3d::Identity();
		tmp(2, 2) = (U*V).determinant();
		Eigen::Matrix3d Rsensor2hand = U*tmp*V;

		Eigen::VectorXd Y_tilde(num_scan);
		for (int i = 0; i < num_scan; i++)
			Y_tilde(i) = Y(i) - A.block<1,3>(i, 0)*Rsensor2hand.block<3, 1>(0, 0)
						- A.block<1, 3>(i, 3)*Rsensor2hand.block<3, 1>(0, 1);

		Eigen::Vector3d refined_p;
		Eigen::MatrixXd A_7to9(num_scan, 3);
		A_7to9.col(0) = A.col(6); A_7to9.col(1) = A.col(7); A_7to9.col(2) = A.col(8);
		refined_p = A_7to9.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y_tilde);

		curScannerToHand_.block<3, 3>(0, 0) = Rsensor2hand;
		curScannerToHand_.block<3, 1>(0, 3) = refined_p;
	}

	std::cout << "final scanner to hand matrix\n" << curScannerToHand_ << "\n";
}

/*
	register point cloud from kinect in robot base frame
*/

void VisionArmCombo::mapWorkspaceUsingKinectArm()
{
	//initRobotArmClient();

	//initKinectThread();

	int scan_count = 0;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->addCoordinateSystem(0.3);

	viewer->registerPointPickingCallback(&VisionArmCombo::pp_callback, *this);

	viewer->spinOnce();

	float start_pose[6] = {-122.42f, -137.83f, -95.37f, 45.60f, 87.80f, -2.40f};
	float goal_pose[6] = {-60.31f, -137.23f, -98.76f, 45.61f, 87.80f, -2.41f};
	

	for (int i = 0; i < 6; i++) start_pose[i] = start_pose[i] / 180.f*M_PI;
	for (int i = 0; i < 6; i++) goal_pose[i] = goal_pose[i] / 180.f*M_PI;

	pcl::io::loadPCDFile("scene.pcd", *kinect_cloud_);

	viewer->addPointCloud(kinect_cloud_);	

	// get self collision free 
	//pp_.PRMCEPreprocessing(); pp_.savePathPlanner("pp");
	pp_.loadPathPlanner("pp");
	//return;

	if (!pp_.planPath(start_pose, goal_pose))
	{
		std::cout << "path not found\n";
		//return;
	}

	viewPlannedPath(pp_, viewer, start_pose, goal_pose);

	pp_.resetOccupancyGrid();

	pp_.addPointCloudToOccupancyGrid(kinect_cloud_);

	if (!pp_.planPath(start_pose, goal_pose))
	{
		std::cout << "path not found\n";
		//return;
	}

	showOccupancyGrid(pp_, viewer);

	viewPlannedPath(pp_, viewer, start_pose, goal_pose);

	return;

	/*while (true)
	{
		std::cout << "Move robot hand then press Enter to scan or 'q'+Enter to close\n";

		char key = std::getchar();

		if (key == 'q')
			break;
	
		// get point cloud from kinect
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		kinect_thread_->getCurPointCloud(point_cloud);

		// get robot hand pose
		double array6[6];

		double joint_array6[6];

		Eigen::Matrix4f hand2base;

		robot_arm_client_->getCartesianInfo(array6);

		robot_arm_client_->getCurJointPose(joint_array6);

		array6ToEigenMat4(array6, hand2base);

		//std::cout << "hand2pose\n" << hand2base << "\n";

		std::cout << "joint pos: ";
		
		for (int i = 0; i < 6; i++)
			//std::cout << joint_array6[i]/M_PI*180. << " ";
			std::cout << joint_array6[i] << " ";

		std::cout << std::endl<<"\n";

		Eigen::Matrix4f cam2base;

		cam2base = hand2base*cam2hand_kinect_;

		PointCloudT::Ptr cloud_in_base(new PointCloudT);

		pcl::transformPointCloud(*point_cloud, *cloud_in_base, cam2base);

		*kinect_cloud_ += *cloud_in_base;

		// voxel grid donwsample
		sor_.setInputCloud(kinect_cloud_);
		sor_.setLeafSize(0.01f, 0.01f, 0.01f);
		
		point_cloud->clear();
		sor_.filter(*point_cloud);

		kinect_cloud_->clear();
		*kinect_cloud_ += *point_cloud;

		viewer->removeAllPointClouds();

		viewer->addPointCloud(kinect_cloud_, "cloud", 0);

		std::cout << "Number of scans: " << ++scan_count << "\n";

		viewer->spin();

	}

	pcl::io::savePCDFileBinary("scene.pcd", *kinect_cloud_);

	std::cout << "move arm to START pose then hit Enter\n";	std::getchar();
	
	double start_joint_array6[6];
	robot_arm_client_->getCurJointPose(start_joint_array6);
	std::cout << "start joint pos: ";
	for (int i = 0; i < 6; i++)
		//std::cout << joint_array6[i]/M_PI*180. << " ";
		std::cout << start_joint_array6[i] << " ";
	std::cout << std::endl << "\n";


	std::cout << "move arm to END pose then hit Enter\n"; std::getchar();

	double end_joint_array6[6];
	robot_arm_client_->getCurJointPose(end_joint_array6);
	std::cout << "end joint pos: ";
	for (int i = 0; i < 6; i++)
		//std::cout << joint_array6[i]/M_PI*180. << " ";
		std::cout << end_joint_array6[i] << " ";
	std::cout << std::endl << "\n";

	std::cout << "plan path\n";

	//pp_.generateInitRandomNodes(kinect_cloud_);

	//pp_.searchPath(start_joint_array6, end_joint_array6);

	for (auto c : pp_.collision_vec_)
	{
		viewer->removeAllShapes();

		for (int i = 0; i < 6; i++)
		{
			pcl::ModelCoefficients cylinder_coeff;
			cylinder_coeff.values.resize(7);

			PointT p = c[i];
			PointT p1 = c[i + 1];

			cylinder_coeff.values[0] = p.x; cylinder_coeff.values[1] = p.y; cylinder_coeff.values[2] = p.z;
			cylinder_coeff.values[3] = p1.x - p.x; cylinder_coeff.values[4] = p1.y - p.y; cylinder_coeff.values[5] = p1.z - p.z;
			cylinder_coeff.values[6] = 0.05f;
			viewer->addCylinder(cylinder_coeff, "cylinder" + std::to_string(i), 0);

			//std::cout << p1 << "\n";
		}

		viewer->spin();
	}

	viewer->removeAllShapes();

	for (int i = 0; i < 6; i++)
	{
		pcl::ModelCoefficients cylinder_coeff;
		cylinder_coeff.values.resize(7);

		PointT p = pp_.arm_joint_points_[i];
		PointT p1 = pp_.arm_joint_points_[i + 1];

		cylinder_coeff.values[0] = p.x; cylinder_coeff.values[1] = p.y; cylinder_coeff.values[2] = p.z;
		cylinder_coeff.values[3] = p1.x - p.x; cylinder_coeff.values[4] = p1.y - p.y; cylinder_coeff.values[5] = p1.z - p.z;
		cylinder_coeff.values[6] = 0.05f;
		viewer->addCylinder(cylinder_coeff, "cylinder" + std::to_string(i), 0);

		//std::cout << p1 << "\n";
	}

	viewer->spin();*/
	
	robot_arm_client_->stopRecvTCP();
}

/*
	iteractive calibration process
*/
void VisionArmCombo::acquireCalibrationPointCloud()
{
	initRobotArmClient();

	initLineProfiler();

	int scan_count = 0;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->addCoordinateSystem(0.1);

	viewer->registerPointPickingCallback(&VisionArmCombo::pp_callback, *this);

	viewer->spinOnce();

	while (true)
	{
		std::cout << "Move robot hand then press Enter to scan or 'q'+Enter to close\n";

		char key = std::getchar();

		if (key == 'q')
			break;

		// sweeping scan
		double motion_vector[3];
		std::memset(motion_vector, 0, 3 * 8);
		motion_vector[0] -= 0.1;

		printf("motion vec: %f, %f, %f", motion_vector[0], motion_vector[1], motion_vector[2]);
		PointCloudT::Ptr point_cloud(new PointCloudT);
		
		scanTranslateOnly(motion_vector, point_cloud);
		
		viewer->removeAllPointClouds();

		viewer->addPointCloud(point_cloud, "cloud", 0);

		std::cout << "Number of scans: " << ++scan_count << "\n";

		viewer->spin();
	}

	robot_arm_client_->stopRecvTCP();
	line_profiler_->stop();
	line_profiler_->finalize();
}

void VisionArmCombo::extractCalibrationPlaneCoefficientsSensorFrame()
{
	// remove plane outliers 
	for (auto & cloud : calibration_point_cloud_vec)
	{
		PointCloudT::Ptr cloud_filtered(new PointCloudT);

		// Create the filtering object
		pcl::VoxelGrid<PointT> sor;
		sor.setInputCloud(cloud);
		sor.setLeafSize(0.005f, 0.005f, 0.005f);
		sor.filter(*cloud_filtered);

		// first do a RANSAC to remove outliers
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<PointT> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.005);
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);

		std::cerr << "Model coefficients: " << coefficients->values[0] << " "
			<< coefficients->values[1] << " "
			<< coefficients->values[2] << " "
			<< coefficients->values[3] << std::endl;

		cloud_filtered->clear();

		pcl::ModelOutlierRemoval<PointT> plane_filter;
		plane_filter.setModelCoefficients(*coefficients);
		plane_filter.setThreshold(0.005);
		plane_filter.setModelType(pcl::SACMODEL_PLANE);
		plane_filter.setInputCloud(cloud);
		plane_filter.filter(*cloud_filtered);
		
		cloud = cloud_filtered;
	}

	// extract 
	for (auto & cloud : calibration_point_cloud_vec)
	{
		pcl::PCA<PointT> pca(*cloud);

		Eigen::Vector4f centroid;
		centroid = pca.getMean();
		
		std::cout << "centroid \n" << centroid<<"\n";

		//Eigen::Vector3f eigen_values;
		//eigen_values = pca.getEigenValues();
		//std::cout << "eigen values \n" << eigen_values << "\n";

		Eigen::Matrix3f eigen_vectors;
		eigen_vectors = pca.getEigenVectors();

		std::cout << "eigen vector \n" << eigen_vectors << "\n";

		Eigen::Vector4d* plane_coeff = new Eigen::Vector4d;

		//ax + by + cz + d = 0, make c positive. (a,b,c) is from origion to plane
		if(eigen_vectors(2, 2)<0)
			(*plane_coeff) << (double)-eigen_vectors(0,2), (double)-eigen_vectors(1, 2), 
						(double)-eigen_vectors(2, 2), -abs((double)(eigen_vectors.col(2).transpose()*centroid.block<3,1>(0,0)));
		else
			(*plane_coeff) << (double)eigen_vectors(0, 2), (double)eigen_vectors(1, 2),
						(double)eigen_vectors(2, 2), -abs((double)(eigen_vectors.col(2).transpose()*centroid.block<3, 1>(0, 0)));

		normalized_plane_coefficients_vec_sensor_frame_.push_back(plane_coeff);

		std::cout << "normalized_plane_coefficients\n" << *plane_coeff << "\n\n";
	}
}

void VisionArmCombo::addArmModelToViewer(std::vector<PathPlanner::RefPoint>& ref_points, boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
	viewer->removeAllShapes();

	for (int i = 0; i < ref_points.size()-1; i++)
	{
		pcl::ModelCoefficients cylinder_coeff;
		cylinder_coeff.values.resize(7);

		PointT p; p.x = ref_points[i].coordinates[0]; p.y = ref_points[i].coordinates[1]; p.z = ref_points[i].coordinates[2];
		PointT p1; p1.x = ref_points[i + 1].coordinates[0]; p1.y = ref_points[i + 1].coordinates[1]; p1.z = ref_points[i + 1].coordinates[2];

		cylinder_coeff.values[0] = p.x; cylinder_coeff.values[1] = p.y; cylinder_coeff.values[2] = p.z;
		cylinder_coeff.values[3] = p1.x - p.x; cylinder_coeff.values[4] = p1.y - p.y; cylinder_coeff.values[5] = p1.z - p.z;
		cylinder_coeff.values[6] = pp_.arm_radius_lookup[i];

		viewer->addCylinder(cylinder_coeff, "cylinder" + std::to_string(i), 0);
	}

	viewer->spin();
}

void VisionArmCombo::addOBBArmModelToViewer(std::vector<PathPlanner::OBB> & arm_obbs, boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
	viewer->removeAllShapes();
	viewer->removeAllCoordinateSystems();
	viewer->addCoordinateSystem(0.3, "world", 0);

	for (int j = 0; j < arm_obbs.size(); j++)
	{
		pcl::ModelCoefficients cube_coeff;
		cube_coeff.values.resize(10);
		for (int i = 0; i < 3; i++) cube_coeff.values[i] = arm_obbs[j].C(i);
		Eigen::Quaternionf quat(arm_obbs[j].A);

		cube_coeff.values[3] = quat.x();
		cube_coeff.values[4] = quat.y();
		cube_coeff.values[5] = quat.z();
		cube_coeff.values[6] = quat.w();

		cube_coeff.values[7] = arm_obbs[j].a(0)*2.f;
		cube_coeff.values[8] = arm_obbs[j].a(1)*2.f;
		cube_coeff.values[9] = arm_obbs[j].a(2)*2.f;

		viewer->addCube(cube_coeff, "cube"+std::to_string(j), 0);

		Eigen::Affine3f transform;
		Eigen::Matrix4f temp_mat = Eigen::Matrix4f::Identity();
		temp_mat.block<3, 3>(0, 0) = arm_obbs[j].A;;
		temp_mat.block<3, 1>(0, 3) << arm_obbs[j].C(0), arm_obbs[j].C(1), arm_obbs[j].C(2);
		transform.matrix() = temp_mat;

		//if(j>0) std::cout << "obb collision of " <<j-1<<" and "<<j<<" = " << pp_.collisionOBB(arm_obbs[j-1], arm_obbs[j]) <<"\n";	
		viewer->addCoordinateSystem(0.2, transform, "co"+std::to_string(j), 0);
	}

	viewer->spin();
}

void VisionArmCombo::showOccupancyGrid(PathPlanner & pp, boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
	PointCloudT::Ptr cloud(new PointCloudT);

	for (int z = 0; z < pp.grid_height_; z++)
	{
		for (int y = 0; y < pp.grid_depth_; y++)
		{
			for (int x = 0; x < pp.grid_width_; x++)
			{
				int cell_idx = x + y*pp.grid_width_ + z*pp.grid_depth_*pp.grid_width_;

				PathPlanner::Cell cell = pp.grid_[cell_idx];

				if (cell.isOccupiedCounter == pp_.prmce_round_counter_ )
				{
					PointT p;
					p.x = ((x + 0.5f)*pp_.cell_size_ + pp_.grid_offset_x_)*0.01f; 
					p.y = ((y + 0.5f)*pp_.cell_size_ + pp_.grid_offset_y_)*0.01f; 
					p.z = ((z + 0.5f)*pp_.cell_size_ + pp_.grid_offset_z_)*0.01f;
					p.r = p.g = p.b = 255;
					cloud->points.push_back(p);
				}

				if (cell.sweptVolumneCounter == pp_.prmce_swept_volume_counter_)
				{
					PointT p;
					p.x = ((x + 0.5f)*pp_.cell_size_ + pp_.grid_offset_x_)*0.01f;
					p.y = ((y + 0.5f)*pp_.cell_size_ + pp_.grid_offset_y_)*0.01f;
					p.z = ((z + 0.5f)*pp_.cell_size_ + pp_.grid_offset_z_)*0.01f;
					p.r = 255; p.g = p.b = 0;
					cloud->points.push_back(p);
				}
			}
		}
	}

	std::cout << "vox cloud size: " << cloud->points.size() << "\n";

	viewer->addPointCloud(cloud, "grid", 0);

	viewer->spin();
}

void VisionArmCombo::viewPlannedPath(PathPlanner & pp, boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer, float* start_pose, float* goal_pose)
{
	std::vector<PathPlanner::RefPoint> ref_points;
	std::vector<Eigen::Matrix3f> rot_mats;
	std::vector<PathPlanner::OBB> arm_obbs;

	std::cout << "start pose\n";

	// visualization
	pp.computeReferencePointsOnArm(start_pose, ref_points, rot_mats);
	pp.getArmOBBModel(ref_points, rot_mats, arm_obbs); addOBBArmModelToViewer(arm_obbs, viewer);

	for (auto index : pp.shortest_path_index_vec_)
	{
		float config[6];

		memcpy(config, pp.random_nodes_buffer_ + index * 6, 6 * sizeof(float));

		pp.computeReferencePointsOnArm(config, ref_points, rot_mats);
		pp.getArmOBBModel(ref_points, rot_mats, arm_obbs); addOBBArmModelToViewer(arm_obbs, viewer);
	}

	std::cout << "goal pose\n";

	pp.computeReferencePointsOnArm(goal_pose, ref_points, rot_mats);
	pp.getArmOBBModel(ref_points, rot_mats, arm_obbs); addOBBArmModelToViewer(arm_obbs, viewer);
}