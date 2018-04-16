#include "VisionArmCombo.h"

//#define ENABLE_PP	//path planner

VisionArmCombo::VisionArmCombo() :
	voxel_grid_size_(0.002f),
	counter_(0)
{
	
	initVisionCombo();

	//	pp_.PRMCEPreprocessing(); pp_.savePathPlanner("pp"); for (int i = 1; i<pp_.num_nodes_; i += 2) viewPlannedPath(pp_.random_nodes_buffer_ + (i - 1) * 6, pp_.random_nodes_buffer_ + i * 6); return;

//	float Fo, Fm, FvFm, qP, qL, qN, NPQ, YNPQ, YNO, F, FmPrime, PAR, YII, ETR, FoPrime;

//	MiniPamActPlusYield(Fo, Fm, FvFm, qP, qL, qN, NPQ, YNPQ, YNO, F, FmPrime, PAR, YII, ETR, FoPrime);

#if 1

//	initRaman();
/*	raman_->getSpectrum(0, 7000);
	raman_->saveLastSpectrum("raman.csv");

	raman_->getSpectrum(1, 7000);
	raman_->saveLastSpectrum("raman_ir.csv");
	*/

#if 0
	initHyperspectralCam();	// must disconnect flir ethernet before specim is connected
#endif

	#if 0
	std::cout << "hyperspectral init done. Press key to continue.\n";
	std::getchar();
	initThermoCam();
	#endif

	initKinectThread();

	initLineProfiler();
	initRobotArmClient();
#endif

#if 0
	cv::Mat color, temp, hyper_img;
	//start imaging
	hypercam_->frames_.clear();
	hypercam_->frame_count_ = 0;
	hypercam_->start();

	while (hypercam_->frame_count_ == 0)
		Sleep(2);

	while (true) {

		// collect the first frame
		hyper_img = hypercam_->getLastestFrame();

		cv::imshow("hyper", hyper_img);

		thermocam_->snapShot(color, temp);
		cv::imshow("thermal", color);
		cv::waitKey(10);
	}
#endif

#ifdef ENABLE_PP
	if (!pp_.loadPathPlanner("pp"))	std::cout << "load path planner fail\n";
	for (int i = 1; i<pp_.num_nodes_; i += 2) viewPlannedPath(pp_.random_nodes_buffer_ + (i - 1) * 6, pp_.random_nodes_buffer_ + i * 6); return;
	//viewPlannedPath(pp_.random_nodes_buffer_, pp_.random_nodes_buffer_ + 6); viewer_->spin();
#endif

#if 0
	PointCloudT::Ptr tmp_point_cloud(new PointCloudT);

	PointCloudT::Ptr point_cloud(new PointCloudT);

	Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();

	translation(0, 3) = -0.6;
/*
	for (int i = 1; i <= 4; i++)
	{
		tmp_point_cloud->points.clear();
		std::string name = "laser_scan" + std::to_string(i) + ".pcd";
		pcl::io::loadPCDFile(name, *tmp_point_cloud);


		pcl::transformPointCloud(*tmp_point_cloud, *tmp_point_cloud, translation);

		*point_cloud += *tmp_point_cloud;
	}
*/
	pcl::io::loadPCDFile("laser_scan.pcd", *point_cloud);
	viewer_->addPointCloud(point_cloud, "cloud", 0);
	viewer_->spin();
	viewer_->removeAllPointClouds();
	std::vector<pcl::PointXYZRGBNormal> probe_pn_vec;
	extractLeafProbingPointsAndHyperspectralScanPoses(point_cloud, probe_pn_vec);
#endif
}

VisionArmCombo::~VisionArmCombo()
{
	motor_controller_.Disconnect();
}

void VisionArmCombo::initVisionCombo()
{
#ifdef ROAD
	//gripper_.activate();
	//gripper_.open();
#endif
	home_config_.setJointPos(-90., 0., -120., -58., 90., -180.);
	home_config_.toRad();

	viewer_.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer_->addCoordinateSystem(0.3);
	viewer_->setSize(1366, 768);
	viewer_->setPosition(0, 100);
	viewer_->registerPointPickingCallback(&VisionArmCombo::pp_callback, *this);

	// load line scanner hand eye calibration matrix
	guessHandToScanner_ = Eigen::Matrix4f::Identity();
	guessHandToScanner_.block<3, 3>(0, 0) = Eigen::AngleAxisf(-0.5*M_PI, Eigen::Vector3f::UnitZ()).matrix();
	guessHandToScanner_.col(3).head(3) << 0.076073, -0.00502, 0.09425;

	handToScanner_ = guessHandToScanner_;

	//std::cout << "hand to scanner\n" << handToScanner_ << "\n";

	std::ifstream file("lineScannerHandEyeCalibration.bin", std::ios::in | std::ios::binary);
	if (file.is_open())
	{
		char buffer[64];
		file.read(buffer, 64);
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
			{
				float* ptr = (float*)buffer + i * 4 + j;
		//		handToScanner_.row(i)(j) = *(ptr);
			}
		std::cout << "handToScanner:\n" << handToScanner_ << "\n";
	}
	else std::cout << "lineScannerHandEyeCalibration load fail\n";
	file.close();

	// hyperspectral camera pose
	hand_to_hyperspectral_ = Eigen::Matrix4f::Identity();
	hand_to_hyperspectral_.block<3, 3>(0, 0) = Eigen::AngleAxisf(-0.5*M_PI, Eigen::Vector3f::UnitZ()).matrix();
	hand_to_hyperspectral_.col(3).head(3) << 0.046736, 0.062186, 0.11;

	// thermal camera pose
	hand_to_thermal_ = Eigen::Matrix4f::Identity();
	hand_to_thermal_.col(3).head(3) << -0.122, 0.057, 0.12;

	cv::FileStorage fs("tool_center_point_calib.yml", cv::FileStorage::READ);
	cv::Vec3d tcp;
	if (fs.isOpened())
	{
		fs["tcp"] >> tcp;
	}
	fs.release();

	for (int i = 0; i < 3; i++) tool_center_point_(i) = tcp[i];

	//tool_center_point_ << 0.0348893, -0.0440583, 0.18337;	//10/24/2016	Enviratron

	//std::cout << "tcp: " << tcp<<"\n";

	probe_to_hand_ = Eigen::Matrix4d::Identity();

	probe_to_hand_.col(3).head<3>() = tool_center_point_.cast<double>();

	//probe pointing up, rotate around y of hand by 180
	probe_to_hand_(0, 0) = -1.; probe_to_hand_(2, 2) = -1.;

	probe_to_hand_ = probe_to_hand_.inverse();

	marker_length_ = 0.1016f;	//4 inch

	fs.open("tool_center_point_calib_PAM.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["tcp"] >> tcp;
	}

	fs.release();

	for (int i = 0; i < 3; i++) tool_center_point_pam_(i) = tcp[i];
	
	probe_to_hand_pam_ = Eigen::Matrix4d::Identity();

	probe_to_hand_pam_.col(3).head<3>() = tool_center_point_pam_.cast<double>();

	//probe pointing up, rotate around y of hand by 180
	probe_to_hand_pam_(0, 0) = -1.; probe_to_hand_pam_(2, 2) = -1.;

	probe_to_hand_pam_ = probe_to_hand_pam_.inverse();

	fs.open("tool_center_point_calib_RAMAN_532.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["tcp"] >> tcp;
	}

	fs.release();

	for (int i = 0; i < 3; i++) tool_center_point_raman_532_(i) = tcp[i];

	probe_to_hand_raman_532_ = Eigen::Matrix4d::Identity();

	probe_to_hand_raman_532_.col(3).head<3>() = tool_center_point_raman_532_.cast<double>();

	//probe pointing up, rotate around y of hand by 180
	probe_to_hand_raman_532_(0, 0) = -1.; probe_to_hand_raman_532_(2, 2) = -1.;

	probe_to_hand_raman_532_ = probe_to_hand_raman_532_.inverse();

	fs.open("tool_center_point_calib_RAMAN_1064.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["tcp"] >> tcp;
	}

	fs.release();

	for (int i = 0; i < 3; i++) tool_center_point_raman_1064_(i) = tcp[i];

	probe_to_hand_raman_1064_ = Eigen::Matrix4d::Identity();

	probe_to_hand_raman_1064_.col(3).head<3>() = tool_center_point_raman_1064_.cast<double>();

	//probe pointing up, rotate around y of hand by 180
	probe_to_hand_raman_1064_(0, 0) = -1.; probe_to_hand_raman_1064_(2, 2) = -1.;

	probe_to_hand_raman_1064_ = probe_to_hand_raman_1064_.inverse();

	

	fs.open("kinectRGBCalibration.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["camera_matrix"] >> kinect_rgb_camera_matrix_cv_;

		fs["distortion_coefficients"] >> kinect_rgb_dist_coeffs_cv_;
	}

	fs.release();

	fs.open("kinectRGBHandEyeCalibration.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["hand to eye"] >> kinect_rgb_hand_to_eye_cv_;
	}

	fs.release();

	fs.open("FlirThermalCalibration.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["camera_matrix"] >> flir_thermal_camera_matrix_cv_;

		fs["distortion_coefficients"] >> flir_thermal_dist_coeffs_cv_;
	}

	fs.release();

	fs.open("FlirThermalHandEyeCalibration.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["hand to eye"] >> flir_thermal_hand_to_eye_cv_;
	}

	fs.release();

	cvTransformToEigenTransform(flir_thermal_hand_to_eye_cv_, hand_to_thermal_d_);

	hand_to_thermal_ = hand_to_thermal_d_.cast<float>();

	// entry full
	fs.open("parameters.yml", cv::FileStorage::READ);
	if (fs.isOpened())
	{
		fs["shelf_z_"] >> shelf_z_;
		fs["voxel_grid_size_laser_"] >> voxel_grid_size_laser_;
		fs["normal_estimation_radius_"] >> normal_estimation_radius_;
		fs["region_grow_residual_threshold_"] >> region_grow_residual_threshold_;
		fs["region_grow_smoothness_threshold_"] >> region_grow_smoothness_threshold_;
		region_grow_smoothness_threshold_ = region_grow_smoothness_threshold_ / 180.*M_PI;
		fs["region_grow_curvature_threshold_"] >> region_grow_curvature_threshold_;
		fs["region_grow_num_neighbors_"] >> region_grow_num_neighbors_;
		fs["region_grow_min_cluster_size_"] >> region_grow_min_cluster_size_;
		fs["region_grow_max_cluster_size_"] >> region_grow_max_cluster_size_;
		fs["plate_center_x_"] >> plate_center_[0];
		fs["plate_center_y_"] >> plate_center_[1];
		fs["plate_center_z_"] >> plate_center_[2];
		fs["plate_radius_"] >> plate_radius_;
		double f;
		fs["laser_scan_frequency_"] >> f;
		laser_scan_period_ = 1.0 / f;
		fs["scan_speed_"] >> scan_speed_;
		fs["scan_acceleration_"] >> scan_acceleration_;
		fs["move_arm_speed_"] >> move_arm_speed_;
		fs["move_arm_acceleration_"] >> move_arm_acceleration_;
		fs["move_joint_speed_"] >> move_joint_speed_;
		fs["move_joint_acceleration_"] >> move_joint_acceleration_;
		fs["view_time_"] >> view_time_;
		fs["seed_resolution_"] >> seed_resolution_;
		fs["spatial_importance_"] >> spatial_importance_;
		fs["normal_importance_"] >> normal_importance_;
		fs["sor_mean_k_"] >> sor_mean_k_;
		fs["sor_std_"] >> sor_std_;
		fs["plant_center_x_"] >> plant_center_[0];
		fs["plant_center_y_"] >> plant_center_[1];
		fs["plant_center_z_"] >> plant_center_[2];
		fs["min_point_AABB_x_"] >> min_point_AABB_(0);
		fs["min_point_AABB_y_"] >> min_point_AABB_(1);
		fs["min_point_AABB_z_"] >> min_point_AABB_(2);
		fs["max_point_AABB_x_"] >> max_point_AABB_(0);
		fs["max_point_AABB_y_"] >> max_point_AABB_(1);
		fs["max_point_AABB_z_"] >> max_point_AABB_(2);
		fs["only_do_probing_"] >> only_do_probing_;
		fs["num_plants_"] >> num_plants_;
		fs["hyperspectral_imaging_dist_"] >> hyperspectral_imaging_dist_;
		fs["max_samples_per_leaf_"] >> max_samples_per_leaf_;
		fs["probing_patch_rmse_"] >> probing_patch_rmse_;
		fs["hyperscan_slice_thickness_"] >> hyperscan_slice_thickness_;
		fs["enable_probing_"] >> enable_probing_;
	}
	fs.release();


	fs.open("kinectIRCalibration.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["camera_matrix"] >> kinect_infrared_camera_matrix_cv_;

		fs["distortion_coefficients"] >> kinect_infrared_dist_coeffs_cv_;

		fs.release();
	}

	fs.open("kinectIRHandEyeCalibration.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["hand to eye"] >> kinect_infrared_hand_to_eye_cv_;

		fs.release();
	}

	// kinect
/*	cam2hand_kinect_ = Eigen::Matrix4f::Identity();
	cam2hand_kinect_(0, 3) = 0.0540247f;
	cam2hand_kinect_(1, 3) = 0.1026325f;
	cam2hand_kinect_(2, 3) = 0.0825227f;
	*/

	cvTransformToEigenTransform(kinect_infrared_hand_to_eye_cv_, hand_to_depth_);

	cam2hand_kinect_ = hand_to_depth_.cast<float>();

	cvTransformToEigenTransform(kinect_rgb_hand_to_eye_cv_, hand_to_rgb_);


	marker_dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);

	detector_params_ = cv::aruco::DetectorParameters::create();

	detector_params_->doCornerRefinement = true; // do corner refinement in markers

	cur_rgb_to_marker_ = Eigen::Matrix4d::Identity();

	pre_point_ << 0, 0, 0;

	scan_start_to_hand_ = Eigen::Matrix4d::Identity();

	scan_start_to_hand_(2, 3) = 0.3; scan_start_to_hand_(1, 3) = -0.05;

	scan_start_to_hand_ = scan_start_to_hand_.inverse()*handToScanner_.cast<double>().inverse();

	scan_end_to_hand_ = Eigen::Matrix4d::Identity();

	scan_end_to_hand_(2, 3) = 0.3; scan_end_to_hand_(1, 3) = +0.05;

	scan_end_to_hand_ = scan_end_to_hand_.inverse()*handToScanner_.cast<double>().inverse();

	kinect_cloud_.reset(new PointCloudT);
	kinect_cloud_->is_dense = true;

	laser_cloud_.reset(new PointCloudT);
	laser_cloud_->is_dense = true;

	growthChamberPointCloudVec_.resize(3);
	for (int i = 0; i < 3; i++)
	{
		growthChamberPointCloudVec_[i].reset(new PointCloudT);
		growthChamberPointCloudVec_[i]->is_dense = true;
	}

	ArmConfig imaging_config;
	//imaging_config.setJointPos(-89.77, -15., -77.72, -89.16, 94.92, -182.31);
	//imaging_config.toRad();
	//imaging_config_vec.push_back(imaging_config);

	imaging_config.setJointPos(-88.55, -91.57, -92.23, -80.27, 90.6, -180.68);
	imaging_config.toRad();
	imaging_config_vec.push_back(imaging_config);

	// this is for probing plants on the bench in lab
	/*imaging_config.setJointPos(-88.87, -77.01, -83.68, -103.93, 89.69, -175.55);
	imaging_config.toRad();
	imaging_config_vec.push_back(imaging_config);*/

	/*imaging_config.setJointPos(-95.44, -84.73, -65.48, -120.08, 90.85, -183.29);
	imaging_config.toRad();
	imaging_config_vec.push_back(imaging_config);*/

	//imaging_config.setJointPos(-95.29, -94.71, -33.16, -136.00, 90.84, -183.12);
	//imaging_config.toRad();
	//imaging_config_vec.push_back(imaging_config);

	//imaging_config.setJointPos(-59.32, -95.62, -31.90, -137.05, 87.10, -147.33);
	//imaging_config.toRad();
	//imaging_config_vec.push_back(imaging_config);

	sor_.setMeanK(sor_mean_k_);
	sor_.setStddevMulThresh(sor_std_);

	int status = motor_controller_.Connect("COM1");

	if (status != RQ_SUCCESS)
		std::cout << "Error connecting to motor controller: " << status << "\n";
}

void VisionArmCombo::pp_callback(const pcl::visualization::PointPickingEvent& event, void*)
{
	if (event.getPointIndex() == -1)
		return;
	Eigen::Vector3f current_point;
	event.getPoint(current_point[0], current_point[1], current_point[2]);

	pre_point_idx_ = event.getPointIndex();

	std::cout << "current point:\n" << current_point << "\n";
	std::cout << "distance: " << (current_point - pre_point_).norm() << "\n";
	pre_point_ = current_point;
	pre_viewer_pose_ = viewer_->getViewerPose(0);
	std::cout << "viewer pose\n"<< pre_viewer_pose_.matrix() << "\n";
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

void VisionArmCombo::eigenMat4dToArray6(Eigen::Matrix4d & mat4d, double * array6)
{
	Eigen::AngleAxisd angle_axis(mat4d.topLeftCorner<3,3>());

	array6[0] = mat4d(0, 3); array6[1] = mat4d(1, 3); array6[2] = mat4d(2, 3);

	Eigen::Vector3d rotation_vector = angle_axis.angle()*angle_axis.axis();

	array6[3] = rotation_vector[0]; array6[4] = rotation_vector[1]; array6[5] = rotation_vector[2];
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

void VisionArmCombo::initHyperspectralCam() {

	hypercam_ = new HyperspectralCamera();

	hypercam_->init();
}

void VisionArmCombo::initThermoCam() {

	ShellExecute(NULL, TEXT("open"), TEXT("C:\\Users\\lietang123\\Documents\\RoAdFiles\\FlirThermoCamServer\\FlirThermoCamServer\\bin\\Release\\FlirThermoCamServer.exe"), NULL, NULL, SW_SHOWDEFAULT);

	Sleep(6000);

	thermocam_ = new FlirThermoCamClient();
}

void VisionArmCombo::initRaman()
{
	raman_ = new Raman();
}

float VisionArmCombo::DDERequest(DWORD idInst, HCONV hConv, wchar_t* szItem)
{
	HSZ hszItem = DdeCreateStringHandle(idInst, szItem, 0);
	HDDEDATA hData = DdeClientTransaction(NULL, 0, hConv, hszItem, CF_TEXT,
		XTYP_REQUEST, 2000, NULL);
	if (hData == NULL)
	{
		printf("Request failed: %s\n", szItem);
	}
	else
	{
		char szResult[255];
		DdeGetData(hData, (unsigned char *)szResult, 255, 0);
		//printf("%s\n", szResult);
		return atof(szResult);
	}

	return -1.;
}

int VisionArmCombo::MiniPamBasicData(float & PAR, float & YII)
{
	//DDE Initialization
	DWORD idInst = 0;
	UINT iReturn;
	iReturn = DdeInitialize(&idInst, (PFNCALLBACK)NULL, APPCLASS_STANDARD | APPCMD_CLIENTONLY, 0);

	if (iReturn != DMLERR_NO_ERROR)
	{
		printf("DDE Initialization Failed: 0x%04x\n", iReturn);
		Sleep(1500);
		return -1;
	}

	//DDE Connect to Server using given AppName and topic.
	HSZ hszApp, hszTopic;
	HCONV hConv;
	hszApp = DdeCreateStringHandle(idInst, L"WinControl", 0);
	hszTopic = DdeCreateStringHandle(idInst, L"WinControl", 0);
	hConv = DdeConnect(idInst, hszApp, hszTopic, NULL);
	DdeFreeStringHandle(idInst, hszApp);
	DdeFreeStringHandle(idInst, hszTopic);
	if (hConv == NULL)
	{
		printf("DDE Connection Failed.\n");
		Sleep(1500); DdeUninitialize(idInst);
		return -1;

	}
	wchar_t* command = L"S=1";
	HDDEDATA hData = DdeClientTransaction((LPBYTE)command, 2 * lstrlen(command) - 1, hConv, 0L, 0, XTYP_EXECUTE, 1000, NULL);
	if (hData == NULL)
		std::cout << "Act.+Yield execute fail\n";

	wchar_t* variable_name = L"S";
	HSZ hszItem = DdeCreateStringHandle(idInst, variable_name, 0);

	//wait for AY = 0
	for (int i = 0; i < 20; i++) {

		Sleep(1000);

		HDDEDATA hData = DdeClientTransaction(NULL, -1, hConv, hszItem, CF_TEXT, XTYP_REQUEST, 1000, NULL);
		if (hData == NULL)
			printf("Request failed: %s\n", variable_name);
		else
		{
			char szResult[4];
			DdeGetData(hData, (unsigned char *)szResult, 4, 0);

			printf("%s\n", szResult);

			if (std::strncmp(szResult, "0", 1) == 0)
				break;
		}
	}

	PAR = DDERequest(idInst, hConv, L"PAR");
	YII = DDERequest(idInst, hConv, L"YII");

	std::cout << "YII: " << YII << "    PAR:" << PAR << "\n";

	//DDE Disconnect and Uninitialize.
	DdeDisconnect(hConv);
	DdeUninitialize(idInst);

	return 0;
}

int VisionArmCombo::MiniPamActPlusYield(float & Fo, float & Fm, float & FvFm, float & qP, float & qL,
	float & qN, float & NPQ, float & YNPQ, float & YNO, float & F, float & FmPrime, float & PAR,
	float & YII, float & ETR, float & FoPrime)
{
	//DDE Initialization
	DWORD idInst = 0;
	UINT iReturn;
	iReturn = DdeInitialize(&idInst, (PFNCALLBACK)NULL, APPCLASS_STANDARD | APPCMD_CLIENTONLY, 0);

	if (iReturn != DMLERR_NO_ERROR)
	{
		printf("DDE Initialization Failed: 0x%04x\n", iReturn);
		Sleep(1500);
		return -1;
	}

	//DDE Connect to Server using given AppName and topic.
	HSZ hszApp, hszTopic;
	HCONV hConv;
	hszApp = DdeCreateStringHandle(idInst, L"WinControl", 0);
	hszTopic = DdeCreateStringHandle(idInst, L"WinControl", 0);
	hConv = DdeConnect(idInst, hszApp, hszTopic, NULL);
	DdeFreeStringHandle(idInst, hszApp);
	DdeFreeStringHandle(idInst, hszTopic);
	if (hConv == NULL)
	{
		printf("DDE Connection Failed.\n");
		Sleep(1500); DdeUninitialize(idInst);
		return -1;
	}

	wchar_t* FoPrimeMode = L"FOM=1";
	HDDEDATA hData = DdeClientTransaction((LPBYTE)FoPrimeMode, 2 * lstrlen(FoPrimeMode) - 1, hConv, 0L, 0, XTYP_EXECUTE, 1000, NULL);

	wchar_t* command = L"AY=1";
	hData = DdeClientTransaction((LPBYTE)command, 2 * lstrlen(command) - 1, hConv, 0L, 0, XTYP_EXECUTE, 1000, NULL);
	if (hData == NULL)
		std::cout << "Act.+Yield execute fail\n";

	wchar_t* variable_name = L"AY";
	HSZ hszItem = DdeCreateStringHandle(idInst, variable_name, 0);

	//wait for AY = 0
	for (int i = 0; i < 40; i++) {

		Sleep(1000);
	
		HDDEDATA hData = DdeClientTransaction(NULL, -1, hConv, hszItem, CF_TEXT, XTYP_REQUEST, 1000, NULL);
		if (hData == NULL)
			printf("Request failed: %s\n", variable_name);
		else
		{
			char szResult[4];
			DdeGetData(hData, (unsigned char *)szResult, 4, 0);

			printf("%s\n", szResult);

			if (std::strncmp(szResult, "0", 1) == 0)
				break;
		}
	}

	Fo = DDERequest(idInst, hConv, L"Fo");
	Fm = DDERequest(idInst, hConv, L"Fm");
	FvFm = DDERequest(idInst, hConv, L"Fv");
	qP = DDERequest(idInst, hConv, L"qP");
	qL = DDERequest(idInst, hConv, L"qL");
	qN = DDERequest(idInst, hConv, L"qN");
	NPQ = DDERequest(idInst, hConv, L"NPQ");
	YNPQ = DDERequest(idInst, hConv, L"YNPQ");
	YNO = DDERequest(idInst, hConv, L"YNO");
	F = DDERequest(idInst, hConv, L"F");
	FmPrime = DDERequest(idInst, hConv, L"FmPrime");
	PAR = DDERequest(idInst, hConv, L"PAR");
	YII = DDERequest(idInst, hConv, L"YII");
	ETR = DDERequest(idInst, hConv, L"ETR");
	FoPrime = DDERequest(idInst, hConv, L"FoPrime");

	std::cout << "Fo: " << Fo << "\nFm: " << Fm << "\nFv/Fm: " << FvFm << "\nqP: " << qP << "\nqL: " << qL << "\nqN: " << qN
		<< "\nNPQ: " << NPQ << "\nY(NPQ): " << YNPQ << "\nY(NO): " << YNO << "\nF: " << F << "\nFm'" << FmPrime
		<< "\nPAR: " << PAR << "\nY(II): " << YII << "\nETR: " << ETR << "\nFo':" << FoPrime << "\n";


	//DDE Disconnect and Uninitialize.
	DdeDisconnect(hConv);
	DdeUninitialize(idInst);

	return 0;
}

void VisionArmCombo::calibrateToolCenterPoint(int numPoseNeeded, int probe_id)
{
	if (robot_arm_client_ == NULL) initRobotArmClient(); 

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
		return;
	}

	Eigen::Vector3d vec3d;

	while (true)
	{
		std::cout << "Press Enter to save pose " << poseIdx << std::endl;
		std::getchar();

		robot_arm_client_->getCartesianInfo(poseVec[poseIdx]);
		robot_arm_client_->printCartesianInfo(poseVec[poseIdx]);

		if (poseIdx == numPoseNeeded - 1)
		{
			//std::cout << "size: "<<poseVec.size() << std::endl;
			Eigen::MatrixXd A(3 * (numPoseNeeded)*(numPoseNeeded-1), 3);
			Eigen::VectorXd b(3 * (numPoseNeeded)*(numPoseNeeded-1));

			int idx = 0;

			for (int i = 0; i < numPoseNeeded; i++)
			{
				for (int j = 0; j < numPoseNeeded; j++)
				{
					if (i != j)
					{
						Eigen::Matrix4d T0;

						array6ToEigenMat4d(poseVec[i], T0);

						//std::cout << "T0" << std::endl << T0 << std::endl;

						Eigen::Matrix4d T1;

						array6ToEigenMat4d(poseVec[j], T1);

						//std::cout << "T1" << std::endl << T1 << std::endl;

						T0 = T0 - T1;

						//std::cout << "T0-T1" << std::endl << T0 << std::endl;

						A.block<3, 3>(3 *idx, 0) = T0.block<3, 3>(0, 0);

						b.block<3, 1>(3 *idx, 0) = T0.block<3, 1>(0, 3);
						++idx;
					}
				}
			}

		/*	Eigen::MatrixXd A(3 * numPoseNeeded / 2, 3);
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
			}*/

			// Solve Ax=b
			//std::cout << "A:" << std::endl << A << std::endl << "b:" << std::endl << b << std::endl;

			vec3d = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

			vec3d *= -1.;

			std::cout << "x (hand to TCP):" << std::endl << vec3d << std::endl;

			break;
		}

		poseIdx++;
	}

	cv::FileStorage fs;
	
	if (probe_id == RAMAN_532) {

		fs.open("tool_center_point_calib_RAMAN_532.yml", cv::FileStorage::WRITE);
	}
	else if (probe_id == RAMAN_1064) {

		fs.open("tool_center_point_calib_RAMAN_1064.yml", cv::FileStorage::WRITE);
	}
	else if (probe_id == PAM) {

		fs.open("tool_center_point_calib_PAM.yml", cv::FileStorage::WRITE);
	}
	else {

		std::cout << "wrong probe id!\n";
		return;
	}

	cv::Vec3d tcp;
	tcp[0] = vec3d(0); tcp[1] = vec3d(1); tcp[2] = vec3d(2);

	fs << "tcp" << tcp;

	for (int j = 0; j < poseVec.size(); j++)
	{
		cv::Vec6d cv_pose;
		
		for (int i = 0; i < 6; i++)
			cv_pose[i] = poseVec[j][i];

		fs << "pose" + std::to_string(j) << cv_pose;
	}

	fs.release();
	
	std::cout << "Saved" << std::endl;
}

void VisionArmCombo::calibrateGripperTip(int numPoseNeeded)
{
	if (robot_arm_client_ == NULL) initRobotArmClient();

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
		return;
	}

	Eigen::Vector3d vec3d;

	while (true)
	{
		std::cout << "Press Enter to save pose " << poseIdx << std::endl;
		std::getchar();

		robot_arm_client_->getCartesianInfo(poseVec[poseIdx]);
		robot_arm_client_->printCartesianInfo(poseVec[poseIdx]);

		if (poseIdx == numPoseNeeded - 1)
		{
			//std::cout << "size: "<<poseVec.size() << std::endl;
			Eigen::MatrixXd A(3 * (numPoseNeeded)*(numPoseNeeded - 1), 3);
			Eigen::VectorXd b(3 * (numPoseNeeded)*(numPoseNeeded - 1));

			int idx = 0;

			for (int i = 0; i < numPoseNeeded; i++)
			{
				for (int j = 0; j < numPoseNeeded; j++)
				{
					if (i != j)
					{
						Eigen::Matrix4d T0;

						array6ToEigenMat4d(poseVec[i], T0);

						Eigen::Matrix4d T1;

						array6ToEigenMat4d(poseVec[j], T1);

						T0 = T0 - T1;

						A.block<3, 3>(3 * idx, 0) = T0.block<3, 3>(0, 0);

						b.block<3, 1>(3 * idx, 0) = T0.block<3, 1>(0, 3);
						++idx;
					}
				}
			}

			vec3d = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

			vec3d *= -1.;

			std::cout << "x (hand to TCP):" << std::endl << vec3d << std::endl;

			break;
		}

		poseIdx++;
	}

	cv::FileStorage fs("gripper_tip_calib.yml", cv::FileStorage::WRITE);

	cv::Vec3d tcp;
	tcp[0] = vec3d(0); tcp[1] = vec3d(1); tcp[2] = vec3d(2);

	fs << "gripper" << tcp;

	fs.release();

	std::cout << "Saved" << std::endl;
}

/*
	assume robot arm already at start pose
	vec3d: motion vector in base frame
	cloud: PCL point cloud to save the data
*/
void VisionArmCombo::scanTranslateOnly(double * vec3d, PointCloudT::Ptr  cloud, float acceleration, float speed)
{
	if (acceleration == 0 || speed == 0)
	{
		std::cout << "acceleration or speed = 0\n";
		return;
	}

	if (robot_arm_client_ == NULL || line_profiler_ == NULL)
	{
		std::cout << "robot arm or line profiler not initialized \n";
		return;
	}

	double curPoseD[6];
	robot_arm_client_->getCartesianInfo(curPoseD);
	//robot_arm_client_->printCartesianInfo(curPoseD);

	double endPoseD[6];
	std::memcpy(endPoseD, curPoseD, 48);
	for (int i = 0; i < 3; i++) endPoseD[i] += vec3d[i];

	double sync_pose[6];

	if (line_profiler_->device_initialized == false)
		line_profiler_->init();

	line_profiler_->m_vecProfileData.clear();
	double tcp_sync_speed[6];
	robot_arm_client_->setStartPoseXYZ();
	robot_arm_client_->moveHandL(endPoseD, acceleration, speed, false);
	robot_arm_client_->waitTillTCPMove();
	
	line_profiler_->start(20);
	
	robot_arm_client_->getCartesianInfo(sync_pose);
	robot_arm_client_->getTCPSpeed(tcp_sync_speed);
	
	robot_arm_client_->waitTillHandReachDstPose(endPoseD);
	line_profiler_->stop();

	// register point cloud
	cloud->clear();

	int num_profiles = line_profiler_->m_vecProfileData.size();

	double sync_speed = sqrt(tcp_sync_speed[0] * tcp_sync_speed[0] + tcp_sync_speed[1] * tcp_sync_speed[1] + tcp_sync_speed[2] * tcp_sync_speed[2]);

	std::cout <<"tcp sync speed: "<<sync_speed<< "\n";

	std::cout << "num profiles: " << num_profiles << std::endl;

	Eigen::Matrix4f startPose;
	Eigen::Matrix4f endPose;

	array6ToEigenMat4(curPoseD, startPose);
	array6ToEigenMat4(endPoseD, endPose);

	double sync_distance = robot_arm_client_->EuclideanDistance(curPoseD, sync_pose);

	std::cout << "sync distance: " << sync_distance << "\n";

	//std::cout << "calculated end pose\n" << endPose << "\n\n";

	/*double testPoseD[6];
	robot_arm_client_->getCartesianInfo(testPoseD);
	Eigen::Matrix4f testPose;
	array6ToEigenMat4(testPoseD, testPose);

	std::cout << "true end pose\n" << testPose << "\n\n";*/


	// express motion vector in base frame
	Eigen::Vector3f motionVector( endPoseD[0] - curPoseD[0],
							      endPoseD[1] - curPoseD[1],
								  endPoseD[2] - curPoseD[2] );

	//std::cout << "motion vector in base frame:" << motionVector << std::endl;

	// transform motion vector in scanner frame
	/*Eigen::Matrix3f m;
	m = startPose.block<3, 3>(0,0);
	// A'*B' = (BA)' where A and B are rotation matrices. vec_senosr = Te2s*Tb2e*vec_base
	m = m*Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitZ());
	motionVector = m.transpose()*motionVector;*/

	motionVector = (handToScanner_.block<3,3>(0,0)).transpose()*(startPose.block<3, 3>(0, 0)).transpose()*motionVector;

	//std::cout << "motion vector in sensor frame:" << motionVector << std::endl;

	float magnitude = motionVector.norm();

	Eigen::Vector3f motionDelta = motionVector / (num_profiles - 1);

	//std::cout << "motion delta:" << motionDelta << std::endl;

	//std::cout << "x start: " << line_profiler_->m_profileInfo.lXStart << " pitch: " << line_profiler_->m_profileInfo.lXPitch << "\n";

	float distance = 0.f;

	float time = 0.f;

	time = sync_speed / acceleration;

	std::cout << "start time:" << time << "\n";

	time = sqrt(sync_distance * 2 / acceleration);

	std::cout << "start time from dist: " << time << "\n";

	const float start_cruise_time = speed/acceleration;

	const float stop_cruise_time = start_cruise_time + (magnitude -start_cruise_time*start_cruise_time*acceleration)/speed;

	const float cruise_time = stop_cruise_time - start_cruise_time;

	const float total_time = start_cruise_time + stop_cruise_time;

	const float acceleration_total_distance = 0.5f*acceleration*start_cruise_time*start_cruise_time;
	
	motionVector.normalize();

	for (int i = 0; i < num_profiles; i++)
	{
		if (time <= start_cruise_time)
			distance = 0.5f*acceleration*time*time;
		else if (time <= stop_cruise_time)
			distance = acceleration_total_distance + (time - start_cruise_time)*speed;
		else
			distance = magnitude - pow(total_time - time, 2.f)*0.5f*acceleration;

		time += laser_scan_period_;

		// the offset maybe be related to scan speed
		Eigen::Vector3f displacement = motionVector*(distance + speed_correction_);

		for (int j = 10; j < 790; j++)
		{
			PointT point;

			point.z = line_profiler_->m_vecProfileData[i].m_pnProfileData[j] * (-1e-8f);

			if (abs(point.z) < 0.140f)
			{
				point.y = displacement(1);
				point.x = (float)(line_profiler_->m_profileInfo.lXStart + j*line_profiler_->m_profileInfo.lXPitch)*(1e-8f) + displacement(0);
				point.z += 0.3f + displacement(2);

				point.r = ((int)(point.z*10000))%255;//(uint8_t)(255.f - 255.f*(point.z + 0.141f) / 0.282f);
				point.g = point.r;
				point.b = 255;

				cloud->push_back(point);
			}
		}
	}

	std::cout << "point cloud size: " << cloud->size() << std::endl;
}

// assuming already moved to the first pose
void VisionArmCombo::scanLeafWithHyperspectral(std::vector<Eigen::Matrix4d*> & valid_scan_hand_pose_sequence,
												std::vector<ArmConfig> & valid_arm_config_sequence, int plant_id) {

	if (valid_scan_hand_pose_sequence.size() == 0) {

		std::cout << "valid_scan_hand_pose_sequence size 0\n";
		return;
	}

	if (robot_arm_client_ == NULL || hypercam_ == NULL)
	{
		std::cout << "robot arm or line profiler not initialized \n";
		return;
	}

	// reverse valid_scan_hand_pose_sequence if not moving in negative y direction of base frame
	if (valid_scan_hand_pose_sequence.size() > 1) {

		Eigen::Vector3d general_moving_dir = valid_scan_hand_pose_sequence.back()->col(3).head(3)
			- valid_scan_hand_pose_sequence.front()->col(3).head(3);

		if (general_moving_dir(0) > 0) {
			std::reverse(std::begin(valid_scan_hand_pose_sequence), std::end(valid_scan_hand_pose_sequence));
			std::reverse(std::begin(valid_arm_config_sequence), std::end(valid_arm_config_sequence));
			std::cout << "reversed trajectory\n";
		}
	}

	double array6[6];
	std::vector<cv::Mat> hyperspectral_frames;

#ifdef ENABLE_PP
	if (!moveToConfigGetKinectPointCloud(valid_arm_config_sequence[0], false, true, false, IMAGING)) {

		std::cout << "Path to hyperspectral imaging pose not found\n";
		return;
	}
#else
	eigenMat4dToArray6(*valid_scan_hand_pose_sequence[0], array6);
	robot_arm_client_->moveHandL(array6, 0.05, 0.05);
#endif

	//start imaging
	hypercam_->frames_.clear();
	hypercam_->frame_count_ = 0;
	hypercam_->start();

	while (hypercam_->frame_count_ == 0)
		Sleep(2);

	unsigned int start_frame_count = hypercam_->frame_count_;

	std::cout << "start grabing frames\n";

	// collect the first frame
	hyperspectral_frames.push_back(hypercam_->getLastestFrame());

	for (int i = 1; i < valid_scan_hand_pose_sequence.size(); i++) {
		
		eigenMat4dToArray6(*valid_scan_hand_pose_sequence[i], array6);
		robot_arm_client_->moveHandL(array6, 0.05, 0.05);
		Sleep(100);
		hyperspectral_frames.push_back(hypercam_->getLastestFrame());
	}

	unsigned int stop_frame_count = hypercam_->frame_count_;

	std::cout << "finished trajectory\n";

	hypercam_->stop();

	//save data
	std::string time_str = getCurrentDateTimeStr();

	std::string folder(data_saving_folder_.begin(), data_saving_folder_.end());

	std::string camera_pose_file_name = folder + "hypercamera_poses_" + "_plant_" + std::to_string(plant_id)
										+ "_time_" + time_str + ".yml";
	
	cv::FileStorage fs(camera_pose_file_name, cv::FileStorage::WRITE);

	for (int i = 0; i < valid_scan_hand_pose_sequence.size(); i++) {

		

		std::string img_file_name = folder + "hyper_16U_spatial_" + std::to_string(HyperspectralCamera::spatial_size_) 
									+ "_spectral_" + std::to_string(HyperspectralCamera::spectral_size_)
									+ "_plant_" + std::to_string(plant_id)
									+ "_scan_" + std::to_string(i) + "_time_" + time_str + ".bin";

		std::ofstream out(img_file_name, std::ios::out | std::ios::binary);

		out.write((char*)hyperspectral_frames[i].data, sizeof(unsigned short)*HyperspectralCamera::img_size_);

		out.close();

		std::string cam_pose_str = "pose_" + std::to_string(i);

		cv::Mat cam_pose_cv; cam_pose_cv.create(4, 4, CV_64F);

		Eigen::Matrix4d came_pose = (*valid_scan_hand_pose_sequence[i])*hand_to_hyperspectral_.cast<double>();

		EigenTransformToCVTransform(came_pose, cam_pose_cv);

		fs << cam_pose_str << cam_pose_cv;
	}

	fs.release();


	//int num_profiles = hypercam_->frames_.size();

	//std::cout << "num frames: " << num_profiles << std::endl;

	//std::cout << "start frame count: " << start_frame_count << " stop frame count: " << stop_frame_count << std::endl;

	// trim end
	//hypercam_->frames_.erase(hypercam_->frames_.begin() + stop_frame_count - 1, hypercam_->frames_.end());

	// trim beginning
	//hypercam_->frames_.erase(hypercam_->frames_.begin(), hypercam_->frames_.begin() + start_frame_count - 1);

	//std::cout << "after erase num frames: " << hypercam_->frames_.size() << "\n";

	//blue 440 nm 35, green 540 nm 110, red 600 nm 155
	int R_row = 155;
	int G_row = 110;
	int B_row = 35;

	cv::Mat img_r_64, img_g_64, img_b_64, img_64;

	img_64.create(hyperspectral_frames.size(), hypercam_->spatial_size_, CV_64F);
	img_r_64.create(hyperspectral_frames.size(), hypercam_->spatial_size_, CV_64F);
	img_g_64.create(hyperspectral_frames.size(), hypercam_->spatial_size_, CV_64F);
	img_b_64.create(hyperspectral_frames.size(), hypercam_->spatial_size_, CV_64F);


	for (int i = 0; i < hyperspectral_frames.size(); i++) {

		cv::Mat R_channel, G_channel, B_channel;

		hyperspectral_frames[i].row(R_row).convertTo(R_channel, CV_64F);
		hyperspectral_frames[i].row(G_row).convertTo(G_channel, CV_64F);
		hyperspectral_frames[i].row(B_row).convertTo(B_channel, CV_64F);

		std::memcpy(img_r_64.ptr<double>(i), R_channel.ptr<double>(), sizeof(double)*hypercam_->spatial_size_);
		std::memcpy(img_g_64.ptr<double>(i), G_channel.ptr<double>(), sizeof(double)*hypercam_->spatial_size_);
		std::memcpy(img_b_64.ptr<double>(i), B_channel.ptr<double>(), sizeof(double)*hypercam_->spatial_size_);
	}

	double min_val, max_val;
	cv::Mat img_8u, img_r_8u, img_g_8u, img_b_8u;

#if 0	
	img_64.create(hypercam_->frames_.size(), hypercam_->spatial_size_, CV_64F);
	img_r_64.create(hypercam_->frames_.size(), hypercam_->spatial_size_, CV_64F);
	img_g_64.create(hypercam_->frames_.size(), hypercam_->spatial_size_, CV_64F);
	img_b_64.create(hypercam_->frames_.size(), hypercam_->spatial_size_, CV_64F);

	for (int i = 0; i < hypercam_->frames_.size(); i++) {

		std::memcpy(hypercam_->img_.ptr<unsigned char>(), hypercam_->frames_[i].data(), hypercam_->frame_data_size_);

		cv::Mat scan_line;

		cv::reduce(hypercam_->img_, scan_line, 0, CV_REDUCE_AVG, CV_64F);

		std::memcpy(img_64.ptr<double>(i), scan_line.ptr<double>(), sizeof(double)*hypercam_->spatial_size_);

		cv::Mat R_channel, G_channel, B_channel;

		hypercam_->img_.row(R_row).convertTo(R_channel, CV_64F);
		hypercam_->img_.row(G_row).convertTo(G_channel, CV_64F);
		hypercam_->img_.row(B_row).convertTo(B_channel, CV_64F);

		std::memcpy(img_r_64.ptr<double>(i), R_channel.ptr<double>(), sizeof(double)*hypercam_->spatial_size_);
		std::memcpy(img_g_64.ptr<double>(i), G_channel.ptr<double>(), sizeof(double)*hypercam_->spatial_size_);
		std::memcpy(img_b_64.ptr<double>(i), B_channel.ptr<double>(), sizeof(double)*hypercam_->spatial_size_);
	}

	cv::minMaxLoc(img_64, &min_val, &max_val);

	//	min_val = 170.;

	
	img_64.convertTo(img_8u, CV_8U, 255. / (max_val - min_val), -255.*min_val / (max_val - min_val));

	cv::Mat transposed;

	cv::transpose(img_8u, transposed);

	cv::imshow("img", transposed);
	//cv::waitKey(0);
#endif

	//red 
	cv::minMaxLoc(img_r_64, &min_val, &max_val);

	//	min_val = 170.;

	img_r_64.convertTo(img_8u, CV_8U, 255. / (max_val - min_val), -255.*min_val / (max_val - min_val));

	cv::transpose(img_8u, img_r_8u);

	cv::flip(img_r_8u, img_r_8u, 0);

	//green
	cv::minMaxLoc(img_g_64, &min_val, &max_val);

	//	min_val = 170.;

	img_g_64.convertTo(img_8u, CV_8U, 255. / (max_val - min_val), -255.*min_val / (max_val - min_val));

	cv::transpose(img_8u, img_g_8u);

	cv::flip(img_g_8u, img_g_8u, 0);

	//blue 
	cv::minMaxLoc(img_b_64, &min_val, &max_val);

	//	min_val = 170.;

	img_b_64.convertTo(img_8u, CV_8U, 255. / (max_val - min_val), -255.*min_val / (max_val - min_val));

	cv::transpose(img_8u, img_b_8u);

	cv::flip(img_b_8u, img_b_8u, 0);

	std::vector<cv::Mat> bgr_vec;
	bgr_vec.push_back(img_b_8u);
	bgr_vec.push_back(img_g_8u);
	bgr_vec.push_back(img_r_8u);

	cv::Mat color;
	cv::merge(bgr_vec, color);

	cv::namedWindow("color", cv::WINDOW_NORMAL);// Create a window for display.
	
	cv::imshow("color", color);
	cv::waitKey(view_time_);

	//hypercam_->frames_.clear();
}

/*
assume robot arm already at start pose
vec3d: motion vector in base frame
cloud: PCL point cloud to save the data
*/
void VisionArmCombo::scanTranslateOnlyHyperspectral(double * vec3d, float acceleration, float speed)
{
	if (acceleration == 0 || speed == 0)
	{
		std::cout << "acceleration or speed = 0\n";
		return;
	}

	if (robot_arm_client_ == NULL || hypercam_ == NULL)
	{
		std::cout << "robot arm or line profiler not initialized \n";
		return;
	}

	//start imaging
	hypercam_->frames_.clear();
	hypercam_->frame_count_ = 0;
	hypercam_->start();


	double curPoseD[6];
	robot_arm_client_->getCartesianInfo(curPoseD);
	//robot_arm_client_->printCartesianInfo(curPoseD);

	double endPoseD[6];
	std::memcpy(endPoseD, curPoseD, 48);
	for (int i = 0; i < 3; i++) endPoseD[i] += vec3d[i];

	double sync_pose[6];
	double tcp_sync_speed[6];

	while (hypercam_->frame_count_ == 0) 
		Sleep(2);

	robot_arm_client_->setStartPoseXYZ();
	robot_arm_client_->moveHandL(endPoseD, acceleration, speed, false);
	robot_arm_client_->waitTillTCPMove();

	unsigned int start_frame_count = hypercam_->frame_count_;

	robot_arm_client_->getCartesianInfo(sync_pose);
	robot_arm_client_->getTCPSpeed(tcp_sync_speed);

	robot_arm_client_->waitTillHandReachDstPose(endPoseD);

	unsigned int stop_frame_count = hypercam_->frame_count_;

	hypercam_->stop();

	int num_profiles = hypercam_->frames_.size();

	std::cout << "num frames: " << num_profiles << std::endl;

	std::cout << "start frame count: " << start_frame_count << " stop frame count: "<<stop_frame_count<<std::endl;

	// trim end
	hypercam_->frames_.erase(hypercam_->frames_.begin() + stop_frame_count - 1, hypercam_->frames_.end());

	// trim beginning
	hypercam_->frames_.erase(hypercam_->frames_.begin(), hypercam_->frames_.begin() + start_frame_count - 1);

	std::cout << "after erase num frames: " << hypercam_->frames_.size() << "\n";

	double sync_speed = sqrt(tcp_sync_speed[0] * tcp_sync_speed[0] + tcp_sync_speed[1] * tcp_sync_speed[1] + tcp_sync_speed[2] * tcp_sync_speed[2]);

	std::cout << "tcp sync speed: " << sync_speed << "\n";

	cv::Mat img_64;
	img_64.create(hypercam_->frames_.size(), hypercam_->spatial_size_, CV_64F);

	//blue 440 nm 35, green 540 nm 110, red 600 nm 155
	int R_row = 155;
	int G_row = 110;
	int B_row = 35;

	cv::Mat img_r_64, img_g_64, img_b_64;
	img_r_64.create(hypercam_->frames_.size(), hypercam_->spatial_size_, CV_64F);
	img_g_64.create(hypercam_->frames_.size(), hypercam_->spatial_size_, CV_64F);
	img_b_64.create(hypercam_->frames_.size(), hypercam_->spatial_size_, CV_64F);

	for (int i = 0; i < hypercam_->frames_.size(); i++) {
		
		std::memcpy(hypercam_->img_.ptr<unsigned char>(), hypercam_->frames_[i].data(), hypercam_->frame_data_size_);

		cv::Mat scan_line;
		
		cv::reduce(hypercam_->img_, scan_line, 0, CV_REDUCE_AVG, CV_64F);

		std::memcpy(img_64.ptr<double>(i), scan_line.ptr<double>(), sizeof(double)*hypercam_->spatial_size_);

		cv::Mat R_channel, G_channel, B_channel;

		hypercam_->img_.row(R_row).convertTo(R_channel, CV_64F);
		hypercam_->img_.row(G_row).convertTo(G_channel, CV_64F);
		hypercam_->img_.row(B_row).convertTo(B_channel, CV_64F);

		std::memcpy(img_r_64.ptr<double>(i), R_channel.ptr<double>(), sizeof(double)*hypercam_->spatial_size_);
		std::memcpy(img_g_64.ptr<double>(i), G_channel.ptr<double>(), sizeof(double)*hypercam_->spatial_size_);
		std::memcpy(img_b_64.ptr<double>(i), B_channel.ptr<double>(), sizeof(double)*hypercam_->spatial_size_);
	}

	double min_val, max_val;

	cv::minMaxLoc(img_64, &min_val, &max_val);

	//	min_val = 170.;

	cv::Mat img_8u, img_r_8u, img_g_8u, img_b_8u;;
	img_64.convertTo(img_8u, CV_8U, 255. / (max_val - min_val), -255.*min_val / (max_val - min_val));

	cv::Mat transposed;

	cv::transpose(img_8u, transposed);

	cv::imshow("img", transposed);
	
	//red 
	cv::minMaxLoc(img_r_64, &min_val, &max_val);

	//	min_val = 170.;

	img_r_64.convertTo(img_8u, CV_8U, 255. / (max_val - min_val), -255.*min_val / (max_val - min_val));

	cv::transpose(img_8u, img_r_8u);

	//green
	cv::minMaxLoc(img_g_64, &min_val, &max_val);

	//	min_val = 170.;

	img_g_64.convertTo(img_8u, CV_8U, 255. / (max_val - min_val), -255.*min_val / (max_val - min_val));

	cv::transpose(img_8u, img_g_8u);

	//blue 
	cv::minMaxLoc(img_b_64, &min_val, &max_val);

	//	min_val = 170.;

	img_b_64.convertTo(img_8u, CV_8U, 255. / (max_val - min_val), -255.*min_val / (max_val - min_val));

	cv::transpose(img_8u, img_b_8u);

	std::vector<cv::Mat> bgr_vec;
	bgr_vec.push_back(img_b_8u);
	bgr_vec.push_back(img_g_8u);
	bgr_vec.push_back(img_r_8u);

	cv::Mat color;
	cv::merge(bgr_vec, color);

	cv::Mat stretch_color;

	cv::resize(color, stretch_color, cv::Size(), 10., 1.);

	cv::imshow("color", stretch_color);
	cv::waitKey(0);

	hypercam_->frames_.clear();

}

void VisionArmCombo::scanLine(PointCloudT::Ptr & cloud)
{
	if (robot_arm_client_ == NULL || line_profiler_ == NULL)
	{
		std::cout << "robot arm or line profiler not initialized \n";
		return;
	}

	if (line_profiler_->device_initialized == false)
		line_profiler_->init();

	double curPoseD[6];
	robot_arm_client_->getCartesianInfo(curPoseD);
	robot_arm_client_->printCartesianInfo(curPoseD);

	//!before start scan,  must clear old data
	line_profiler_->m_vecProfileData.clear();
	line_profiler_->start(10);
	line_profiler_->stop();

	cloud->clear();

	int num_profiles = line_profiler_->m_vecProfileData.size();

	std::cout << "num profiles: " << num_profiles << std::endl;

	//for (int i = 0; i < num_profiles; i++)
	int i = num_profiles / 2;
	{
		for (int j = 10; j < 790; j++)
		{
			PointT point;

			point.z = line_profiler_->m_vecProfileData[i].m_pnProfileData[j] * (-1e-8f);

			if (abs(point.z) < 0.140f)
			{
				point.z += 0.3f;
				point.y = 0.f;
				point.x = (float)(line_profiler_->m_profileInfo.lXStart + j*line_profiler_->m_profileInfo.lXPitch)*(1e-8f);

				point.r = (uint8_t)(255.f - 255.f*(point.z + 0.14f) / 0.28f);
				point.g = point.r;
				point.b = 255;

				cloud->push_back(point);
			}
		}
	}

	std::cout << "point cloud size: " << cloud->size() << std::endl;
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
				//std::cout << "file name: " << front_part << "\n";
				
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

	GetLocalTime(&st);

	//GetSystemTime(&st);

	char currentTime[84] = "";

	std::sprintf(currentTime, "%d-%d-%d-%d-%d-%d-%d", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);

	return std::string(currentTime);
}




/*
	register point cloud from kinect in robot base frame
*/

void VisionArmCombo::mapWorkspaceUsingKinectArm(int rover_position, int num_plants)
{

	//pcl::io::loadPCDFile("laser_scan.pcd", *kinect_cloud_);

	//extractLeafProbingPoints(kinect_cloud_);

	//return;

	//pp_.PRMCEPreprocessing(); pp_.savePathPlanner("pp"); for (int i=1; i<pp_.num_nodes_; i+=2) viewPlannedPath(pp_.random_nodes_buffer_+(i-1)*6, pp_.random_nodes_buffer_+i*6); return;

	growthChamberPointCloudVec_[rover_position]->clear();

#ifdef ENABLE_PP
	pp_.resetOccupancyGrid();
#endif

	kinect_cloud_->clear();

#if 0
	PointCloudT::Ptr window_left_panel(new PointCloudT);

	for (float x = 0.44f; x < 0.85f; x += 0.01f)
	{
		for (float y = -0.3f; y < -0.21f; y += 0.01f)
		{
			for (float z = -0.44f; z < 1.14f; z += 0.01f)
			{
				PointT p;
				p.x = x; p.y = y; p.z = z;
				p.r = p.g = p.b = 255;
				window_left_panel->push_back(p);
			}
		}
	}

	pp_.addPointCloudToOccupancyGrid(window_left_panel);
	showOccupancyGrid();
	display();
#endif

	int scan_count = 0;

	ArmConfig mapping_config;
	//mapping_config.setJointPos(-90, -90, -90, -90, 90, -180);
	mapping_config.setJointPos(-87.42, -73.57, -114.13, -82.3, 90, -177.45);
	mapping_config.toRad();
	std::cout << "mapping scene\n";
	scanGrowthChamberWithKinect(rover_position, mapping_config, true);

	std::cout << "showing occupancy grid\n";
	showOccupancyGrid();

#if 0
	if (rover_position == 1)	//center
	{
		// left
		mapping_config.setJointPos(-42, -105, -71, -90, 84, -132);
		mapping_config.toRad();

		moveToConfigGetKinectPointCloud(mapping_config, true, true, true);

		//right
		mapping_config.setJointPos(-124, -115, -57, -91, 93, -214);
		mapping_config.toRad();

		moveToConfigGetKinectPointCloud(mapping_config, true, true, true);
	}
#endif

#if 0 
	std::cout << "acquire Kinect data...\n";
	for (int i = 0; i < imaging_config_vec.size(); i++)
	{
		moveToConfigGetKinectPointCloud(imaging_config_vec[i], true, true, true);
	}
#endif

	// no data
	if (kinect_cloud_->points.size() < 100) return;

	*growthChamberPointCloudVec_[rover_position] += *kinect_cloud_;

	if (rover_position != 1)
	{
		// initial guess transforn
		PointCloudT::Ptr initial_transformed_pc(new PointCloudT);

		Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();
		initial_transform(0, 3) = rover_position == 0 ? -rover_dist_ : rover_dist_;

		pcl::transformPointCloud(*growthChamberPointCloudVec_[1], *initial_transformed_pc, initial_transform);

		viewer_->removeAllPointClouds();

		std::cout << "initial alignment\n";
		viewer_->addPointCloud(kinect_cloud_, "new pc", 0);
		viewer_->addPointCloud(initial_transformed_pc, "initial transform", 0);
		display();

		std::cout << "ICP\n";
		pcl::IterativeClosestPoint<PointT, PointT> icp;
		icp.setInputSource(initial_transformed_pc);
		icp.setInputTarget(kinect_cloud_);
		PointCloudT::Ptr final_transformed_pc(new PointCloudT);
		icp.align(*final_transformed_pc);

		for (auto & p : final_transformed_pc->points) p.r = 255;

		viewer_->removeAllPointClouds();

		std::cout << "final alignment\n";
		viewer_->addPointCloud(kinect_cloud_, "right pc", 0);
		viewer_->addPointCloud(final_transformed_pc, "final_transformed_pc", 0);

		icp_final_transformation_ = icp.getFinalTransformation();
		std::cout << icp_final_transformation_ << std::endl;

		display();

		//combine current kinect cloud with the cloud in the middle position
		*final_transformed_pc += *kinect_cloud_;

		// voxel grid donwsample
		vox_.setInputCloud(final_transformed_pc);
		vox_.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);

		vox_.filter(*kinect_cloud_);

#ifdef ENABLE_PP
		pp_.addPointCloudToOccupancyGrid(kinect_cloud_);
#endif
		std::cout << "showing occupancy grid\n";
		showOccupancyGrid();
		
	}
	//return;

	//if (rover_position == 1)
	{
		//const float shelf_z = -0.52f;
		processGrowthChamberEnviroment(kinect_cloud_, shelf_z_, num_plants, rover_position);
	}
#if 0
	else 
	{
		//right position, pot x value > 0
		for (int i = 0; i < object_centers_.rows; i++)
		{
			if (rover_position == 0 /*&& pot_process_status_[i] == false*/ && object_centers_.at<cv::Vec3f>(i, 0)[0] > scan_pot_x_abs_limit_)
			{
				//transform pot center location to current view
				Eigen::Vector4f old_center(object_centers_.at<cv::Vec3f>(i, 0)[0] - rover_dist_,
										object_centers_.at<cv::Vec3f>(i, 0)[1],
										object_centers_.at<cv::Vec3f>(i, 0)[2],
										1.0f);

				Eigen::Vector4f new_center =  icp_final_transformation*old_center;

				cv::Vec3f new_object_center;

				new_object_center[0] = new_center(0);
				new_object_center[1] = new_center(1);
				new_object_center[2] = new_center(2);

				pcl::ModelCoefficients coeffs;
				coeffs.values.resize(4);
				coeffs.values[0] = new_center(0);
				coeffs.values[1] = new_center(1);
				coeffs.values[2] = -0.7f;// new_center(2);
				coeffs.values[3] = 0.15f;

				std::cout << new_center << "\n";

				viewer_->addSphere(coeffs, "sphere" + std::to_string(i), 0);

				display();

			//	if (!scanPlantCluster(new_object_center, i)) std::cout << "scan objecy " << i << " fail\n";

				//pot_process_status_[i] = true;
			}
		}
	}
#endif

}

/*
	iteractive calibration process
*/
void VisionArmCombo::testLineScannerProbeCalibrationMatrix()
{
	if(robot_arm_client_ == NULL) initRobotArmClient();

	if(line_profiler_ == NULL) initLineProfiler();

	PointCloudT::Ptr cloud(new PointCloudT);

	double pos[6];

	pcl::KdTreeFLANN<PointT> kdtree;

	while (true)
	{
		std::cout << "Move robot hand then press Enter to scan or 'q'+Enter to close\n";

		std::getchar();

		PointCloudT::Ptr tmp_cloud(new PointCloudT);

		scanLine(tmp_cloud);

		
		Eigen::Matrix4f base2hand;
		robot_arm_client_->getCartesianInfo(pos);
		array6ToEigenMat4(pos, base2hand);
		
		Eigen::Matrix4f base2scanner = base2hand*handToScanner_;

		PointCloudT::Ptr tmp_cloud_base(new PointCloudT);
		pcl::transformPointCloud(*tmp_cloud, *tmp_cloud_base, base2scanner);

		*cloud += *tmp_cloud_base;
		
		viewer_->removeAllPointClouds();

		viewer_->addPointCloud(cloud, "cloud", 0);

		display();

		std::cout << "move hand?\n";

		std::string key;
		std::getline(std::cin, key);

		if (key == "y")
		{
			Eigen::Vector3f trans = pre_point_ - base2hand.topLeftCorner<3, 3>()*tool_center_point_;

			pos[0] = trans(0); pos[1] = trans(1); pos[2] = trans(2);

			robot_arm_client_->moveHandL(pos, 0.05, 0.05);
		}
	}

	robot_arm_client_->stopRecvTCP();
	line_profiler_->stop();
	line_profiler_->finalize();
}

void VisionArmCombo::acquireLinesOnPlanes()
{
	if(robot_arm_client_ == NULL) initRobotArmClient();

	if(line_profiler_ == NULL) initLineProfiler();

	int line_count = 0;
	int plane_count = 0;
	int num_lines_per_plane = 6;

	while (true)
	{
		std::cout << "Move robot hand to plane id " <<plane_count<<" then press Enter to scan on the plane or 'q'+Enter to close\n";

		char key = std::getchar();

		if (key == 'q')
			break;

		//std::vector<PointCloudT::Ptr> line_vec;

		for (int i = 0; i < num_lines_per_plane; i++)
		{
			std::cout << "	press Enter to scan line "<<i<<" on plane " <<plane_count<<"\n";

			char key = std::getchar();

			std::cout << "scan line id " << line_count << "\n";

			PointCloudT::Ptr point_cloud(new PointCloudT);

			scanLine(point_cloud);

			// save data
			std::string filename = getCurrentDateTimeStr() + ".pcd";

			pcl::io::savePCDFile(filename, *point_cloud, true);

			result_file_.open("ScannerRobotArmCalibrationFile.txt", std::ios::app);

			double curPoseD[6];

			robot_arm_client_->getCartesianInfo(curPoseD);

			Eigen::Matrix4d mat4d;
			array6ToEigenMat4d(curPoseD, mat4d);
			std::cout << "pose:\n" << mat4d << "\n";

			char buffer[200];

			sprintf(buffer, "%.16e,%.16e,%.16e,%.16e,%.16e,%.16e", curPoseD[0], curPoseD[1], curPoseD[2], curPoseD[3], curPoseD[4], curPoseD[5]);

			std::string startPoseStr(buffer);

			result_file_ << filename << "," << startPoseStr << "," << plane_count << "," << line_count << std::endl;

			result_file_.close();

			viewer_->removeAllPointClouds();
			viewer_->addPointCloud(point_cloud);
			display();

			//line_vec.push_back(point_cloud);

			line_count++;
		}

		plane_count++;
	}
}

/*
Solve scanner pose wrt robot hand assuming normalDistanceVec populated.
Carlson, F. B., Johansson, R., & Robertsson, A. (2015, September).
Six DOF eye-to-hand calibration from 2D measurements using planar constraints.
In Intelligent Robots and Systems (IROS), 2015 IEEE/RSJ International Conference on (pp. 3628-3632). IEEE.
*/
void VisionArmCombo::lineScannerHandEyeCalibration(int num_lines_per_plane)
{
	Eigen::Matrix4d curHandToScanner_ = guessHandToScanner_.cast<double>();

	std::cout << "initial guess:\n" << curHandToScanner_ << "\n";

	int rows = 0; for (auto c : calibration_point_cloud_vec) rows += c->points.size();

	Eigen::MatrixXd A(rows, 9);
	Eigen::VectorXd Y(rows);
	Eigen::VectorXd w(6);
	double min_rms = 1e5;
	std::cout << "A rows: " << rows << "\n";

	ofstream myfile;
	myfile.open("points 2 planes distances.txt");

	ofstream myfile1;
	myfile1.open("RMS.txt");

	int iteration = 10;
	

	for (int iter = 0; iter <= iteration; iter++)
	{
		std::cout << "\niteration " << iter << "\n";
		double rms = 0.;
		plane_normal_embed_dist_vec.clear();
		double distance_error = 0.0;

		// compute plane normal
		for (int i = 0; i < calibration_point_cloud_vec.size(); i += num_lines_per_plane)
		{
			PointCloudT::Ptr plane_cloud(new PointCloudT);

			for (int j = 0; j < num_lines_per_plane; j++)
			{
				Eigen::Matrix4f base_to_scanner;
				base_to_scanner = hand_pose_vec_[i + j]->cast<float>()*curHandToScanner_.cast<float>();
				PointCloudT::Ptr tmp_cloud(new PointCloudT);
				pcl::transformPointCloud(*calibration_point_cloud_vec[i + j], *tmp_cloud, base_to_scanner);
				*plane_cloud += *tmp_cloud;
			}

			/*viewer_->removeAllPointClouds();
			viewer_->addPointCloud(plane_cloud);
			display();*/

			//Eigen::Matrix3d covar_mat = Eigen::Matrix3d::Identity();
			//double xmean = 0.; double ymean = 0.; double zmean = 0.;
			////compute mean
			//for (int j = 0; j < plane_cloud->points.size(); j++)
			//{
			//	xmean += plane_cloud->points[j].x;
			//	ymean += plane_cloud->points[j].y;
			//	zmean += plane_cloud->points[j].z;
			//}

			//xmean /= plane_cloud->points.size();
			//ymean /= plane_cloud->points.size();
			//zmean /= plane_cloud->points.size();

			//double xy_covar = 0.; double xz_covar = 0.; double yz_covar = 0.;
			//double xx_covar = 0.; double yy_covar = 0.; double zz_covar = 0.;
			//for (int j = 0; j < plane_cloud->points.size(); j++)
			//{
			//	xy_covar += (plane_cloud->points[j].x - xmean)*(plane_cloud->points[j].y - ymean);
			//	xz_covar += (plane_cloud->points[j].x - xmean)*(plane_cloud->points[j].z - zmean);
			//	yz_covar += (plane_cloud->points[j].z - zmean)*(plane_cloud->points[j].y - ymean);

			//	xx_covar += (plane_cloud->points[j].x - xmean)*(plane_cloud->points[j].x - xmean);
			//	yy_covar += (plane_cloud->points[j].y - ymean)*(plane_cloud->points[j].y - ymean);
			//	zz_covar += (plane_cloud->points[j].z - zmean)*(plane_cloud->points[j].z - zmean);
			//}

			//covar_mat.diagonal() << xx_covar, yy_covar, zz_covar;
			//covar_mat(0, 1) = xy_covar; covar_mat(0, 2) = xz_covar; covar_mat(1, 2) = yz_covar;
			//covar_mat(1, 0) = xy_covar; covar_mat(2, 0) = xz_covar; covar_mat(2, 1) = yz_covar;

			//Eigen::Vector3d centroid_d(xmean, ymean, zmean);

			//Eigen::EigenSolver<Eigen::Matrix3d> es(covar_mat);

			//int min_idx = 0;
			//double min_eigen = es.eigenvalues()[0].real();
			//for (int j = 1; j < 3; j++)
			//{
			//	if (es.eigenvalues()[j].real() < min_eigen)
			//	{
			//		min_idx = j; min_eigen = es.eigenvalues()[j].real();
			//	}
			//}

			//Eigen::Vector3d min_eigen_vector = es.eigenvectors().col(min_idx).real();


			// estimate normal
			pcl::PCA<PointT> pca(*plane_cloud);

			Eigen::Vector4d centroid = pca.getMean().cast<double>();

			//std::cout << "centroid \n" << centroid << "\n";

			Eigen::Vector3d eigen_values = pca.getEigenValues().cast<double>();
			//std::cout << "eigen values \n" << eigen_values << "\n";

			distance_error += eigen_values(2);	//sum of squared distance to plane

			myfile << std::sqrt(eigen_values(2)/plane_cloud->points.size())<<",";	//RMS

			Eigen::Matrix3d eigen_vectors = pca.getEigenVectors().cast<double>();

			//std::cout << "eigen vector \n" << eigen_vectors << "\n";

			// n in the paper
			Eigen::Vector3d* plane_normal_embed_dist = new Eigen::Vector3d;

			(*plane_normal_embed_dist) = eigen_vectors.col(2)*(eigen_vectors.col(2).dot(centroid.head<3>()));
			// try my own eigen decompo
			//(*plane_normal_embed_dist) = min_eigen_vector*(min_eigen_vector.dot(centroid_d));

			plane_normal_embed_dist_vec.push_back(plane_normal_embed_dist);

			

		/*	pcl::ModelCoefficients line;
			line.values.resize(6);
			line.values[0] = 0.f; line.values[1] = 0.f; line.values[2] = 0.f;
			line.values[3] = (*plane_normal_embed_dist)(0); line.values[4] = (*plane_normal_embed_dist)(1);
			line.values[5] = (*plane_normal_embed_dist)(2);

			viewer_->removeAllShapes();
			std::string name = "line";
			viewer_->addLine(line, name, 0);
			viewer_->removeAllPointClouds();
			viewer_->addPointCloud(plane_cloud);
			display();*/
		}

		myfile << "\n";

		distance_error = std::sqrt(distance_error / rows);
		myfile1 << distance_error << "\n";
		std::cout << "distance error" << distance_error << "\n";

		// compute A and Y
		int row_idx = 0; rms = 0.;
		for (int i = 0; i < calibration_point_cloud_vec.size(); i += num_lines_per_plane)
		{
			Eigen::Vector3d n = *plane_normal_embed_dist_vec[i / num_lines_per_plane];

			for (int j = 0; j < num_lines_per_plane; j++)
			{
				PointCloudT::Ptr cloud = calibration_point_cloud_vec[i + j];

				Eigen::RowVector3d nR = n.transpose()*hand_pose_vec_[i + j]->topLeftCorner<3, 3>();

				double np = n.dot(hand_pose_vec_[i + j]->topRightCorner<3,1>());

				for (auto & ps : cloud->points)
				{
					A.block<1, 3>(row_idx, 0) = nR*(double)ps.x;
					A.block<1, 3>(row_idx, 3) = nR*(double)ps.z;
					A.block<1, 3>(row_idx, 6) = nR;
					Y(row_idx++) = n.dot(n) - np;
				}
			}
		}

		w.head<3>() = curHandToScanner_.topLeftCorner<3, 1>(); w.segment<3>(3) = curHandToScanner_.col(2).head(3);
		w.tail<3>() = curHandToScanner_.col(3).head(3);
		rms = sqrt((A*w - Y).squaredNorm() / rows);
		std::cout << "Before RMS: " <<  rms << "\n";

		// solve w using SVD
		w = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
		
		std::cout << "w:\n" << w.transpose() << "\n";

		Eigen::Matrix3d tmp_rot;
		Eigen::Vector3d rx = w.head<3>();
		Eigen::Vector3d rz = w.segment<3>(3);
		tmp_rot.col(0) = rx;
		tmp_rot.col(1) = rz.cross(rx);
		tmp_rot.col(2) = rz;

		// Orthogonalize rotational matrix
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(tmp_rot, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV().transpose();
		Eigen::Matrix3d tmp = Eigen::Matrix3d::Identity();
		tmp(2, 2) = (U*V).determinant();
		Eigen::Matrix3d rot_hand_to_sensor = U*tmp*V;

		Eigen::MatrixXd A16 = A.topLeftCorner(rows, 6);
		Eigen::MatrixXd A79 = A.topRightCorner(rows, 3);

		Eigen::VectorXd R_tilde_star(6);
		R_tilde_star.head<3>() = rot_hand_to_sensor.col(0);
		R_tilde_star.tail<3>() = rot_hand_to_sensor.col(2);
		
		Eigen::VectorXd Y_tilde = Y - A16*R_tilde_star;

		Eigen::Vector3d tran_hand_to_sensor = A79.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y_tilde);

		curHandToScanner_ = Eigen::Matrix4d::Identity();
		curHandToScanner_.topLeftCorner<3, 3>() = rot_hand_to_sensor;
		curHandToScanner_.topRightCorner<3, 1>() = tran_hand_to_sensor;

		w.head<6>() = R_tilde_star; w.tail<3>() = curHandToScanner_.topRightCorner<3, 1>();

		double rms_after = sqrt((A*w - Y).squaredNorm() / rows);

		std::cout << "After RMS: " << rms_after << "\n";

	//	if (rms_after < min_rms)
		{
			min_rms = rms_after;
			handToScanner_ = curHandToScanner_.cast<float>();
		}

		std::cout << "curHandToScanner:\n" << curHandToScanner_ << "\n";
	}

	double average_distance = 0.0f;
	// points to plane distances
	for (int i = 0; i < calibration_point_cloud_vec.size(); i += num_lines_per_plane)
	{
		PointCloudT::Ptr plane_cloud(new PointCloudT);

		for (int j = 0; j < num_lines_per_plane; j++)
		{
			Eigen::Matrix4f base_to_scanner;
			base_to_scanner = hand_pose_vec_[i + j]->cast<float>()*curHandToScanner_.cast<float>();
			PointCloudT::Ptr tmp_cloud(new PointCloudT);
			pcl::transformPointCloud(*calibration_point_cloud_vec[i + j], *tmp_cloud, base_to_scanner);
			*plane_cloud += *tmp_cloud;
		}



		// estimate normal
		pcl::PCA<PointT> pca(*plane_cloud);

		Eigen::Vector4d centroid = pca.getMean().cast<double>();
		//std::cout << "centroid \n" << centroid << "\n";

		Eigen::Vector3d eigen_values = pca.getEigenValues().cast<double>();
		std::cout << "eigen values \n" << eigen_values << "\n";

		Eigen::Matrix3d eigen_vectors = pca.getEigenVectors().cast<double>();

		std::cout << "eigen vector \n" << eigen_vectors << "\n";

		// n in the paper
		double d = eigen_vectors.col(2).dot(centroid.head<3>());

		double sign = acos(d / centroid.head<3>().norm()) > 0. ? 1.0 : -1.0;

		double sum_squared_d = 0.0;

		for (int k = 0; k < plane_cloud->points.size(); k++)
		{
			Eigen::Vector3d p; 
			p << plane_cloud->points[k].x, plane_cloud->points[k].y, plane_cloud->points[k].z;
			
			double point_to_plane_dist = eigen_vectors.col(2).dot(p) - sign*d;
			//myfile << point_to_plane_dist<<"\n";
			sum_squared_d += pow(point_to_plane_dist, 2);

			average_distance += point_to_plane_dist;
		}

		std::cout << "sum_squared_d: " << sum_squared_d << "\n";

		/*PointT center;
		center.x = centroid(0); center.y = centroid(1); center.z = centroid(2);
		center.r = 255; center.g = 0; center.b = 0;
		plane_cloud->points.push_back(center);
		viewer_->removeAllPointClouds();
		viewer_->addPointCloud(plane_cloud);
		display();*/
	}

	std::cout << "average signed distance: " << average_distance << "\n";
	std::cout << "\nmin RMS: " << min_rms << "\n final handToScanner:\n" << handToScanner_ << "\n";

	std::ofstream file("lineScannerHandEyeCalibration.bin", std::ios::out | std::ios::binary | std::ios::trunc);
	if (file.is_open())
	{
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				file.write((char*)&handToScanner_.row(i)(j), sizeof(float));
	}
	else
	{
		std::cout << "save line scanner hand eye calibration fail\n" ;
	}

	file.close();

	myfile.close();
	myfile1.close();
}

void VisionArmCombo::addArmModelToViewer(std::vector<PathPlanner::RefPoint>& ref_points)
{
	viewer_->removeAllShapes();

	for (int i = 0; i < ref_points.size()-1; i++)
	{
		pcl::ModelCoefficients cylinder_coeff;
		cylinder_coeff.values.resize(7);

		PointT p; p.x = ref_points[i].coordinates[0]; p.y = ref_points[i].coordinates[1]; p.z = ref_points[i].coordinates[2];
		PointT p1; p1.x = ref_points[i + 1].coordinates[0]; p1.y = ref_points[i + 1].coordinates[1]; p1.z = ref_points[i + 1].coordinates[2];

		cylinder_coeff.values[0] = p.x; cylinder_coeff.values[1] = p.y; cylinder_coeff.values[2] = p.z;
		cylinder_coeff.values[3] = p1.x - p.x; cylinder_coeff.values[4] = p1.y - p.y; cylinder_coeff.values[5] = p1.z - p.z;
		cylinder_coeff.values[6] = pp_.arm_radius_lookup[i];

		viewer_->addCylinder(cylinder_coeff, "cylinder" + std::to_string(i), 0);
	}

	display();;
}

void VisionArmCombo::addOBBArmModelToViewer(std::vector<PathPlanner::OBB> & arm_obbs)
{
	viewer_->removeAllShapes();
	viewer_->removeAllCoordinateSystems();
	viewer_->addCoordinateSystem(0.3, "world", 0);

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

		viewer_->addCube(cube_coeff, "cube"+std::to_string(j), 0);

		Eigen::Affine3f transform;
		Eigen::Matrix4f temp_mat = Eigen::Matrix4f::Identity();
		temp_mat.block<3, 3>(0, 0) = arm_obbs[j].A;;
		temp_mat.block<3, 1>(0, 3) << arm_obbs[j].C(0), arm_obbs[j].C(1), arm_obbs[j].C(2);
		transform.matrix() = temp_mat;

		//if(j>0) std::cout << "obb collision of " <<j-1<<" and "<<j<<" = " << pp_.collisionOBB(arm_obbs[j-1], arm_obbs[j]) <<"\n";	
		//viewer_->addCoordinateSystem(0.2, transform, "co"+std::to_string(j), 0);
	}

	display();
}

void VisionArmCombo::showOccupancyGrid(bool spin)
{
	if (!pp_.path_planner_ready_)	return;

	PointCloudT::Ptr cloud(new PointCloudT);

	for (int z = 0; z < pp_.grid_height_; z++)
	{
		for (int y = 0; y < pp_.grid_depth_; y++)
		{
			for (int x = 0; x < pp_.grid_width_; x++)
			{
				int cell_idx = x + y*pp_.grid_width_ + z*pp_.grid_depth_*pp_.grid_width_;

				PathPlanner::Cell cell = pp_.grid_[cell_idx];

				if (cell.isOccupiedCounter == pp_.prmce_round_counter_ )
				{
					PointT p;
					p.x = ((-x)*pp_.cell_size_ + pp_.grid_offset_x_)*0.01f; 
					p.y = ((-y)*pp_.cell_size_ + pp_.grid_offset_y_)*0.01f; 
					p.z = ((z)*pp_.cell_size_ + pp_.grid_offset_z_)*0.01f;
					p.r = p.g = p.b = 255;
					cloud->points.push_back(p);
				}

				if (cell.sweptVolumneCounter == pp_.prmce_swept_volume_counter_)
				{
					PointT p;
					p.x = ((-x)*pp_.cell_size_ + pp_.grid_offset_x_)*0.01f;
					p.y = ((-y)*pp_.cell_size_ + pp_.grid_offset_y_)*0.01f;
					p.z = ((z)*pp_.cell_size_ + pp_.grid_offset_z_)*0.01f;
					p.r = 255; p.g = p.b = 0;
					cloud->points.push_back(p);
				}
			}
		}
	}

	std::cout << "vox cloud size: " << cloud->points.size() << "\n";

	viewer_->removePointCloud("grid");
	viewer_->addPointCloud(cloud, "grid");

	if(spin)
		display();
}

void VisionArmCombo::viewPlannedPath(float* start_pose, float* goal_pose)
{
	std::vector<PathPlanner::RefPoint> ref_points;
	std::vector<Eigen::Matrix3f> rot_mats;
	std::vector<PathPlanner::OBB> arm_obbs;

	std::cout << "start pose\n";

	// visualization
	pp_.computeReferencePointsOnArm(start_pose, ref_points, rot_mats);
	pp_.getArmOBBModel(ref_points, rot_mats, arm_obbs); addOBBArmModelToViewer(arm_obbs);

	for (auto index : pp_.shortest_path_index_vec_)
	{
		float config[6];

		memcpy(config, pp_.random_nodes_buffer_ + index * pp_.num_joints_, pp_.num_joints_ * sizeof(float));

		pp_.computeReferencePointsOnArm(config, ref_points, rot_mats);
		pp_.getArmOBBModel(ref_points, rot_mats, arm_obbs); addOBBArmModelToViewer(arm_obbs);
	}

	std::cout << "goal pose\n";

	pp_.computeReferencePointsOnArm(goal_pose, ref_points, rot_mats);
	pp_.getArmOBBModel(ref_points, rot_mats, arm_obbs); addOBBArmModelToViewer(arm_obbs);

#if 0
	std::vector<PathPlanner::prmceedge_descriptor> edge_vec; 
	pp_.sweptVolume(start_pose, goal_pose, edge_vec);
	pp_.viewOccupancyGrid(viewer_);
	pp_.prmce_swept_volume_counter_++;
	pp_.prmce_round_counter_++;
	viewer_->removeAllPointClouds();
#endif

}


/*
	https://github.com/ros-industrial/universal_robot/blob/indigo-devel/ur_kinematics/src/ur_kin.cpp
	Analytical solutions + picking the feasible one
*/
int VisionArmCombo::inverseKinematics(Eigen::Matrix4d & T, std::vector<int> & ik_sols_vec, int imaging_or_probing)
{
	ik_sols_vec.clear();
	const double q6_des = -PI;
	int num_sols = 0;
	/*double T02 = -*T; T++; double T00 = *T; T++; double T01 = *T; T++; double T03 = -*T; T++;
	double T12 = -*T; T++; double T10 = *T; T++; double T11 = *T; T++; double T13 = -*T; T++;
	double T22 = *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 = *T;*/
	double T00 = T(0, 0); double T01 = T(0, 1); double T02 = T(0, 2); double T03 = T(0, 3);
	double T10 = T(1, 0); double T11 = T(1, 1); double T12 = T(1, 2); double T13 = T(1, 3);
	double T20 = T(2, 0); double T21 = T(2, 1); double T22 = T(2, 2); double T23 = T(2, 3);

	////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
	double q1[2];
	{
		double A = d6*T12 - T13;
		double B = d6*T02 - T03;
		double R = A*A + B*B;
		if (fabs(A) < ZERO_THRESH) {
			double div;
			if (fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
				div = -SIGN(d4)*SIGN(B);
			else
				div = -d4 / B;
			double arcsin = asin(div);
			if (fabs(arcsin) < ZERO_THRESH)
				arcsin = 0.0;
			if (arcsin < 0.0)
				q1[0] = arcsin + 2.0*PI;
			else
				q1[0] = arcsin;
			q1[1] = PI - arcsin;
		}
		else if (fabs(B) < ZERO_THRESH) {
			double div;
			if (fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
				div = SIGN(d4)*SIGN(A);
			else
				div = d4 / A;
			double arccos = acos(div);
			q1[0] = arccos;
			q1[1] = 2.0*PI - arccos;
		}
		else if (d4*d4 > R) {
			return num_sols;
		}
		else {
			double arccos = acos(d4 / sqrt(R));
			double arctan = atan2(-B, A);
			double pos = arccos + arctan;
			double neg = -arccos + arctan;
			if (fabs(pos) < ZERO_THRESH)
				pos = 0.0;
			if (fabs(neg) < ZERO_THRESH)
				neg = 0.0;
			if (pos >= 0.0)
				q1[0] = pos;
			else
				q1[0] = 2.0*PI + pos;
			if (neg >= 0.0)
				q1[1] = neg;
			else
				q1[1] = 2.0*PI + neg;
		}
	}

	////////////////////////////// wrist 2 joint (q5) //////////////////////////////
	double q5[2][2];
	{
		for (int i = 0; i<2; i++) {
			double numer = (T03*sin(q1[i]) - T13*cos(q1[i]) - d4);
			double div;
			if (fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
				div = SIGN(numer) * SIGN(d6);
			else
				div = numer / d6;
			double arccos = acos(div);
			q5[i][0] = arccos;
			q5[i][1] = 2.0*PI - arccos;
		}
	}
	////////////////////////////////////////////////////////////////////////////////
	{
		for (int i = 0; i<2; i++) {
			for (int j = 0; j<2; j++) {
				double c1 = cos(q1[i]), s1 = sin(q1[i]);
				double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
				double q6;
				////////////////////////////// wrist 3 joint (q6) //////////////////////////////
				if (fabs(s5) < ZERO_THRESH)
					q6 = q6_des;
				else {
					q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1),
						SIGN(s5)*(T00*s1 - T10*c1));
					if (fabs(q6) < ZERO_THRESH)
						q6 = 0.0;
					if (q6 < 0.0)
						q6 += 2.0*PI;
				}
				////////////////////////////////////////////////////////////////////////////////

				double q2[2], q3[2], q4[2];
				///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
				double c6 = cos(q6), s6 = sin(q6);
				double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
				double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
				double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) +
					T03*c1 + T13*s1;
				double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

				double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
				if (fabs(fabs(c3) - 1.0) < ZERO_THRESH)
					c3 = SIGN(c3);
				else if (fabs(c3) > 1.0) {
					// TODO NO SOLUTION
					continue;
				}
				double arccos = acos(c3);
				q3[0] = arccos;
				q3[1] = 2.0*PI - arccos;
				double denom = a2*a2 + a3*a3 + 2 * a2*a3*c3;
				double s3 = sin(arccos);
				double A = (a2 + a3*c3), B = a3*s3;
				q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
				q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
				double c23_0 = cos(q2[0] + q3[0]);
				double s23_0 = sin(q2[0] + q3[0]);
				double c23_1 = cos(q2[1] + q3[1]);
				double s23_1 = sin(q2[1] + q3[1]);
				q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
				q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
				////////////////////////////////////////////////////////////////////////////////
				for (int k = 0; k<2; k++) {
					if (fabs(q2[k]) < ZERO_THRESH)
						q2[k] = 0.0;
					else if (q2[k] < 0.0) q2[k] += 2.0*PI;
					if (fabs(q4[k]) < ZERO_THRESH)
						q4[k] = 0.0;
					else if (q4[k] < 0.0) q4[k] += 2.0*PI;
					ik_sols_[num_sols * 6 + 0] = q1[i];    ik_sols_[num_sols * 6 + 1] = q2[k];
					ik_sols_[num_sols * 6 + 2] = q3[k];    ik_sols_[num_sols * 6 + 3] = q4[k];
					ik_sols_[num_sols * 6 + 4] = q5[i][j]; ik_sols_[num_sols * 6 + 5] = q6;
					num_sols++;
				}
			}
		}
	}

	// the solution joint angle may not be in the range we want
	for (int i = 0; i < num_sols; i++)
	{
		bool valid_solution = true;

		// try to bring the joint angle back to the range we want
		for (int j = 0; j < 6; j++)
		{
			double min = pp_.joint_range_[j * 2];
			double max = pp_.joint_range_[j * 2 + 1];

			if (imaging_or_probing == PROBING && j == 4) { //wrist 2

				min = pp_.probing_joint_range_wrist_2_[0];
				max = pp_.probing_joint_range_wrist_2_[1];
			}


			double q = ik_sols_[i * 6 + j];

			if (q > max) q -= 2 * PI;
			else if (q < min) q += 2 * PI;
			else continue;

			if (q <= max && q >= min) ik_sols_[i * 6 + j] = q;
			else
			{
				valid_solution = false;
				break;
			}
		}

		if (valid_solution)
		{
			ik_sols_vec.push_back(i);
		/*	std::cout << ik_sols_vec.back() << ": ";
			for (int k = 0; k < 6; k++)
				std::cout << ik_sols_[i * 6 + k] << " ";
			std::cout << "\n";*/
		}
	}

	return num_sols;
}


void VisionArmCombo::forward(const double* q, double* T)
{
	double s1 = sin(*q), c1 = cos(*q); q++;
	double q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
	double s3 = sin(*q), c3 = cos(*q); q234 += *q; q++;
	q234 += *q; q++;
	double s5 = sin(*q), c5 = cos(*q); q++;
	double s6 = sin(*q), c6 = cos(*q);
	double s234 = sin(q234), c234 = cos(q234);
	*T = ((c1*c234 - s1*s234)*s5) / 2.0 - c5*s1 + ((c1*c234 + s1*s234)*s5) / 2.0; T++;
	*T = (c6*(s1*s5 + ((c1*c234 - s1*s234)*c5) / 2.0 + ((c1*c234 + s1*s234)*c5) / 2.0) -
		(s6*((s1*c234 + c1*s234) - (s1*c234 - c1*s234))) / 2.0); T++;
	*T = (-(c6*((s1*c234 + c1*s234) - (s1*c234 - c1*s234))) / 2.0 -
		s6*(s1*s5 + ((c1*c234 - s1*s234)*c5) / 2.0 + ((c1*c234 + s1*s234)*c5) / 2.0)); T++;
	*T = ((d5*(s1*c234 - c1*s234)) / 2.0 - (d5*(s1*c234 + c1*s234)) / 2.0 -
		d4*s1 + (d6*(c1*c234 - s1*s234)*s5) / 2.0 + (d6*(c1*c234 + s1*s234)*s5) / 2.0 -
		a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3); T++;
	*T = c1*c5 + ((s1*c234 + c1*s234)*s5) / 2.0 + ((s1*c234 - c1*s234)*s5) / 2.0; T++;
	*T = (c6*(((s1*c234 + c1*s234)*c5) / 2.0 - c1*s5 + ((s1*c234 - c1*s234)*c5) / 2.0) +
		s6*((c1*c234 - s1*s234) / 2.0 - (c1*c234 + s1*s234) / 2.0)); T++;
	*T = (c6*((c1*c234 - s1*s234) / 2.0 - (c1*c234 + s1*s234) / 2.0) -
		s6*(((s1*c234 + c1*s234)*c5) / 2.0 - c1*s5 + ((s1*c234 - c1*s234)*c5) / 2.0)); T++;
	*T = ((d5*(c1*c234 - s1*s234)) / 2.0 - (d5*(c1*c234 + s1*s234)) / 2.0 + d4*c1 +
		(d6*(s1*c234 + c1*s234)*s5) / 2.0 + (d6*(s1*c234 - c1*s234)*s5) / 2.0 + d6*c1*c5 -
		a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3); T++;
	*T = ((c234*c5 - s234*s5) / 2.0 - (c234*c5 + s234*s5) / 2.0); T++;
	*T = ((s234*c6 - c234*s6) / 2.0 - (s234*c6 + c234*s6) / 2.0 - s234*c5*c6); T++;
	*T = (s234*c5*s6 - (c234*c6 + s234*s6) / 2.0 - (c234*c6 - s234*s6) / 2.0); T++;
	*T = (d1 + (d6*(c234*c5 - s234*s5)) / 2.0 + a3*(s2*c3 + c2*s3) + a2*s2 -
		(d6*(c234*c5 + s234*s5)) / 2.0 - d5*c234); T++;
	*T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;
}

void VisionArmCombo::float2double(float* array6_f, double* array6_d)
{
	for (int i = 0; i < 6; i++) array6_d[i] = array6_f[i];
}

void VisionArmCombo::double2float(double* array6_d, float* array6_f)
{
	for (int i = 0; i < 6; i++) array6_f[i] = array6_d[i];
}

bool VisionArmCombo::moveToConfigGetKinectPointCloud(ArmConfig & config, bool get_cloud, bool try_direct_path, bool add_cloud_to_occupancy_grid, int imaging_or_probing)
{

#ifdef ENABLE_PP
	if (!pp_.path_planner_ready_)
	{
		std::cout << "path planner\n";
		return false;
	}
#endif

	if (robot_arm_client_ == NULL)
	{
		std::cout << "robot arm client not ready\n";
		return false;
	}

#ifdef ENABLE_PP
	if (pp_.collisionCheckForSingleConfig(config.joint_pos_f))
#else 
	if (pp_.selfCollision(config.joint_pos_f))
#endif
	{
		std::cout << "target config collision found\n";
		return false;
	}

	double start_pose_d[6];
	robot_arm_client_->getCurJointPose(start_pose_d);

	float start_pose[6];
	double2float(start_pose_d, start_pose);

	double start_to_goal_L2 = L2Norm(start_pose_d, config.joint_pos_d);
	//std::cout << "L2 NORM: " << start_to_goal_L2 << "\n";

	float distance_traveled_tcp = 0.0f;
	float distance_traveled_elbow = 0.0f;

	Eigen::Vector3f pre_tcp_pos;
	Eigen::Vector3f pre_elbow_pos;

	if (start_to_goal_L2 > 1e-3)	//away from goal
	{

#ifdef ENABLE_PP
		if (!pp_.planPath(start_pose, config.joint_pos_f, true, try_direct_path, imaging_or_probing))
		{
			std::cout << "collsion-free path not found\n";
			viewPlannedPath(start_pose, config.joint_pos_f);
			return false;
		}

		viewPlannedPath(start_pose, config.joint_pos_f);

		pre_tcp_pos = pp_.fk_mat_.col(3).head<3>();
		pre_elbow_pos = pp_.DH_mat_vec_[1].col(3).head<3>();

		for (int i = 0; i < pp_.shortest_path_index_vec_.size(); i++)
		{
			float config_f[6];
			memcpy(config_f, pp_.random_nodes_buffer_ + pp_.shortest_path_index_vec_[i] * pp_.num_joints_, pp_.num_joints_*sizeof(float));

			pp_.forwardKinematicsUR10(config_f);
			distance_traveled_tcp += (pp_.fk_mat_.col(3).head<3>() - pre_tcp_pos).norm();
			distance_traveled_elbow += (pp_.DH_mat_vec_[1].col(3).head<3>() - pre_elbow_pos).norm();
			pre_tcp_pos = pp_.fk_mat_.col(3).head<3>();
			pre_elbow_pos = pp_.DH_mat_vec_[1].col(3).head<3>();

			double config_d[6];
			float2double(config_f, config_d);
			robot_arm_client_->moveHandJ(config_d, move_joint_speed_, move_joint_acceleration_, true);
			//std::getchar();
		}

#endif

		robot_arm_client_->moveHandJ(config.joint_pos_d, move_joint_speed_, move_joint_acceleration_, true);

		pp_.forwardKinematicsUR10(config.joint_pos_f);
		distance_traveled_tcp += (pp_.fk_mat_.col(3).head<3>() - pre_tcp_pos).norm();
		distance_traveled_elbow += (pp_.DH_mat_vec_[1].col(3).head<3>() - pre_elbow_pos).norm();

		std::cout << "tcp traveled distance (m): " << distance_traveled_tcp<<"\n";
		std::cout << "elbow traveled distance (m): " << distance_traveled_elbow << "\n";
	}

	if (get_cloud)
	{
		Sleep(1000);

		// get point cloud from kinect
		PointCloudT::Ptr point_cloud(new PointCloudT);

		kinect_thread_->getCurPointCloud(point_cloud);

	/*	viewer_->removeAllPointClouds();
		viewer_->addPointCloud(point_cloud);
		display();*/

		// depth value correction
		//for (int i = 0; i < point_cloud->points.size(); i++) point_cloud->points[i].z -= 0.024f;

		// get robot hand pose
		Eigen::Matrix4f hand2base;

		getCurHandPose(hand2base);

		Eigen::Matrix4f cam2base;

		cam2base = hand2base*cam2hand_kinect_;

		PointCloudT::Ptr cloud_in_base(new PointCloudT);

		pcl::transformPointCloud(*point_cloud, *cloud_in_base, cam2base);

		pass_.setInputCloud(cloud_in_base);
		pass_.setFilterFieldName("y");
		pass_.setFilterLimits(-1.02f, -0.35f);
		pass_.setFilterLimitsNegative(false);
		pass_.filter(*point_cloud);

		pass_.setInputCloud(point_cloud);
		pass_.setFilterFieldName("z");
		pass_.setFilterLimits(-0.65f, 1.28f);
		pass_.filter(*cloud_in_base);

		*kinect_cloud_ += *cloud_in_base;

		// voxel grid donwsample
		vox_.setInputCloud(kinect_cloud_);
		vox_.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);

		point_cloud->clear();
		vox_.filter(*point_cloud);

		kinect_cloud_->clear();

		*kinect_cloud_ += *point_cloud;

		std::wcout << "Kinect cloud size: " << kinect_cloud_->size() << "\n";

#ifdef ENABLE_PP
		if(add_cloud_to_occupancy_grid)	pp_.addPointCloudToOccupancyGrid(kinect_cloud_);
#endif

		viewer_->removeAllPointClouds();

		viewer_->addPointCloud(kinect_cloud_, "cloud", 0);

		viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

		display();
	}

	return true;
}

double VisionArmCombo::L2Norm(double* array6_1, double* array6_2)
{
	double distance = 0.;
	for (int i = 0; i < 6; i++)
	{
		double  r = array6_1[i] - array6_2[i];
		distance += r*r;
	}
	return sqrt(distance);
}

void VisionArmCombo::processGrowthChamberEnviroment(PointCloudT::Ptr cloud_in_arm_base, float shelf_z_value, int num_plants, int rover_position = 1)
{
	int64 t0 = cv::getTickCount();

	// keep points above shelf 
	PointCloudT::Ptr cloud(new PointCloudT);
#if 1
	pass_.setInputCloud(cloud_in_arm_base);
	pass_.setFilterFieldName("z");
	pass_.setFilterLimits(shelf_z_value, 1.28f);
	pass_.setFilterLimitsNegative(false);
	pass_.filter(*cloud);

	// remove growth chamber door
	*cloud_in_arm_base = *cloud;
	pass_.setInputCloud(cloud_in_arm_base);
	pass_.setFilterFieldName("y");
	pass_.setFilterLimits(-1.5f, -0.3f);
	pass_.setFilterLimitsNegative(false);
	pass_.filter(*cloud);
#endif

	// remove side walls
	pass_.setInputCloud(cloud_in_arm_base);
	pass_.setFilterFieldName("x");

	int num_pot_left = 0;
	
	if(rover_position == 1)
		//pass_.setFilterLimits(-1.f, 1.f);
		pass_.setFilterLimits(-1.0f, 0.58f);
	else if (rover_position == 0)
	{
		int boundary_object_idx;
		for (int i = 0; i < object_centers_.rows; i++)
		{
			if (object_centers_.at<cv::Vec3f>(i, 0)[0] > scan_pot_x_abs_limit_)
			{
				boundary_object_idx = i;
				num_pot_left++;
			}
			else break;
		}

		int cluster_idx;
		for (int i = 0; i < kmeans_label_sorted_.size(); i++)
		{
			if (kmeans_label_sorted_[i] == boundary_object_idx)
			{
				cluster_idx = i;
				break;
			}
		}

		std::cout << "num pots left: " << num_pot_left << "\n";

		Eigen::Vector4f point((*plant_cluster_min_vec_[cluster_idx])(0)- rover_dist_, 0, 0, 1);

		std::cout << "initial boundary point " << point.transpose() << "\n";
	
		point = icp_final_transformation_*point;

		std::cout << "final boundary point " << point.transpose() << "\n";

		pass_.setFilterLimits(point(0), 0.4f);
	}
	else if (rover_position == 2)
	{
		int boundary_object_idx;
		for (int i = object_centers_.rows-1; i >= 0; i--)
		{
			if (object_centers_.at<cv::Vec3f>(i, 0)[0] < -scan_pot_x_abs_limit_)
			{
				boundary_object_idx = i;
				num_pot_left++;
			}
			else break;
		}

		int cluster_idx;
		for (int i = 0; i < kmeans_label_sorted_.size(); i++)
		{
			if (kmeans_label_sorted_[i] == boundary_object_idx)
			{
				cluster_idx = i;
				break;
			}
		}

		std::cout << "num pots left: " << num_pot_left << "\n";

		Eigen::Vector4f point((*plant_cluster_max_vec_[cluster_idx])(0) + rover_dist_, 0, 0, 1);

		std::cout << "initial boundary point " << point.transpose() << "\n";

		point = icp_final_transformation_*point;

		std::cout << "final boundary point " << point.transpose() << "\n";

		pass_.setFilterLimits(-0.5f, point(0));
	}

	pass_.setFilterLimitsNegative(false);
	pass_.filter(*cloud);

	int64 t1 = cv::getTickCount();
	std::cout << "pass through filter time:" << (t1 - t0) / cv::getTickFrequency() << "\n";

	std::string save_path(data_saving_folder_.begin(), data_saving_folder_.end());
	save_path.append("kinect_top_view.pcd");

	std::cout <<"cloud size: "<< cloud->size() << "\n";
	std::cout << save_path << "\n";
	if(cloud->size() != 0)
		pcl::io::savePCDFileBinary(save_path, *cloud);

	// remove small clusters
	t0 = cv::getTickCount();
	*cloud_in_arm_base = *cloud;
	smallClusterRemoval(cloud_in_arm_base, 0.02, 10, cloud);

	t1 = cv::getTickCount();
	std::cout << "remove small cluster time:" << (t1 - t0) / cv::getTickFrequency() << "\n";

	//Eigen::Vector4f min, max;
	//pcl::getMinMax3D(*cloud, min, max);
	//std::cout << "min: " << min.transpose() << " max: " << max.transpose() << "\n";
	
	t0 = cv::getTickCount();
	// opencv kmeans
	std::vector<cv::Point3f> cv_points(cloud->points.size());
	cv::Mat labels;
	
	for (int i = 0; i < cloud->points.size(); i++)
	{
		cv::Point3f point;
		point.x = cloud->points[i].x;
		point.y = cloud->points[i].y;
		//point.z = cloud->points[i].z;
		point.z = 0;		// project point cloud to x-y plane to get pot center (x,y)
		cv_points[i] = point;
	}

	cv::Mat object_centers_on_side;
	// object_centers z value = 0 
	if(rover_position == 1)
		cv::kmeans(cv_points, num_plants, labels, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001), 10, cv::KMEANS_PP_CENTERS, object_centers_);
	else
		cv::kmeans(cv_points, num_pot_left, labels, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001), 10, cv::KMEANS_PP_CENTERS, object_centers_on_side);

	t1 = cv::getTickCount();
	std::cout << "k-means time:" << (t1 -t0)/cv::getTickFrequency()<<"\n";

	//std::cout << object_centers_.rows<<" "<<object_centers_.cols<< "\n";

	//object_centers dimension rows: number of plants, cols: 3 (x,y,z)

	// insertion sort object centers based on x value
	if(rover_position == 1)
	for (int i = 1; i < object_centers_.rows; i++)
	{
		int j = i;
		while (j > 0 && object_centers_.at<cv::Vec3f>(j, 0)[0] > object_centers_.at<cv::Vec3f>(j-1,0)[0])
		{
			cv::Vec3f tmp(object_centers_.at<cv::Vec3f>(j-1, 0));
			object_centers_.at<cv::Vec3f>(j - 1, 0) = object_centers_.at<cv::Vec3f>(j, 0);
			object_centers_.at<cv::Vec3f>(j, 0) = tmp;
			j--;
		}
	}
	else
	for (int i = 1; i < object_centers_on_side.rows; i++)
	{
		int j = i;
		while (j > 0 && object_centers_on_side.at<cv::Vec3f>(j, 0)[0] > object_centers_on_side.at<cv::Vec3f>(j - 1, 0)[0])
		{
			cv::Vec3f tmp(object_centers_on_side.at<cv::Vec3f>(j - 1, 0));
			object_centers_on_side.at<cv::Vec3f>(j - 1, 0) = object_centers_on_side.at<cv::Vec3f>(j, 0);
			object_centers_on_side.at<cv::Vec3f>(j, 0) = tmp;
			j--;
		}
	}

	int tmp_num = rover_position == 1 ? num_plants : num_pot_left;

	std::vector<PointCloudT::Ptr> clusters(tmp_num);
	
	// generate some random colors
	cv::RNG rng;

	std::vector<float> colors(tmp_num);

	for (int i = 0; i < tmp_num; i++)
	{
		PointCloudT::Ptr cloud(new PointCloudT);
		clusters[i] = cloud;
		uint32_t rgb = ((uint32_t)rng.uniform(0, 255) << 16 | (uint32_t)rng.uniform(0, 255) << 8 | (uint32_t)rng.uniform(0, 255));
		colors[i] = *reinterpret_cast<float*>(&rgb);
	}
	
	for (int i = 0; i < cv_points.size(); i++)
	{
		int label = labels.at<int>(i, 0);
		//PointT point;
		//point.x = cv_points[i].x;
		//point.y = cv_points[i].y;
		//point.z = cloud->points[i].z;//cv_points[i].z;
		//point.rgb = colors[label];
		cloud->points[i].rgb = colors[label];
		clusters[label]->push_back(cloud->points[i]);
	}

	viewer_->removeAllPointClouds(0);


	std::vector<float> cluster_radius_vec(tmp_num);
	std::vector<Eigen::Vector4f> cluster_center_vec(tmp_num);
	std::vector<float> cluster_max_z(tmp_num);
	
	std::vector<int> kmeans_label_sorted_local;

	if (rover_position == 1)
	{
		kmeans_label_sorted_.clear();
		kmeans_label_sorted_.resize(tmp_num);
		std::vector<int> search_done_label(tmp_num);
		for (int i = 0; i < tmp_num; i++) search_done_label[i] = 0;

		for (int i = 0; i < clusters.size(); i++)
		{
			PointCloudT tmp_pc;

			tmp_pc += *clusters[i];

			for (auto & p : tmp_pc.points)	p.z = 0;

			Eigen::Vector4f centroid;

			pcl::compute3DCentroid(tmp_pc, centroid);

			float min_distance = 1e3f;
			int best_idx = -1;
			for (int j = 0; j < object_centers_.rows; j++)
			{
				if (search_done_label[j] == 1) continue;

				float diff_x = centroid(0) - object_centers_.at<cv::Vec3f>(j, 0)[0];

				float diff_y = centroid(1) - object_centers_.at<cv::Vec3f>(j, 0)[1];

				float diff_z = centroid(2) - object_centers_.at<cv::Vec3f>(j, 0)[2];

				float diff = diff_x*diff_x + diff_y*diff_y + diff_z*diff_z;

				if (diff < min_distance)
				{
					min_distance = diff;
					best_idx = j;
				}
			}

			kmeans_label_sorted_[i] = best_idx;

			search_done_label[best_idx] = 1;

			std::cout << "kmeans label " << i << " ->" << " sorted " << best_idx << "\n";
		}
	}
	else
	{
		kmeans_label_sorted_local.resize(tmp_num);
		std::vector<int> search_done_label(tmp_num);
		for (int i = 0; i < tmp_num; i++) search_done_label[i] = 0;

		for (int i = 0; i < clusters.size(); i++)
		{
			PointCloudT tmp_pc;

			tmp_pc += *clusters[i];

			for (auto & p : tmp_pc.points)	p.z = 0;

			Eigen::Vector4f centroid;

			pcl::compute3DCentroid(tmp_pc, centroid);

			float min_distance = 1e3f;
			int best_idx = -1;
			for (int j = 0; j < object_centers_on_side.rows; j++)
			{
				if (search_done_label[j] == 1) continue;

				float diff_x = centroid(0) - object_centers_on_side.at<cv::Vec3f>(j, 0)[0];

				float diff_y = centroid(1) - object_centers_on_side.at<cv::Vec3f>(j, 0)[1];

				float diff_z = centroid(2) - object_centers_on_side.at<cv::Vec3f>(j, 0)[2];

				float diff = diff_x*diff_x + diff_y*diff_y + diff_z*diff_z;

				if (diff < min_distance)
				{
					min_distance = diff;
					best_idx = j;
				}
			}

			kmeans_label_sorted_local[i] = best_idx;

			search_done_label[best_idx] = 1;

			std::cout << "kmeans label local " << i << " ->" << " sorted " << best_idx << "\n";
		}
	}

	for (int i = 0; i < tmp_num; i++)
	{
		Eigen::Vector4f* min = new Eigen::Vector4f();
		Eigen::Vector4f* max = new Eigen::Vector4f();;

		pcl::getMinMax3D(*clusters[i], *min, *max);

		if (rover_position == 1)
		{
			plant_cluster_min_vec_[i] = min;
			plant_cluster_max_vec_[i] = max;
		}

		cluster_radius_vec[i] = ((*max)(0) - (*min)(0))*0.5f;
		cluster_center_vec[i] << 0.5f*(*max + *min);

		cluster_max_z[i] = (*max)(2);

		std::string name = "cluster" + std::to_string(i);

		viewer_->addPointCloud(clusters[i], name, 0);

		viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
	}

	if(rover_position == 1)
	for (int i = 0; i < num_plants; i++)
	{
		pcl::ModelCoefficients coeffs;
		coeffs.values.resize(4);
		coeffs.values[0] = object_centers_.at<cv::Vec3f>(i, 0)[0];
		coeffs.values[1] = object_centers_.at<cv::Vec3f>(i, 0)[1];
		coeffs.values[2] = object_centers_.at<cv::Vec3f>(i, 0)[2];
		coeffs.values[3] = cluster_radius_vec[i];

		std::cout << object_centers_.at<cv::Vec3f>(i, 0)[0] << "\n";

		viewer_->addSphere(coeffs,"sphere" + std::to_string(i), 0);
	}
	else
	for (int i = 0; i < num_pot_left; i++)
	{
		pcl::ModelCoefficients coeffs;
		coeffs.values.resize(4);
		coeffs.values[0] = object_centers_on_side.at<cv::Vec3f>(i, 0)[0];
		coeffs.values[1] = object_centers_on_side.at<cv::Vec3f>(i, 0)[1];
		coeffs.values[2] = -0.55f;// object_centers_on_side.at<cv::Vec3f>(i, 0)[2];
		coeffs.values[3] = cluster_radius_vec[i];

		std::cout << object_centers_on_side.at<cv::Vec3f>(i, 0)[0] << "\n";

		viewer_->addSphere(coeffs, "sphere" + std::to_string(i), 0);
	}

	display();

	plant_laser_pc_vec_.clear();
	//return;
	// scan each cluster
	if (rover_position == 1)
	{
		for (int i = 0; i < num_plants; i++)
		{
			// x
			if (abs(object_centers_.at<cv::Vec3f>(i, 0)[0]) > scan_pot_x_abs_limit_)
			{
				//pot_process_status_[i] = false;
				continue;
			}

			//pot_process_status_[i] = true;
			int cluster_idx;

			for (int j = 0; j < kmeans_label_sorted_.size(); j++)
			{
				if (i == kmeans_label_sorted_[j])
				{
					cluster_idx = j;
					break;
				}
			}

			std::cout << "cluster idx " << cluster_idx << "\n";

			float max_z = (*plant_cluster_max_vec_[cluster_idx])(2);
			float radius = 0.5f*((*plant_cluster_max_vec_[cluster_idx])(0) - (*plant_cluster_min_vec_[cluster_idx])(0));

			scanPlantCluster(object_centers_.at<cv::Vec3f>(i, 0), max_z, radius, i);
			std::cout << "processed pot " << i << "\n";

			std::vector<pcl::PointXYZRGBNormal> probe_pn_vec;

			std::cout << "Extract leaf probing points for plant " << i + 1 << "\n";

			std::vector<pcl::PointIndices> leaf_cluster_indices_vec;
			std::vector<std::vector<Eigen::Matrix4d*>> hyperscan_hand_pose_sequence_vec;
			std::vector<std::vector<ArmConfig>> hyperscan_arm_config_sequence_vec;
			std::vector<int> hyperscan_leaf_id_vec;

			extractLeafProbingPointsAndHyperspectralScanPoses(plant_laser_pc_vec_[i], probe_pn_vec,
																leaf_cluster_indices_vec,
																hyperscan_hand_pose_sequence_vec,
																hyperscan_arm_config_sequence_vec,
																hyperscan_leaf_id_vec, i);

			if (enable_probing_ == 1) {
				// multiple probing points on a leaf
				// go through each leaf
				for (int leaf_idx = 0; leaf_idx < leaf_probing_pn_vector_.size(); leaf_idx++)
				{
					const int sorted_leaf_idx = leaf_cluster_order_[leaf_idx].id;

					int num_successful_probing = 0;
					PointT pre_probed_point;
					pre_probed_point.x = pre_probed_point.y = pre_probed_point.z = 0.0f;
					for (int patch_idx = 0; patch_idx < leaf_probing_pn_vector_[sorted_leaf_idx].size(); patch_idx++)
					{
						if (num_successful_probing >= max_samples_per_leaf_) break;

						PointT p;
						p.x = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].x;
						p.y = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].y;
						p.z = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].z;

						/*	if (std::sqrt(pow(p.x - pre_probed_point.x, 2.0f) + pow(p.y - pre_probed_point.y, 2.0f)
								+ pow(p.z - pre_probed_point.z, 2.0f)) < 0.04)
								continue;
								*/
						pcl::Normal n;
						n.normal_x = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].normal_x;
						n.normal_y = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].normal_y;
						n.normal_z = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].normal_z;

						//	continue;

						if (probeLeaf(p, n, PAM, i))
						{
							if (raman_ != NULL)
								probeLeaf(p, n, RAMAN_1064, i);

							num_successful_probing++;
							pre_probed_point.x = p.x;
							pre_probed_point.y = p.y;
							pre_probed_point.z = p.z;
						}

						//std::cout << leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].x << "\n";
					}
				}
			}

			if (hypercam_ != NULL) {

				robot_arm_client_->lineLightControl(true);

				std::cout << "hyperspectral scan...\n";

				for (int hyperscan_idx = 0; hyperscan_idx < hyperscan_hand_pose_sequence_vec.size(); hyperscan_idx++) {

					scanLeafWithHyperspectral(hyperscan_hand_pose_sequence_vec[hyperscan_idx], hyperscan_arm_config_sequence_vec[hyperscan_idx], i);
				}

				robot_arm_client_->lineLightControl(false);
			}
			else {

				std::cout << "Hyperspectral not initialized.\n";
			}
		}

#if 0
		for (int i = 0; i < plant_laser_pc_vec_.size(); i++)
		{
			std::vector<pcl::PointXYZRGBNormal> probe_pn_vec;

			std::cout << "Extract leaf probing points for plant "<<i+1<<"\n";

			std::vector<pcl::PointIndices> leaf_cluster_indices_vec;
			std::vector<std::vector<Eigen::Matrix4d*>> hyperscan_hand_pose_sequence_vec;
			std::vector<std::vector<ArmConfig>> hyperscan_arm_config_sequence_vec;
			std::vector<int> hyperscan_leaf_id_vec;

			extractLeafProbingPoints(plant_laser_pc_vec_[i], probe_pn_vec, 
									leaf_cluster_indices_vec, 
									hyperscan_hand_pose_sequence_vec, 
									hyperscan_arm_config_sequence_vec, 
									hyperscan_leaf_id_vec);

#if 1
			// multiple probing points on a leaf
			// go through each leaf
			for (int leaf_idx = 0; leaf_idx < leaf_probing_pn_vector_.size(); leaf_idx++)
			{
				const int sorted_leaf_idx = leaf_cluster_order_[leaf_idx].id;
				int max_samples_per_leaf = 2;
				int num_successful_probing = 0;
				PointT pre_probed_point;
				pre_probed_point.x = pre_probed_point.y = pre_probed_point.z = 0.0f;
				for (int patch_idx = 0; patch_idx < leaf_probing_pn_vector_[sorted_leaf_idx].size(); patch_idx++)
				{
					if (num_successful_probing >= max_samples_per_leaf) break;

					PointT p;
					p.x = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].x;
					p.y = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].y;
					p.z = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].z;

					if (std::sqrt(pow(p.x - pre_probed_point.x, 2.0f) + pow(p.y - pre_probed_point.y, 2.0f)
						+ pow(p.z - pre_probed_point.z, 2.0f)) < 0.04)
						continue;

					pcl::Normal n;
					n.normal_x = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].normal_x;
					n.normal_y = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].normal_y;
					n.normal_z = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].normal_z;

				//	continue;

					if (probeLeaf(p, n))
					{
						num_successful_probing++;
						pre_probed_point.x = p.x;
						pre_probed_point.y = p.y;
						pre_probed_point.z = p.z;
					}

					//std::cout << leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].x << "\n";
				}
			}
#endif
			if (hypercam_ != NULL) {

				for (int hyperscan_idx = 0; hyperscan_idx < hyperscan_hand_pose_sequence_vec.size(); hyperscan_idx++) {

					scanLeafWithHyperspectral(hyperscan_hand_pose_sequence_vec[hyperscan_idx], hyperscan_arm_config_sequence_vec[hyperscan_idx]);
				}
			}
			else {

				std::cout << "Hyperspectral not initialized.\n";
			}
		}
#endif
	}
	else
	{
		for (int i = 0; i < object_centers_on_side.rows; i++)
		{
			int cluster_idx;

			for (int j = 0; j < kmeans_label_sorted_local.size(); j++)
			{
				if (i == kmeans_label_sorted_local[j])
				{
					cluster_idx = j;
					break;
				}
			}

			std::cout << "cluster idx " << cluster_idx << "\n";

			float max_z = cluster_max_z[cluster_idx];
			float radius = cluster_radius_vec[cluster_idx];
			scanPlantCluster(object_centers_on_side.at<cv::Vec3f>(i, 0), max_z, radius);
		}
	}

	return;

#if 0
	pcl::MinCutSegmentation<PointT> seg;

	seg.setInputCloud(cloud);

	for (int i = 0; i < num_plants; i++)
	{
		PointCloudT::Ptr foreground_points(new PointCloudT);
		PointT point;
		point.x = object_centers_.at<cv::Vec3f>(i, 0)[0];
		point.y = object_centers_.at<cv::Vec3f>(i, 0)[1];
		point.z = object_centers_.at<cv::Vec3f>(i, 0)[2];

		foreground_points->points.push_back(point);
		seg.setForegroundPoints(foreground_points);

		PointCloudT::Ptr background_points(new PointCloudT);
		for (int j = 0; j < num_plants; j++)
		{
			if (j == i) continue;

			PointT bpoint;
			bpoint.x = object_centers_.at<cv::Vec3f>(j, 0)[0];
			bpoint.y = object_centers_.at<cv::Vec3f>(j, 0)[1];
			bpoint.z = object_centers_.at<cv::Vec3f>(j, 0)[2];
			background_points->points.push_back(bpoint);
		}

		seg.setBackgroundPoints(background_points);

		seg.setSigma(0.2);
		seg.setRadius(0.1);
		seg.setNumberOfNeighbours(10);
		seg.setSourceWeight(0.8);

		std::vector <pcl::PointIndices> clusters;
		seg.extract(clusters);

		PointCloudT::Ptr colored_cloud = seg.getColoredCloud();
		viewer_->removeAllPointClouds();
		viewer_->addPointCloud(colored_cloud);
		display();
	}

	kinect_cloud_->clear();
	*kinect_cloud_ += *cloud;

	viewer_->removeAllPointClouds();
	viewer_->addPointCloud(cloud);
	display();
#endif
}

#if 1
void VisionArmCombo::addSupervoxelConnectionsToViewer(PointT &supervoxel_center,
											PointCloudT &adjacent_supervoxel_centers, std::string name,
											boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
	for (int i = 0; i < adjacent_supervoxel_centers.points.size(); i++)
	{
		pcl::ModelCoefficients line;
		line.values.resize(6);
		line.values[0] = supervoxel_center.x; line.values[1] = supervoxel_center.y; line.values[2] = supervoxel_center.z;
		line.values[3] = adjacent_supervoxel_centers.points[i].x - supervoxel_center.x;
		line.values[4] = adjacent_supervoxel_centers.points[i].y - supervoxel_center.y;
		line.values[5] = adjacent_supervoxel_centers.points[i].z - supervoxel_center.z;

		std::string line_name = name + std::to_string(i);

		viewer->addLine(line, line_name, 0);
	}
}


void VisionArmCombo::extractProbeSurfacePatchFromPointCloud(PointCloudT::Ptr cloud, std::vector<pcl::Supervoxel<PointT>::Ptr> & potential_probe_supervoxels, 
															std::vector<pcl::PointXYZRGBNormal> & probing_point_normal_vec)
{
	pcl::SupervoxelClustering<PointT> super(voxel_grid_size_, seed_resolution_);
	super.setInputCloud(cloud);
	super.setColorImportance(color_importance_);
	super.setSpatialImportance(spatial_importance_);
	super.setNormalImportance(normal_importance_);
	std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
	super.extract(supervoxel_clusters);

	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);

	//viewer_->removeAllPointClouds();
	/*PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud();
	viewer_->addPointCloud(voxel_centroid_cloud, "voxel centroids");
	viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "voxel centroids");
	viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.95, "voxel centroids");
	*/
	PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud();

	std::cout << "showing supervoxel cloud...\n";
	viewer_->removeAllPointClouds();
	viewer_->addPointCloud(labeled_voxel_cloud, "labeled voxels"+std::to_string(cv::getTickCount())); //viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "labeled voxels");

	pcl::SupervoxelClustering<PointT>::VoxelAdjacencyList supervoxelAdjacencyList;

	// label and edge length
	super.getSupervoxelAdjacencyList(supervoxelAdjacencyList);

	typedef pcl::SupervoxelClustering<PointT>::VoxelAdjacencyList::vertex_iterator sv_vertex_iterator;
	typedef pcl::SupervoxelClustering<PointT>::VoxelAdjacencyList::edge_iterator sv_edge_iterator;
	typedef pcl::SupervoxelClustering<PointT>::VoxelID VoxelID;	//descriptor
	typedef pcl::SupervoxelClustering<PointT>::EdgeID EdgeID;

	//sv_vertex_iterator vit, vend;
	//std::tie(vit, vend) = boost::vertices(supervoxelAdjacencyList);
	//int id = 1;
	//for (sv_vertex_iterator it = vit; it != vend; it++)
	//{
	
	//	std::cout <<"id "<<id++<< " "<<supervoxelAdjacencyList[*it] << "\n";
	//}

	//sv_edge_iterator eit, eend;
	//std::tie(eit, eend) = boost::edges(supervoxelAdjacencyList);

	//for (sv_edge_iterator it = eit; it != eend; it++)
	//{
	////	std::cout <<"edge weight "<< supervoxelAdjacencyList[(*it)] << "\n";

	//	//std::cout<<supervoxelAdjacencyList[(*it).m_source]<<"\n";
	//	
	//}

	//PointLCloudT::Ptr supervoxel_cloud(new PointLCloudT);
	//PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud(supervoxel_clusters);
	////We have this disabled so graph is easy to see, uncomment to see supervoxel normals
	////viewer_->addPointCloudNormals<pcl::PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");
	
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);

	potential_probe_supervoxels.clear();

	//To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
	std::multimap<uint32_t, uint32_t>::iterator label_itr = supervoxel_adjacency.begin();
	for (; label_itr != supervoxel_adjacency.end(); )
	{
		//First get the label
		uint32_t supervoxel_label = label_itr->first;
		//Now get the supervoxel corresponding to the label
		pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);

		std::pair<std::multimap<uint32_t, uint32_t>::iterator, std::multimap<uint32_t, uint32_t>::iterator> range = supervoxel_adjacency.equal_range(supervoxel_label);
		int num_neighbors = std::distance(range.first, range.second);

		if (supervoxel->voxels_->size() > 30 && num_neighbors > 2)
		{
			// PCA 
			pcl::PCA<PointT> pca(*supervoxel->voxels_);
			Eigen::Vector3f eigen_values = pca.getEigenValues();
			//std::cout << "size "<< supervoxel->voxels_->points.size()<<" min eigen value "<< eigen_values(2) << "\n";
			
			//check RMS distance
			if ( std::sqrt(eigen_values(2)/supervoxel->voxels_->points.size()) < probing_patch_rmse_)// && supervoxel->voxels_->points.size() > 60 )
			{
				potential_probe_supervoxels.push_back(supervoxel);
				pcl::ModelCoefficients line; line.values.resize(6);
				line.values[0] = supervoxel->centroid_.x;
				line.values[1] = supervoxel->centroid_.y;
				line.values[2] = supervoxel->centroid_.z;
				float sign = supervoxel->normal_.normal_z >= 0 ? 1.0f : -1.0f;
				line.values[3] = supervoxel->normal_.normal_x*0.02f*sign;
				line.values[4] = supervoxel->normal_.normal_y*0.02f*sign;
				line.values[5] = supervoxel->normal_.normal_z*0.02f*sign;
				
				//viewer_->addLine(line, "l" + std::to_string(supervoxel_label), 0);
				//viewer_->addPointCloud(supervoxel->voxels_, "c" + std::to_string(supervoxel_label), 0);

				std::vector<int> pointIdxNKNSearch(1);
				std::vector<float> pointNKNSquaredDistance(1);
				PointT searchPoint;
				searchPoint.x = supervoxel->centroid_.x;
				searchPoint.y = supervoxel->centroid_.y;
				searchPoint.z = supervoxel->centroid_.z;

				if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
				{
					pcl::PointXYZRGBNormal pn;
					pn.x = cloud->points[pointIdxNKNSearch[0]].x;
					pn.y = cloud->points[pointIdxNKNSearch[0]].y;
					pn.z = cloud->points[pointIdxNKNSearch[0]].z;
					pn.r = pn.g = pn.b = 255;
					pn.normal_x = supervoxel->normal_.normal_x*sign;
					pn.normal_y = supervoxel->normal_.normal_y*sign;
					pn.normal_z = supervoxel->normal_.normal_z*sign;

					probing_point_normal_vec.push_back(pn);
				}
			}
		}

		//Move iterator forward to next label
		label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
	}

	display();
}
#endif

bool VisionArmCombo::computeCollisionFreeProbeOrScanPose(PointT & point, pcl::Normal & normal, int sensor_type, std::vector<ArmConfig> & solution_config_vec,
															Eigen::Matrix4d & scan_start_or_probe_hand_pose, Eigen::Vector3d & hand_translation)
{
	Eigen::Vector3d z_dir;

	// make z point down
	if (normal.normal_z > 0) z_dir << -normal.normal_x, -normal.normal_y, -normal.normal_z;
	else z_dir << normal.normal_x, normal.normal_y, normal.normal_z;

	z_dir.normalize();

	pcl::ModelCoefficients line; line.values.resize(6);
	line.values[0] = point.x;
	line.values[1] = point.y;
	line.values[2] = point.z;
	line.values[3] = -z_dir(0)*0.1;
	line.values[4] = -z_dir(1)*0.1;
	line.values[5] = -z_dir(2)*0.1;

	std::string name = "line";
	viewer_->removeAllShapes();
	viewer_->addLine(line, name, 0);
	display();

	// favor y pointing forward (Enviratron) (y<0)
	// sampling y from (1,0,0) to (-1,0,0)
	Eigen::Vector3d init_y_dir;
	Eigen::Matrix4d pose_to_hand;
	if (sensor_type == PROBE)
	{
		init_y_dir << 1.0, 0.0, 0.0;
		pose_to_hand = probe_to_hand_;
	}
	else if(sensor_type == SCANNER)
	{
		init_y_dir << 0.0, 1.0, 0.0;	// scanner
		pose_to_hand = scan_start_to_hand_;
	}
	else if (sensor_type == THERMAL)
	{
		init_y_dir << 1.0, 0.0, 0.0;

		Eigen::Matrix4d temp = Eigen::Matrix4d::Identity();

		temp(2, 3) = 0.3;

		temp = temp.inverse()*hand_to_thermal_.cast<double>().inverse();

		pose_to_hand = temp;
	}

	// project to plane
	init_y_dir = init_y_dir - z_dir*(init_y_dir.dot(z_dir));

	init_y_dir.normalize();

	//std::cout << "init_y_dir " << init_y_dir.transpose() << "\n";

	double step_size = 10. / 180.*M_PI;

	solution_config_vec.clear();

	std::vector<double> y_value_vec;

	double min_y = 1.0f;

	Eigen::Matrix4d optimal_hand_pose;

	ArmConfig optimal_config;

	double optimal_angle;

	bool solution_found = false;
 
#ifdef ENABLE_PP
	showOccupancyGrid(false);
#endif
	
	for (double angle = 0.; angle <= M_PI; angle += step_size)
	{
		Eigen::Vector3d y_dir = Eigen::AngleAxisd(angle, z_dir) * init_y_dir;

		//std::cout << "y_dir " << y_dir.transpose() << "\n";

		Eigen::Vector3d x_dir = y_dir.cross(z_dir);

		// robot hand pose
		Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

		pose.col(0).head<3>() = x_dir;
		pose.col(1).head<3>() = y_dir;
		pose.col(2).head<3>() = z_dir;
		
/*		if (sensor_type == PROBE)
		{
			Eigen::Vector3d point_retract;
			// If occupancy grid resolution too low, need to increase this buffer length, otherwise collision
			const double retract_dist = 0.04;	//
			point_retract(0) = point.x - z_dir(0)*retract_dist;
			point_retract(1) = point.y - z_dir(1)*retract_dist;
			point_retract(2) = point.z - z_dir(2)*retract_dist;
			pose.col(3).head<3>() = point_retract;
		}
		else if(sensor_type == SCANNER)*/
			pose.col(3).head<3>() << point.x, point.y, point.z;

		// visualization
#if 0
		Eigen::Affine3f affine_pose;
		affine_pose.matrix() = pose.cast<float>();
		viewer_->removeAllCoordinateSystems();
		viewer_->addCoordinateSystem(0.2, affine_pose, "co");
		
		affine_pose.matrix() = pose.cast<float>();
		viewer_->addCoordinateSystem(0.2, affine_pose, "co1");
		display();
#endif

		pose = pose * pose_to_hand;

		std::vector<int> ik_sols_vec;

		if(sensor_type == PROBE)
			inverseKinematics(pose, ik_sols_vec, PROBING);
		else
			inverseKinematics(pose, ik_sols_vec, IMAGING);

		//if (ik_sols_vec.size() == 0) std::cout << "no ik solution\n";
		
		for (auto idx : ik_sols_vec)
		{
			float sol_f[6];

			double2float(ik_sols_ + idx * num_joints_, sol_f);

#if 0
			// visualization
			ArmConfig config;
			config.setJointPos(sol_f[0], sol_f[1], sol_f[2], sol_f[3], sol_f[4], sol_f[5]);
			std::vector<PathPlanner::RefPoint> ref_points;
			std::vector<Eigen::Matrix3f> rot_mats;
			std::vector<PathPlanner::OBB> arm_obbs;
			pp_.computeReferencePointsOnArm(config.joint_pos_f, ref_points, rot_mats);
			pp_.getArmOBBModel(ref_points, rot_mats, arm_obbs); addOBBArmModelToViewer(arm_obbs);
#endif
			bool collision = true;

#ifdef ENABLE_PP
			if (sensor_type == PROBE)
				collision = pp_.collisionCheckForSingleConfig(sol_f, true);	//shorten probes
			else 
				collision = pp_.collisionCheckForSingleConfig(sol_f, false);
#else
			collision = pp_.selfCollision(sol_f);
#endif
			if(!collision)
			{
				//std::cout << "no collision\n";
				if (pose(1, 1) <= min_y)
				{
					optimal_hand_pose = pose;

					optimal_angle = angle;

					min_y = pose(1, 1);

					optimal_config.setJointPos(ik_sols_[idx*num_joints_], ik_sols_[idx*num_joints_ + 1],
						ik_sols_[idx*num_joints_ + 2], ik_sols_[idx*num_joints_ + 3],
						ik_sols_[idx*num_joints_ + 4], ik_sols_[idx*num_joints_ + 5]);

					solution_found = true;
				}
			}
			
			//else std::cout << "single config collision\n";
		}
	}

	if (!solution_found)
	{
		std::cout << "no collision free pose found\n";
		return solution_found;
	}

	std::vector<PathPlanner::RefPoint> ref_points;
	std::vector<Eigen::Matrix3f> rot_mats;
	std::vector<PathPlanner::OBB> arm_obbs;

	// visualization
	pp_.computeReferencePointsOnArm(optimal_config.joint_pos_f, ref_points, rot_mats);
	pp_.getArmOBBModel(ref_points, rot_mats, arm_obbs); addOBBArmModelToViewer(arm_obbs);

	solution_config_vec.push_back(optimal_config);

	scan_start_or_probe_hand_pose = optimal_hand_pose;

	if (sensor_type == PROBE || sensor_type == THERMAL) return solution_found;
	
	pose_to_hand = scan_end_to_hand_;

	Eigen::Vector3d y_dir = Eigen::AngleAxisd(optimal_angle, z_dir) * init_y_dir;

	//std::cout << "scan end y_dir " << y_dir.transpose() << "\n";

	Eigen::Vector3d x_dir = y_dir.cross(z_dir);

	Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

	pose.col(0).head<3>() = x_dir;
	pose.col(1).head<3>() = y_dir;
	pose.col(2).head<3>() = z_dir;
	pose.col(3).head<3>() << point.x, point.y, point.z;

	pose = pose * pose_to_hand;

	Eigen::Affine3f affine_pose;
	affine_pose.matrix() = pose.cast<float>();
	viewer_->removeAllCoordinateSystems();
	viewer_->addCoordinateSystem(0.2, affine_pose, "co", 0);
	display();

	std::vector<int> ik_sols_vec;

	inverseKinematics(pose, ik_sols_vec, IMAGING);

	solution_found = false;

	float config[6];

	for (auto idx : ik_sols_vec)
	{
		float sol_f[6];

		double2float(ik_sols_ + idx * num_joints_, sol_f);

#ifdef ENABLE_PP
		if (!pp_.collisionCheckForSingleConfig(sol_f))
#else
		if (!pp_.selfCollision(sol_f))
#endif
		{
			double2float(ik_sols_ + idx * num_joints_, config);

			solution_found = true;
		}
	}

	if (!solution_found)
	{
		solution_config_vec.clear();
		std::cout << "scan end pose not found\n";
		return solution_found;
	}

	// visualization
	std::cout << "scan end pose\n";
	pp_.computeReferencePointsOnArm(config, ref_points, rot_mats);
	pp_.getArmOBBModel(ref_points, rot_mats, arm_obbs); addOBBArmModelToViewer(arm_obbs);

	hand_translation = pose.col(3).head<3>() - optimal_hand_pose.col(3).head<3>();
	scan_start_or_probe_hand_pose = optimal_hand_pose;
	return solution_found;
}

void VisionArmCombo::getCurHandPose(Eigen::Matrix4f & pose)
{
	double array6[6];

	robot_arm_client_->getCartesianInfo(array6);

	array6ToEigenMat4(array6, pose);
}

void VisionArmCombo::getCurHandPoseD(Eigen::Matrix4d & pose)
{
	double array6[6];

	robot_arm_client_->getCartesianInfo(array6);

	array6ToEigenMat4d(array6, pose);
}

void VisionArmCombo::probeScannedSceneTest(PointCloudT::Ptr cloud)
{
	viewer_->removeAllPointClouds();
	viewer_->addPointCloud(cloud, "probeScannedCloud"+std::to_string(counter_), 0);

	while (true)
	{
		display();

		std::cout << "move hand?\n";

		std::string command;
		std::getline(std::cin, command);
		
		if (command == "y")
		{
			double pos[6];

			Eigen::Matrix4f base2hand;
			robot_arm_client_->getCartesianInfo(pos);
			array6ToEigenMat4(pos, base2hand);

			Eigen::Vector3f trans = pre_point_ - base2hand.topLeftCorner<3, 3>()*tool_center_point_;

			pos[0] = trans(0); pos[1] = trans(1); pos[2] = trans(2);

			// user defined pose
			Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
			transform.topLeftCorner<3,3>() = pre_viewer_pose_.matrix().cast<double>().block<3, 3>(0, 0);
			transform.col(3).head(3) = pre_point_.cast<double>();	
			transform = transform*probe_to_hand_;
			eigenMat4dToArray6(transform, pos);

			robot_arm_client_->moveHandL(pos, move_arm_acceleration_, move_arm_speed_);
		}
		else if (command == "q")
		{
			break;
		}
	}
}

void VisionArmCombo::smallClusterRemoval(PointCloudT::Ptr cloud_in, double clusterTolerance, int minClusterSize, PointCloudT::Ptr cloud_out)
{
	// Euclidean cluster, remove small clusters
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(clusterTolerance); //distance m
	ec.setMinClusterSize(1);
	ec.setMaxClusterSize(cloud_in->points.size());
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_in);
	ec.extract(cluster_indices);

	cloud_out->points.clear();

	for (int j = 0; j<cluster_indices.size(); j++)
	{
		if (cluster_indices[j].indices.size() > minClusterSize)
			for (int i = 0; i<cluster_indices[j].indices.size(); i++)
				cloud_out->push_back(cloud_in->points[cluster_indices[j].indices[i]]);
	}
}

void VisionArmCombo::setScanRadius(float radius)
{
	scan_start_to_hand_(1, 3) = -radius;
	scan_end_to_hand_(1, 3) = radius;
}

void VisionArmCombo::extractLeafProbingPointsAndHyperspectralScanPoses(PointCloudT::Ptr cloud_in, std::vector<pcl::PointXYZRGBNormal> & probe_pn_vec,
												std::vector<pcl::PointIndices> & leaf_cluster_indices_vec, 
												std::vector<std::vector<Eigen::Matrix4d*>> & hyperscan_hand_pose_sequence_vec,
												std::vector<std::vector<ArmConfig>> & hyperscan_arm_config_sequence_vec,
												std::vector<int> & hyperscan_leaf_id_vec, int plant_id)
{
	probe_pn_vec.clear();
	leaf_cluster_indices_vec.clear();
	hyperscan_hand_pose_sequence_vec.clear();
	hyperscan_arm_config_sequence_vec.clear();
	hyperscan_leaf_id_vec.clear();

	if (cloud_in->points.size() < 100)
	{
		std::cout << "not enough points\n";
		return;
	}

	sor_.setInputCloud(cloud_in);

	PointCloudT::Ptr cloud_clean(new PointCloudT);
	sor_.setMeanK(sor_mean_k_);
	sor_.setStddevMulThresh(sor_std_);
	sor_.filter(*cloud_clean);

	cloud_in->clear();
	*cloud_in += *cloud_clean;

	std::string save_path(data_saving_folder_.begin(), data_saving_folder_.end());
	save_path = save_path + "laser_scan_plant_" + std::to_string(plant_id) + ".pcd";

	pcl::io::savePCDFileBinary(save_path, *cloud_in);

	int64 t0 = cv::getTickCount();
	
	//region growing
	pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> >(new pcl::search::KdTree<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud_in);
	//normal_estimator.setKSearch(region_grow_num_neighbors_);
	normal_estimator.setRadiusSearch(normal_estimation_radius_);
	normal_estimator.compute(*normals);

	pcl::RegionGrowing<PointT, pcl::Normal> reg;
	reg.setMinClusterSize(region_grow_min_cluster_size_);
	reg.setMaxClusterSize(region_grow_max_cluster_size_);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(region_grow_num_neighbors_);
	reg.setInputCloud(cloud_in);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setResidualTestFlag(true);
	reg.setCurvatureTestFlag(false);
	reg.setSmoothModeFlag(true);
	reg.setResidualThreshold(region_grow_residual_threshold_);
	reg.setSmoothnessThreshold(region_grow_smoothness_threshold_);
	reg.setCurvatureThreshold(region_grow_curvature_threshold_);
	reg.extract(leaf_cluster_indices_vec);

	int64 t1 = cv::getTickCount();

	std::cout << "Region growing time: " << (t1 - t0) / cv::getTickFrequency() << "\n";

	std::cout << "num clusters found: " << leaf_cluster_indices_vec.size() << "\n";

	if (leaf_cluster_indices_vec.size() == 0) return;

	PointCloudT::Ptr colored_cloud = reg.getColoredCloud();
	viewer_->removeAllPointClouds();
	viewer_->addPointCloud(colored_cloud, "region", 0); viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "region");
	display();

	leaf_probing_pn_vector_.clear();

	leaf_cluster_order_.clear();
		
	for (int j = 0; j < leaf_cluster_indices_vec.size(); j++)
	{
		int cluster_size = leaf_cluster_indices_vec[j].indices.size();

		pcl::PointIndices::Ptr inliers(new pcl::PointIndices(leaf_cluster_indices_vec[j]));

#if 0
		for (int i = 0; i < cluster_size; i++)
		{
			int index = clusters[j].indices[i];

			Eigen::Vector3f point_normal(normals->points[index].normal_x, normals->points[index].normal_y, normals->points[index].normal_z);

			// make normal point up
			if (point_normal(2) < 0) point_normal *= -1.f;
			
			average_normal += point_normal;
		}

		average_normal.normalize();
#endif

#if 0
		// for each cluster, compute average normal direction
		Eigen::Vector3f average_normal(0.f, 0.f, 0.f);
		for (int i = 0; i < inliers->indices.size(); i++)
		{
			int index = inliers->indices[i];

			Eigen::Vector3f point_normal(normals->points[index].normal_x, normals->points[index].normal_y, normals->points[index].normal_z);

			// make normal point up
			if (point_normal(2) < 0) point_normal *= -1.f;

			average_normal += point_normal;
		}

		average_normal.normalize();
#endif
		//copy leaf segment point cloud for slicing
		PointCloudT::Ptr cloud(new PointCloudT);
		extract_indices_.setInputCloud(cloud_in);
		extract_indices_.setIndices(inliers);
		extract_indices_.setNegative(false);
		extract_indices_.filter(*cloud);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_fe(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::copyPointCloud(*cloud, *cloud_for_fe);

		pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor_;
		feature_extractor_.setInputCloud(cloud_for_fe);
		//feature_extractor_.setInputCloud(cloud_in);
		//feature_extractor_.setIndices(inliers);

		feature_extractor_.setAngleStep(400);
		feature_extractor_.compute();

		pcl::PointXYZ min_point_AABB;
		pcl::PointXYZ max_point_AABB;
		pcl::PointXYZ min_point_OBB;
		pcl::PointXYZ max_point_OBB;
		pcl::PointXYZ position_OBB;
		Eigen::Matrix3f rotational_matrix_OBB;
		float major_value, middle_value, minor_value;
		Eigen::Vector3f major_vector, middle_vector, minor_vector;
		Eigen::Vector3f mass_center;
		//feature_extractor_.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
		feature_extractor_.getEigenValues(major_value, middle_value, minor_value);
		feature_extractor_.getEigenVectors(major_vector, middle_vector, minor_vector);
		feature_extractor_.getMassCenter(mass_center);

		
#if 1	//extract imaging poses for hyperspectral camera
		if (hypercam_ != NULL) {

			Eigen::Matrix4f transform_to_origin = Eigen::Matrix4f::Identity();

			transform_to_origin.col(0).head(3) = major_vector;
			transform_to_origin.col(1).head(3) = middle_vector;
			transform_to_origin.col(2).head(3) = minor_vector;
			transform_to_origin.col(3).head(3) = mass_center;

			float cosine = std::cos(30./180.*M_PI);
			float sine = std::sin(30. / 180.*M_PI);

			if (std::abs(major_vector.dot( Eigen::Vector3f::UnitX() )) < cosine) {

				 Eigen::Vector3f tmp_dir(cosine, sine, 0.);

				 transform_to_origin.col(0).head(3) = (tmp_dir - tmp_dir.dot(minor_vector)*minor_vector).normalized();

				 transform_to_origin.col(1).head(3) = transform_to_origin.col(2).head<3>().cross(transform_to_origin.col(0).head<3>());
			}

			transform_to_origin = transform_to_origin.inverse();

			PointCloudT::Ptr cloud_origin(new PointCloudT);

			pcl::transformPointCloud(*cloud, *cloud_origin, transform_to_origin);

			viewer_->removeAllPointClouds();
			viewer_->removeAllShapes();
			viewer_->removeAllCoordinateSystems();
			//viewer_->addPointCloud(cloud_origin, std::to_string(cv::getTickCount()));

			// slice the leaf segment
			Eigen::Vector4f min_pt, max_pt;
			pcl::getMinMax3D(*cloud_origin, min_pt, max_pt);
			float length = max_pt(0) - min_pt(0);

			pcl::PassThrough<PointT> pass;
			pass.setFilterFieldName("x");
			pass.setInputCloud(cloud_origin);

			std::vector<std::vector<int>> slices_indices;

			float step_size = 0.005f;
			float half_slice_width = hyperscan_slice_thickness_;

			for (float x = min_pt(0) + half_slice_width; x <= length - half_slice_width; x += step_size) {

				std::vector<int> indices;
				pass.setFilterLimits(x - half_slice_width, x + half_slice_width);
				pass.filter(indices);
				slices_indices.push_back(indices);
			}

			int cnt = 0;
			for (auto & indices : slices_indices) {

				uint32_t rgb;

				if (cnt % 2 == 0)
					rgb = (uint32_t)255 << 16;
				else
					rgb = (uint32_t)255 << 8;

				float rgb_f = *reinterpret_cast<float*>(&rgb);

				for (auto & id : indices) {

					cloud->points[id].rgb = rgb_f;
				}

				++cnt;
			}

			pcl::PCA<pcl::PointXYZRGB> pca;

			pca.setInputCloud(cloud);

			cnt = 0;

			viewer_->addPointCloud(cloud, std::to_string(cv::getTickCount()));
			viewer_->addCoordinateSystem(0.3);

			display();

			std::vector<Eigen::Matrix4f*> scanner_poses;

			pcl::KdTreeFLANN<PointT> kdtree;
			kdtree.setInputCloud(cloud);

			for (auto & indices : slices_indices) {

				if (indices.size() < 10)
					continue;

				pcl::PointIndices::Ptr slice_indices(new pcl::PointIndices);

				slice_indices->indices = indices;

				pca.setIndices(slice_indices);

				Eigen::Matrix3f eigenvectors = pca.getEigenVectors();

				Eigen::Vector3f mean = pca.getMean().head(3);

				std::vector<int> pointIdxNKNSearch(1);
				std::vector<float> pointNKNSquaredDistance(1);
				PointT searchPoint;
				searchPoint.x = mean(0);
				searchPoint.y = mean(1);
				searchPoint.z = mean(2);

				if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
				{
					PointT p0, p1, p2;
					p0 = cloud->points[pointIdxNKNSearch[0]];

					Eigen::Matrix4f *scanner_pose_ptr(new Eigen::Matrix4f);

					*scanner_pose_ptr = Eigen::Matrix4f::Identity();

					// save scan pose (translations and z axis)
					if (eigenvectors(2, 2) > 0)
						scanner_pose_ptr->col(2).head(3) = -1.0f*eigenvectors.col(2);
					else
						scanner_pose_ptr->col(2).head(3) = eigenvectors.col(2);

					scanner_pose_ptr->col(3).head(3) = cloud->points[pointIdxNKNSearch[0]].getVector3fMap();

					scanner_poses.push_back(scanner_pose_ptr);

#if 0
					float sign = eigenvectors(2, 2) >= 0 ? 1.0f : -1.0f;

					p1.x = p0.x + eigenvectors(0, 2)*0.02f*sign;
					p1.y = p0.y + eigenvectors(1, 2)*0.02f*sign;
					p1.z = p0.z + eigenvectors(2, 2)*0.02f*sign;

					viewer_->addLine(p0, p1, 0, 0, 1, "l" + std::to_string(cv::getTickCount()));

					// x axis of scanner frame
					sign = eigenvectors(0, 0) >= 0 ? 1.0f : -1.0f;

					p2.x = p0.x + eigenvectors(0, 0)*0.02f*sign;
					p2.y = p0.y + eigenvectors(1, 0)*0.02f*sign;
					p2.z = p0.z + eigenvectors(2, 0)*0.02f*sign;

					viewer_->addLine(p0, p2, 0, 1, 0, "l" + std::to_string(cv::getTickCount()));
#endif
				}
			}

			std::vector<Eigen::Matrix4d*> valid_scan_hand_pose_sequence;

			std::vector<ArmConfig> valid_arm_config_sequence;

			if (scanner_poses.size() > 1) {

				for (int pose_id = 0; pose_id < scanner_poses.size() - 1; ++pose_id) {

					// leaf midrib direction
					//Eigen::Vector3f leaf_midrib_dir = scanner_poses[pose_id + 1]->col(3).head(3) - scanner_poses[pose_id - 1]->col(3).head(3);
					Eigen::Vector3f leaf_midrib_dir = scanner_poses.back()->col(3).head(3) - scanner_poses.front()->col(3).head(3);

					leaf_midrib_dir.normalize();

					// project 
					Eigen::Vector3f leaf_midrib_dir_proj = leaf_midrib_dir - leaf_midrib_dir.dot(scanner_poses[pose_id]->col(2).head(3))*scanner_poses[pose_id]->col(2).head(3);

					//scanner y dir should point towards chamber
					if (leaf_midrib_dir_proj(0) < 0) leaf_midrib_dir_proj *= -1.f;

					scanner_poses[pose_id]->col(1).head(3) = leaf_midrib_dir_proj.normalized();

					scanner_poses[pose_id]->col(0).head(3) = scanner_poses[pose_id]->col(1).head<3>().cross(scanner_poses[pose_id]->col(2).head<3>());

					// retract 
					scanner_poses[pose_id]->col(3).head(3) -= hyperspectral_imaging_dist_*scanner_poses[pose_id]->col(2).head(3);

					Eigen::Matrix4d* hand_pose(new Eigen::Matrix4d);
					*hand_pose = scanner_poses[pose_id]->cast<double>()*hand_to_hyperspectral_.cast<double>().inverse();

					Eigen::Affine3f affine_pose;
					affine_pose.matrix() = *scanner_poses[pose_id];

					viewer_->addCoordinateSystem(hyperspectral_imaging_dist_, affine_pose, std::to_string(cv::getTickCount()));

				//	display();

				//	continue;

					std::vector<int> ik_sols_vec;

					inverseKinematics(*hand_pose, ik_sols_vec, IMAGING);

					//if (ik_sols_vec.size() == 0) std::cout << "no ik solution\n";

					cnt = 0;

					for (auto idx : ik_sols_vec)
					{
						float sol_f[6];

						double2float(ik_sols_ + idx * num_joints_, sol_f);

						std::cout << "solution: " << cnt << "\n";

#ifdef ENABLE_PP
						if (!pp_.collisionCheckForSingleConfig(sol_f))
#else
						if (!pp_.selfCollision(sol_f))
#endif
						{
							//std::cout << "no collision\n";

							ArmConfig config;
							config.setJointPos(sol_f[0], sol_f[1], sol_f[2], sol_f[3], sol_f[4], sol_f[5]);
#if 0
							// visualization
							std::vector<PathPlanner::RefPoint> ref_points;
							std::vector<Eigen::Matrix3f> rot_mats;
							std::vector<PathPlanner::OBB> arm_obbs;
							pp_.computeReferencePointsOnArm(config.joint_pos_f, ref_points, rot_mats);
							pp_.getArmOBBModel(ref_points, rot_mats, arm_obbs); addOBBArmModelToViewer(arm_obbs);
#endif
							valid_scan_hand_pose_sequence.push_back(hand_pose);
							valid_arm_config_sequence.push_back(config);

							//robot_arm_client_->moveHandJ(ik_sols_ + idx * num_joints_, 0.4, 0.4, true);
							//double array6[6];
							//eigenMat4dToArray6(*hand_pose, array6);
							//robot_arm_client_->moveHandL(array6, 0.1, 0.05);
						}
						else std::cout << "self collision\n";

						cnt++;
					}
				}
			}

			if (valid_scan_hand_pose_sequence.size() != 0) {

				hyperscan_hand_pose_sequence_vec.push_back(valid_scan_hand_pose_sequence);
				hyperscan_arm_config_sequence_vec.push_back(valid_arm_config_sequence);
				hyperscan_leaf_id_vec.push_back(j);
			}

			//scanLeafWithHyperspectral(valid_scan_hand_pose_sequence, valid_arm_config_sequence);
		}
#endif
		
		//viewer_->spin();

	//	if (mass_center(2) < -0.49f) continue;

		LeafIDnX leaf_id_x;
		leaf_id_x.id = j;
		leaf_id_x.x = mass_center(0);
		leaf_cluster_order_.push_back(leaf_id_x);

		//std::cout << major_value << " " << middle_value << " " << minor_value << "\n";

		float elongation = major_value / middle_value;

		//std::cout << "elongation " << elongation << "\n\n";

		// keep long leaf
		/*if (elongation < 5.f)k
		{
			//std::cout << "not long enough\n";
			continue;
		}*/

		std::vector<pcl::Supervoxel<PointT>::Ptr> potential_probe_supervoxels;
		std::vector<pcl::PointXYZRGBNormal> probing_point_normal_vec;

#if 1
		t0 = cv::getTickCount();
		extractProbeSurfacePatchFromPointCloud(cloud, potential_probe_supervoxels, probing_point_normal_vec);
		t1 = cv::getTickCount();
		std::cout << "Supervoxel time: " << (t1 - t0) / cv::getTickFrequency() << "\n";
#endif
		
		int i = 0;
		for (auto & pn0 : probing_point_normal_vec)
		{
			pcl::ModelCoefficients line; line.values.resize(6);
			line.values[0] = pn0.x;
			line.values[1] = pn0.y;
			line.values[2] = pn0.z;
			float sign = pn0.normal_z >= 0 ? 1.0f : -1.0f;
			line.values[3] = pn0.normal_x*0.02f*sign;
			line.values[4] = pn0.normal_y*0.02f*sign;
			line.values[5] = pn0.normal_z*0.02f*sign;

			viewer_->addLine(line, "l" + std::to_string(cv::getTickCount()), 0);

			pcl::ModelCoefficients sphere; sphere.values.resize(4);
			sphere.values[0] = pn0.x; sphere.values[1] = pn0.y; sphere.values[2] = pn0.z;
			sphere.values[3] = 0.002;

			viewer_->addSphere(sphere, "sphere"+std::to_string(cv::getTickCount()), 0);
		}

		//display();

		//viewer_->removeAllShapes();

		leaf_probing_pn_vector_.push_back(probing_point_normal_vec);
		
		//probe_pn_vec.push_back(pn);
		probe_pn_vec.insert(probe_pn_vec.end(), probing_point_normal_vec.begin(), probing_point_normal_vec.end());
	}

	std::sort(leaf_cluster_order_.begin(), leaf_cluster_order_.end(), VisionArmCombo::leaf_x_comparator);

	for (int leaf_idx = 0; leaf_idx < leaf_probing_pn_vector_.size(); leaf_idx++)
	{
		std::sort(leaf_probing_pn_vector_[leaf_idx].begin(), leaf_probing_pn_vector_[leaf_idx].end(), VisionArmCombo::probing_rank_comparator);
	}

	showOccupancyGrid();
	display();
}


bool VisionArmCombo::probeLeaf(PointT & probe_point, pcl::Normal & normal, int probe_id, int plant_id, int point_index)
{
	std::vector<ArmConfig> solution_config_vec_refine;
	Eigen::Matrix4d probe_hand_pose_final;
	Eigen::Vector3d hand_translation_refine;

	//std::cout << "try to probe a leaf\n";

	if (probe_id == PAM)
		probe_to_hand_ = probe_to_hand_pam_;
	else if (probe_id == RAMAN_532)
		probe_to_hand_ = probe_to_hand_raman_532_;
	else if (probe_id == RAMAN_1064)
		probe_to_hand_ = probe_to_hand_raman_1064_;

	// try probe
	if (computeCollisionFreeProbeOrScanPose(probe_point, normal, PROBE, solution_config_vec_refine, probe_hand_pose_final, hand_translation_refine))
	{
		/*std::cout << "probe?\n";
		std::string key;
		std::getline(std::cin, key);
		if (key == "n")	return false;*/
		

		//pp_.forwardKinematicsUR10(solution_config_vec_refine[0].joint_pos_f);

		//std::cout << "my fk:\n" << pp_.fk_mat_ << "\n";
		//pp_.forwardKinematicsUR10ROS(solution_config_vec_refine[0].joint_pos_f);

		/*Eigen::Vector3f probe_point_eigen(probe_point.x, probe_point.y, probe_point.z);
		Eigen::Vector3f probe_normal_eigen(normal.normal_x, normal.normal_y, normal.normal_z);
		
		std::cout << "point: "<<pp_.fk_mat_.col(3).head(3)-probe_point_eigen<<"\n"<<"normal: "<< pp_.fk_mat_.col(2).head(3)<< " "<<probe_normal_eigen << "\n";*/


		// compute the hand pose with probe 10cm away from the target
	//	Eigen::Matrix4d final_to_prepare = Eigen::Matrix4d::Identity();
	//	final_to_prepare(2, 3) = 0.0;
	//	Eigen::Matrix4d probe_hand_pose_prepare = probe_hand_pose_final*probe_to_hand_.inverse()*final_to_prepare*probe_to_hand_;

		Eigen::Matrix4d probe_hand_pose_prepare = probe_hand_pose_final;
		probe_hand_pose_prepare.col(3).head<3>() += probe_hand_pose_prepare.col(2).head<3>()*0.08;	// plus: probe in opposite direction of hand

		//std::cout << "probe_hand_pose_final\n" << probe_hand_pose_final << "\n";

		//std::cout << "probe_hand_pose_prepare\n" << probe_hand_pose_prepare << "\n";

#if 0
		// visualization
		std::cout << "showing prepare probe pose\n";
		Eigen::Affine3f affine_pose;
		affine_pose.matrix() = probe_hand_pose_prepare.cast<float>();
		viewer_->removeAllCoordinateSystems();
		viewer_->addCoordinateSystem(0.2, affine_pose, "prepare probe pose", 0);
		display();
#endif

		// ik
		std::vector<int> ik_sols_vec;

		inverseKinematics(probe_hand_pose_prepare, ik_sols_vec, PROBING);

		bool solution_found = false;

		float config[6];

		ArmConfig prepare_probe_config;

		for (auto idx : ik_sols_vec)
		{
			float sol_f[6];

			double2float(ik_sols_ + idx * num_joints_, sol_f);

#ifdef ENABLE_PP
			if (!pp_.collisionCheckForSingleConfig(sol_f))
#else
			if (!pp_.selfCollision(sol_f))
#endif
			{
				double2float(ik_sols_ + idx * num_joints_, config);

				prepare_probe_config.setJointPos(ik_sols_[idx*num_joints_], ik_sols_[idx*num_joints_ + 1],
													ik_sols_[idx*num_joints_ + 2], ik_sols_[idx*num_joints_ + 3],
														ik_sols_[idx*num_joints_ + 4], ik_sols_[idx*num_joints_ + 5]);

				solution_found = true;
			}
		}

		if (!solution_found)
		{
			std::cout << "probe_hand_pose_prepare ik solution not found\n";
			return false;
		}

		//std::cout << "try to move to prepare probe pose\n";

		if (!moveToConfigGetKinectPointCloud(prepare_probe_config, false, true))
		{
			std::cout << "could not reach prepare probe pose\n";
			return false;
		}

		Eigen::Vector3f point_on_leaf(probe_point.x, probe_point.y, probe_point.z);
		Eigen::Vector3f trans = point_on_leaf - probe_hand_pose_final.topLeftCorner<3, 3>().cast<float>()*tool_center_point_;

		double pose[6];
		Eigen::Matrix4d transform;
		getCurHandPoseD(transform);	//need to get current pose because of the small difference between fk solution using joint values and UR current pose
		
		//save initial pose
		double tmp_pose[6];
		eigenMat4dToArray6(transform, tmp_pose);

		Eigen::Vector3d dir = transform.col(3).head(3);
		
		transform.col(3).head(3) = point_on_leaf.cast<double>();
		transform = transform*probe_to_hand_;

		dir = transform.col(3).head(3) - dir;

		//eigenMat4dToArray6(transform, pose);
		eigenMat4dToArray6(probe_hand_pose_final, pose);

		double step_size = 0.001;
		double norm = dir.norm();

		dir.normalize();

		double length;

		// probe block test
	/*	for (length = step_size; length < norm; length += step_size)
		{
			tmp_pose[0] += step_size*dir[0]; tmp_pose[1] += step_size*dir[1]; tmp_pose[2] += step_size*dir[2];
			robot_arm_client_->moveHandL(tmp_pose, 0.002, 0.002);
			//robot_arm_client_->waitTillHandReachDstPose(pose);
			std::cout << "length " << length << " norm " << norm << "\n";
			std::getchar();
		}
		*/

		robot_arm_client_->moveHandL(pose, 0.04, 0.04);
		//robot_arm_client_->moveHandL(pose, 0.005, 0.005);
		//robot_arm_client_->waitTillHandReachDstPose(pose);

		Sleep(1000);


	/*	std::cout << "Done?\n";

		std::getline(std::cin, key);

		if (key == "n")
		{
			for (length = norm+step_size; length < norm+0.01; length += step_size)
			{
				tmp_pose[0] += step_size*dir[0]; tmp_pose[1] += step_size*dir[1]; tmp_pose[2] += step_size*dir[2];
				robot_arm_client_->moveHandL(tmp_pose, 0.002, 0.002);
				//robot_arm_client_->waitTillHandReachDstPose(pose);
				std::cout << "length " << length << " norm+0.01 " << norm+0.01 << "\n";
				std::getchar();
			}
		}*/

		std::string save_path(data_saving_folder_.begin(), data_saving_folder_.end());

		std::string time = getCurrentDateTimeStr();

		cv::Mat probing_point_cv; probing_point_cv.create(3, 1, CV_32F);

		probing_point_cv.at<float>(0, 0) = probe_point.x;
		probing_point_cv.at<float>(1, 0) = probe_point.y;
		probing_point_cv.at<float>(2, 0) = probe_point.z;

		cv::Mat probing_point_normal_cv; probing_point_normal_cv.create(3, 1, CV_32F);

		probing_point_normal_cv.at<float>(0, 0) = normal.normal_x;
		probing_point_normal_cv.at<float>(1, 0) = normal.normal_y;
		probing_point_normal_cv.at<float>(2, 0) = normal.normal_z;
		
		if (probe_id == PAM) {

#if 0
			float Fo, Fm, FvFm, qP, qL, qN, NPQ, YNPQ, YNO, F, FmPrime, PAR, YII, ETR, FoPrime;

			if (MiniPamActPlusYield(Fo, Fm, FvFm, qP, qL, qN, NPQ, YNPQ, YNO, F, FmPrime, PAR, YII, ETR, FoPrime) == 0)
			{
				std::string pam_file = save_path + "PAM_plant_" + std::to_string(plant_id) + "_time_" + time + ".yml";

				cv::FileStorage fs(pam_file, cv::FileStorage::WRITE);

				fs << "probing_point" << probing_point_cv;
				fs << "probing_normal" << probing_point_normal_cv;
				fs << "Fo" << Fo; fs << "Fm" << Fm; fs << "FvFm" << FvFm;
				fs << "qP" << qP; fs << "qL" << qL; fs << "qN" << qN;
				fs << "NPQ" << NPQ; fs << "YNPQ" << YNPQ; fs << "YNO" << YNO;
				fs << "F" << F; fs << "FmPrime" << FmPrime; fs << "PAR" << PAR;
				fs << "YII" << YII; fs << "ETR" << ETR; fs << "FoPrime" << FoPrime;

				fs.release();
			}
#endif
			float PAR, YII;

			if (MiniPamBasicData(PAR, YII) == 0)
			{
				std::string pam_file = save_path + "PAM_plant_" + std::to_string(plant_id) + "_time_" + time + ".yml";

				cv::FileStorage fs(pam_file, cv::FileStorage::WRITE);

				fs << "probing_point" << probing_point_cv;
				fs << "probing_normal" << probing_point_normal_cv;
				fs << "YII" << YII;
				fs << "PAR" << PAR;

				fs.release();
			}
		}
		else if (probe_id == RAMAN_1064 || probe_id == RAMAN_532) {

			std::string wavelength_str;
			
			int status = -1;

			if (probe_id == RAMAN_1064) {
				status = raman_->getSpectrum(1, 7000);
				wavelength_str = "1064";
			}
			else {
				status = raman_->getSpectrum(0, 7000);
				wavelength_str = "532";
			}

			if (status == 0) {

				std::string raman_csv = save_path + "raman_" + wavelength_str + "_plant_" + std::to_string(plant_id) + "_time_" + time + ".csv";

				raman_->saveLastSpectrum(raman_csv);

				std::string raman_pose_path = save_path + "raman_" + wavelength_str + "_plant_" + std::to_string(plant_id) + "_time_" + time + ".yml";

				cv::FileStorage fs(raman_pose_path, cv::FileStorage::WRITE);

				fs << "probing_point" << probing_point_cv;

				fs << "probing_normal" << probing_point_normal_cv;

				fs.release();
			}
		}

		display();

		//robot_arm_client_->moveHandJ(solution_config_vec_refine[0].joint_pos_d, move_joint_speed_, move_joint_acceleration_, true);

		robot_arm_client_->moveHandJ(prepare_probe_config.joint_pos_d, move_joint_speed_, move_joint_acceleration_*0.5, true);

		return true;
	}

	return false;
}

void VisionArmCombo::display()
{
	if (view_time_ == 0) viewer_->spin();
	else viewer_->spinOnce(view_time_);
}

void VisionArmCombo::calibrateKinectRGBCamera()
{
	if (robot_arm_client_ == NULL) initRobotArmClient();
	if (kinect_thread_ == NULL)	initKinectThread();

	int nframes = 20;

	if (nframes % 2 != 0 || nframes < 6)
	{
		std::cout << "number of frames not even or not enough\n";
		return;
	}

	cv::Mat cameraMatrix, distCoeffs;
	std::vector<cv::Mat> image_vec;
	std::vector<Eigen::Matrix4d*> tcp_pose_vec; tcp_pose_vec.resize(nframes);
	cv::Size boardSize, imageSize;
	float squareSize, aspectRatio;

	std::vector<std::vector<cv::Point2f>> imagePoints;

	// IMPORTANT
	cv::SimpleBlobDetector::Params params;
	params.maxArea = 200 * 200;
	params.minArea = 20 * 20;
	cv::Ptr<cv::FeatureDetector> blobDetect = cv::SimpleBlobDetector::create(params);

	imageSize.width = kinect_thread_->cColorWidth;
	imageSize.height = kinect_thread_->cColorHeight;

	boardSize.width = 4;
	boardSize.height = 11;

	squareSize = 0.02f;

	for (int i = 0; i < nframes; i++)
	{
		cv::Mat view, viewGray;

		std::vector<cv::Point2f> pointbuf;

		while (true)
		{
			view = kinect_thread_->getCurRGB();

			cv::cvtColor(view, viewGray, CV_BGR2GRAY);

			pointbuf.clear();

			// ASYMMETRIC CIRCLE GRID PATTERN
			bool found = findCirclesGrid(view, boardSize, pointbuf, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetect);

			cv::Mat view_copy;
			
			view.copyTo(view_copy);

			if (found)
			{
				cv::drawChessboardCorners(view_copy, boardSize, cv::Mat(pointbuf), found);
				cv::circle(view_copy, pointbuf[0], 30, cv::Scalar(0, 255, 0), 4);
				cv::circle(view_copy, pointbuf[1], 30, cv::Scalar(0, 0, 255), 4);
			}

			cv::Mat shrinked;

			cv::resize(view_copy, shrinked, cv::Size(), 0.5, 0.5);

			cv::imshow("rgb", shrinked);

			int key = cv::waitKey(1);

			//hit space
			if (key == 32)	break;
		}

		std::cout << "Image " << i << " done\n";

		cv::Mat view_save;

		view.copyTo(view_save);

		image_vec.push_back(view_save);

		imagePoints.push_back(pointbuf);

		double array6[6];

		robot_arm_client_->getCartesianInfo(array6);

		Eigen::Matrix4d* tcp_pose = new Eigen::Matrix4d;

		array6ToEigenMat4d(array6, *tcp_pose);

		std::cout << *tcp_pose << "\n";

		tcp_pose_vec[i] = tcp_pose;
	}

	cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

	distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

	std::vector<std::vector<cv::Point3f>> objectPoints(1);

	std::vector<cv::Point3f> corners;

	for (int i = 0; i < boardSize.height; i++)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(cv::Point3f(float((2 * j + i % 2)*squareSize), float(i*squareSize), 0));
		}
	}

	objectPoints[0] = corners;

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	std::vector<cv::Mat> camera_rotation_vec, camera_translation_vec;	//camera to calibration pattern

	double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, camera_rotation_vec, camera_translation_vec, 0);

	printf("RMS error reported by calibrateCamera: %g\n", rms);

	/*for (int i = 0; i < image_vec.size(); i++)
	{
		std::cout << "rotation\n" << camera_rotation_vec[i] << "\ntranslation\n" << camera_translation_vec[i]<<"\n";
		cv::imshow("image", image_vec[i]);
		cv::waitKey(0);
	}*/

	bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);

	cv::FileStorage fs("kinectRGBCalibration.yml", cv::FileStorage::WRITE);

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;
	fs << "nframes" << nframes;
	fs << "camera poses" << "[";
	for (int i = 0; i < nframes; i++)
	{
		cv::Mat rotation;
		cv::Rodrigues(camera_rotation_vec[i], rotation);
		cv::Mat transform = cv::Mat::eye(4, 4,CV_64F);
		cv::Mat sub = transform(cv::Rect(0, 0, 3, 3));
		rotation.copyTo(sub);
		sub = transform(cv::Rect(3, 0, 1, 3));
		camera_translation_vec[i].copyTo(sub);

		fs << transform;
	}
	fs << "]";

	fs << "TCP poses" << "[";
	for (int i = 0; i < nframes; i++)
	{
		cv::Mat tcp_pose(4, 4, CV_64F);
		for (int y = 0; y < 4; y++)
			for (int x = 0; x < 4; x++)
				tcp_pose.at<double>(y, x) = (*tcp_pose_vec[i])(y, x);

		fs << tcp_pose;
	}
	fs << "]";

	fs.release();
}

void VisionArmCombo::calibrateThermalCamera() {

	if (robot_arm_client_ == NULL) initRobotArmClient();
	if (!thermocam_->isConnected)	initThermoCam();

	int nframes = 30;

	if (nframes % 2 != 0 || nframes < 6)
	{
		std::cout << "number of frames not even or not enough\n";
		return;
	}

	cv::Mat cameraMatrix, distCoeffs;
	std::vector<cv::Mat> image_vec;
	std::vector<Eigen::Matrix4d*> tcp_pose_vec; tcp_pose_vec.resize(nframes);
	cv::Size boardSize, imageSize;
	float squareSize, aspectRatio;

	std::vector<std::vector<cv::Point2f>> imagePoints;

	// IMPORTANT
	cv::SimpleBlobDetector::Params params;
	params.maxArea = 200 * 200;
	params.minArea = 20 * 20;
	cv::Ptr<cv::FeatureDetector> blobDetect = cv::SimpleBlobDetector::create(params);

	imageSize.width = 640;
	imageSize.height = 480;

	boardSize.width = 4;
	boardSize.height = 11;

	squareSize = 0.02f;

	for (int i = 0; i < nframes; i++)
	{
		cv::Mat view, viewGray;

		std::vector<cv::Point2f> pointbuf;

		while (true)
		{
			cv::Mat temp_map;

			thermocam_->snapShot(view, temp_map);

			cv::cvtColor(view, viewGray, CV_BGR2GRAY);

			viewGray = 255 - viewGray;

			pointbuf.clear();

			// ASYMMETRIC CIRCLE GRID PATTERN
			bool found = findCirclesGrid(viewGray, boardSize, pointbuf, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetect);

			cv::Mat view_copy;

			view.copyTo(view_copy);

			if (found)
			{
				cv::drawChessboardCorners(view_copy, boardSize, cv::Mat(pointbuf), found);
				cv::circle(view_copy, pointbuf[0], 30, cv::Scalar(0, 255, 0), 4);
				cv::circle(view_copy, pointbuf[1], 30, cv::Scalar(0, 0, 255), 4);
			}

			cv::imshow("rgb", view_copy);

			int key = cv::waitKey(1);

			//hit space
			if (key == 32)	break;
		}

		std::cout << "Image " << i << " done\n";

		cv::Mat view_save;

		view.copyTo(view_save);

		image_vec.push_back(view_save);

		imagePoints.push_back(pointbuf);

		double array6[6];

		robot_arm_client_->getCartesianInfo(array6);

		Eigen::Matrix4d* tcp_pose = new Eigen::Matrix4d;

		array6ToEigenMat4d(array6, *tcp_pose);

		std::cout << *tcp_pose << "\n";

		tcp_pose_vec[i] = tcp_pose;
	}

	cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

	distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

	std::vector<std::vector<cv::Point3f>> objectPoints(1);

	std::vector<cv::Point3f> corners;

	for (int i = 0; i < boardSize.height; i++)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(cv::Point3f(float((2 * j + i % 2)*squareSize), float(i*squareSize), 0));
		}
	}

	objectPoints[0] = corners;

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	std::vector<cv::Mat> camera_rotation_vec, camera_translation_vec;	//camera to calibration pattern

	double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, camera_rotation_vec, camera_translation_vec, 0);

	printf("RMS error reported by calibrateCamera: %g\n", rms);

	/*for (int i = 0; i < image_vec.size(); i++)
	{
	std::cout << "rotation\n" << camera_rotation_vec[i] << "\ntranslation\n" << camera_translation_vec[i]<<"\n";
	cv::imshow("image", image_vec[i]);
	cv::waitKey(0);
	}*/

	bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);

	cv::FileStorage fs("FlirThermalCalibration.yml", cv::FileStorage::WRITE);

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;
	fs << "nframes" << nframes;
	fs << "camera poses" << "[";
	for (int i = 0; i < nframes; i++)
	{
		cv::Mat rotation;
		cv::Rodrigues(camera_rotation_vec[i], rotation);
		cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
		cv::Mat sub = transform(cv::Rect(0, 0, 3, 3));
		rotation.copyTo(sub);
		sub = transform(cv::Rect(3, 0, 1, 3));
		camera_translation_vec[i].copyTo(sub);

		fs << transform;
	}
	fs << "]";

	fs << "TCP poses" << "[";
	for (int i = 0; i < nframes; i++)
	{
		cv::Mat tcp_pose(4, 4, CV_64F);
		for (int y = 0; y < 4; y++)
			for (int x = 0; x < 4; x++)
				tcp_pose.at<double>(y, x) = (*tcp_pose_vec[i])(y, x);

		fs << tcp_pose;
	}
	fs << "]";

	fs.release();

}

void VisionArmCombo::calibrateKinectIRCamera()
{
	if (robot_arm_client_ == NULL) initRobotArmClient();
	if (kinect_thread_ == NULL)	initKinectThread();

	kinect_thread_->stream_infrared_ = true;

	int nframes = 30;

	if (nframes % 2 != 0 || nframes < 6)
	{
		std::cout << "number of frames not even or not enough\n";
		return;
	}

	cv::Mat cameraMatrix, distCoeffs;
	std::vector<cv::Mat> image_vec;
	std::vector<Eigen::Matrix4d*> tcp_pose_vec; tcp_pose_vec.resize(nframes);
	cv::Size boardSize, imageSize;
	float squareSize, aspectRatio;

	std::vector<std::vector<cv::Point2f>> imagePoints;

	// IMPORTANT
	cv::SimpleBlobDetector::Params params;
	params.maxArea = 50 * 50;
	params.minArea = 5 * 5;
	cv::Ptr<cv::FeatureDetector> blobDetect = cv::SimpleBlobDetector::create(params);

	imageSize.width = kinect_thread_->cDepthWidth;
	imageSize.height = kinect_thread_->cDepthHeight;

	boardSize.width = 4;
	boardSize.height = 11;

	squareSize = 0.02f;

	for (int i = 0; i < nframes; i++)
	{
		cv::Mat view, viewGray;

		std::vector<cv::Point2f> pointbuf;

		while (true)
		{
			view = kinect_thread_->getCurIR();

			cv::normalize(view, viewGray, 0, 255, cv::NORM_MINMAX, CV_8UC1);

			pointbuf.clear();

			// ASYMMETRIC CIRCLE GRID PATTERN
			bool found = findCirclesGrid(viewGray, boardSize, pointbuf, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetect);

			cv::Mat view_copy;

			view.copyTo(view_copy);

			if (found)
			{
				cv::drawChessboardCorners(view_copy, boardSize, cv::Mat(pointbuf), found);
				cv::circle(view_copy, pointbuf[0], 10, cv::Scalar(0, 255, 0), 2);
				cv::circle(view_copy, pointbuf[1], 10, cv::Scalar(0, 0, 255), 2);
			}

			cv::imshow("IR", view_copy);

			int key = cv::waitKey(1);

			//hit space
			if (key == 32 && found)	break;
		}

		std::cout << "Image " << i << " done\n";

		cv::Mat view_save;

		view.copyTo(view_save);

		image_vec.push_back(view_save);

		imagePoints.push_back(pointbuf);

		double array6[6];

		robot_arm_client_->getCartesianInfo(array6);

		Eigen::Matrix4d* tcp_pose = new Eigen::Matrix4d;

		array6ToEigenMat4d(array6, *tcp_pose);

		std::cout << *tcp_pose << "\n";

		tcp_pose_vec[i] = tcp_pose;
	}

	cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

	distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

	std::vector<std::vector<cv::Point3f>> objectPoints(1);

	std::vector<cv::Point3f> corners;

	for (int i = 0; i < boardSize.height; i++)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(cv::Point3f(float((2 * j + i % 2)*squareSize), float(i*squareSize), 0));
		}
	}

	objectPoints[0] = corners;

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	std::vector<cv::Mat> camera_rotation_vec, camera_translation_vec;	//camera to calibration pattern

	double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, camera_rotation_vec, camera_translation_vec, 0);

	printf("RMS error reported by calibrateCamera: %g\n", rms);

	/*for (int i = 0; i < image_vec.size(); i++)
	{
	std::cout << "rotation\n" << camera_rotation_vec[i] << "\ntranslation\n" << camera_translation_vec[i]<<"\n";
	cv::imshow("image", image_vec[i]);
	cv::waitKey(0);
	}*/

	bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);

	cv::FileStorage fs("kinectIRCalibration.yml", cv::FileStorage::WRITE);

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;
	fs << "nframes" << nframes;
	fs << "camera poses" << "[";
	for (int i = 0; i < nframes; i++)
	{
		cv::Mat rotation;
		cv::Rodrigues(camera_rotation_vec[i], rotation);
		cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
		cv::Mat sub = transform(cv::Rect(0, 0, 3, 3));
		rotation.copyTo(sub);
		sub = transform(cv::Rect(3, 0, 1, 3));
		camera_translation_vec[i].copyTo(sub);

		fs << transform;
	}
	fs << "]";

	fs << "TCP poses" << "[";
	for (int i = 0; i < nframes; i++)
	{
		cv::Mat tcp_pose(4, 4, CV_64F);
		for (int y = 0; y < 4; y++)
			for (int x = 0; x < 4; x++)
				tcp_pose.at<double>(y, x) = (*tcp_pose_vec[i])(y, x);

		fs << tcp_pose;
	}
	fs << "]";

	fs.release();
}

/*
	robot sensor calibration solving ax=xb on euclidean group 1994
*/
void VisionArmCombo::KinectRGBHandEyeCalibration()
{
	cv::FileStorage fs("kinectRGBCalibration.yml", cv::FileStorage::READ);
	cv::FileNode camera_poses = fs["camera poses"];
	cv::FileNode tcp_poses = fs["TCP poses"];

	int nframes;

	fs["nframes"] >> nframes;

	std::vector<Eigen::Matrix4d*> camera_pose_vec; 
	std::vector<Eigen::Matrix4d*> tcp_pose_vec;	

	// iterate through a sequence using FileNodeIterator
	for (cv::FileNodeIterator it = camera_poses.begin(); it != camera_poses.end(); ++it)
	{
		cv::Mat camera_pose;
		(*it) >> camera_pose;

		Eigen::Matrix4d* transform = new Eigen::Matrix4d;

		for (int y = 0; y < 4; y++)
			for (int x = 0; x < 4; x++)
				(*transform)(y, x) = camera_pose.at<double>(y, x);

		camera_pose_vec.push_back(transform);
	}

	for (cv::FileNodeIterator it = tcp_poses.begin(); it != tcp_poses.end(); ++it)
	{
		cv::Mat tcp_pose;
		(*it) >> tcp_pose;

		Eigen::Matrix4d* transform = new Eigen::Matrix4d;

		for (int y = 0; y < 4; y++)
			for (int x = 0; x < 4; x++)
				(*transform)(y, x) = tcp_pose.at<double>(y, x);

		tcp_pose_vec.push_back(transform);
	}

	fs.release();

	// hand eye calibration
	Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
	Eigen::MatrixXd C(3 * nframes *(nframes-1), 3);
	Eigen::VectorXd d(3 * nframes * (nframes-1));
	Eigen::VectorXd bA(3 * nframes * (nframes-1));
	Eigen::VectorXd bB(3 * nframes * (nframes-1));

	int count = 0;

	for (int i = 0; i < nframes; i++)
	{
		for (int j = 0; j < nframes; j++)
		{
			if (i == j) continue;

			// TCP pose motion
			Eigen::Matrix4d A;
			A = (*tcp_pose_vec[i]).inverse() * (*tcp_pose_vec[j]);	//base to robot hand

			// camera pose motion
			Eigen::Matrix4d B;
			B = (*camera_pose_vec[i])*(*camera_pose_vec[j]).inverse();	//camera to calibration board

			//log Rotation
			Eigen::Matrix3d alpha, beta;

			double theta = acos(0.5*(A.block<3, 3>(0, 0).trace() - 1.));

			alpha = theta*0.5 / sin(theta)*(A.block<3, 3>(0, 0) - A.block<3, 3>(0, 0).transpose());

			theta = acos(0.5*(B.block<3, 3>(0, 0).trace() - 1.));

			beta = theta*0.5 / sin(theta)*(B.block<3, 3>(0, 0) - B.block<3, 3>(0, 0).transpose());

			M = M + beta*alpha.transpose();

			C.block<3, 3>(count * 3, 0) = Eigen::Matrix3d::Identity() - A.block<3, 3>(0, 0);
			bA.block<3, 1>(count * 3, 0) = A.block<3, 1>(0, 3);
			bB.block<3, 1>(count * 3, 0) = B.block<3, 1>(0, 3);
			count++;
		}
	}

	Eigen::EigenSolver<Eigen::Matrix3d> es(M.transpose()*M);

	Eigen::Matrix3d lambda;

	lambda = es.eigenvalues().real().cwiseSqrt().cwiseInverse().asDiagonal();
	
	Eigen::Matrix3d hand_to_eye_rotation = es.eigenvectors().real()*lambda*es.eigenvectors().real().inverse()*M.transpose();

	for (int i = 0; i < nframes*(nframes - 1); i++)
		bB.block<3, 1>(i * 3, 0) = hand_to_eye_rotation*bB.block<3, 1>(i * 3, 0);

	d = bA - bB;

	Eigen::Vector3d hand_to_eye_translation = (C.transpose()*C).inverse()*C.transpose()*d;

	cv::Mat hand_to_eye = cv::Mat::eye(4, 4, CV_64F);

	for (int y = 0; y < 3; y++)
		for (int x = 0; x < 3; x++)
			hand_to_eye.at<double>(y, x) = hand_to_eye_rotation(y, x);

	for (int i = 0; i < 3; i++)
		hand_to_eye.at<double>(i, 3) = hand_to_eye_translation(i);

	std::cout << "hand to eye\n" << hand_to_eye << "\n";

	cv::FileStorage fs1("kinectRGBHandEyeCalibration.yml", cv::FileStorage::WRITE);

	fs1 << "hand to eye" << hand_to_eye;

	fs1.release();
}

void VisionArmCombo::KinectIRHandEyeCalibration()
{
	cv::FileStorage fs("kinectIRCalibration.yml", cv::FileStorage::READ);
	cv::FileNode camera_poses = fs["camera poses"];
	cv::FileNode tcp_poses = fs["TCP poses"];

	int nframes;

	fs["nframes"] >> nframes;

	std::vector<Eigen::Matrix4d*> camera_pose_vec;
	std::vector<Eigen::Matrix4d*> tcp_pose_vec;

	// iterate through a sequence using FileNodeIterator
	for (cv::FileNodeIterator it = camera_poses.begin(); it != camera_poses.end(); ++it)
	{
		cv::Mat camera_pose;
		(*it) >> camera_pose;

		Eigen::Matrix4d* transform = new Eigen::Matrix4d;

		for (int y = 0; y < 4; y++)
			for (int x = 0; x < 4; x++)
				(*transform)(y, x) = camera_pose.at<double>(y, x);

		camera_pose_vec.push_back(transform);
	}

	for (cv::FileNodeIterator it = tcp_poses.begin(); it != tcp_poses.end(); ++it)
	{
		cv::Mat tcp_pose;
		(*it) >> tcp_pose;

		Eigen::Matrix4d* transform = new Eigen::Matrix4d;

		for (int y = 0; y < 4; y++)
			for (int x = 0; x < 4; x++)
				(*transform)(y, x) = tcp_pose.at<double>(y, x);

		tcp_pose_vec.push_back(transform);
	}

	fs.release();

	// hand eye calibration
	Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
	Eigen::MatrixXd C(3 * nframes *(nframes - 1), 3);
	Eigen::VectorXd d(3 * nframes * (nframes - 1));
	Eigen::VectorXd bA(3 * nframes * (nframes - 1));
	Eigen::VectorXd bB(3 * nframes * (nframes - 1));

	int count = 0;

	for (int i = 0; i < nframes; i++)
	{
		for (int j = 0; j < nframes; j++)
		{
			if (i == j) continue;

			// TCP pose motion
			Eigen::Matrix4d A;
			A = (*tcp_pose_vec[i]).inverse() * (*tcp_pose_vec[j]);	//base to robot hand

																	// camera pose motion
			Eigen::Matrix4d B;
			B = (*camera_pose_vec[i])*(*camera_pose_vec[j]).inverse();	//camera to calibration board

																		//log Rotation
			Eigen::Matrix3d alpha, beta;

			double theta = acos(0.5*(A.block<3, 3>(0, 0).trace() - 1.));

			alpha = theta*0.5 / sin(theta)*(A.block<3, 3>(0, 0) - A.block<3, 3>(0, 0).transpose());

			theta = acos(0.5*(B.block<3, 3>(0, 0).trace() - 1.));

			beta = theta*0.5 / sin(theta)*(B.block<3, 3>(0, 0) - B.block<3, 3>(0, 0).transpose());

			M = M + beta*alpha.transpose();

			C.block<3, 3>(count * 3, 0) = Eigen::Matrix3d::Identity() - A.block<3, 3>(0, 0);
			bA.block<3, 1>(count * 3, 0) = A.block<3, 1>(0, 3);
			bB.block<3, 1>(count * 3, 0) = B.block<3, 1>(0, 3);
			count++;
		}
	}

	Eigen::EigenSolver<Eigen::Matrix3d> es(M.transpose()*M);

	Eigen::Matrix3d lambda;

	lambda = es.eigenvalues().real().cwiseSqrt().cwiseInverse().asDiagonal();

	Eigen::Matrix3d hand_to_eye_rotation = es.eigenvectors().real()*lambda*es.eigenvectors().real().inverse()*M.transpose();

	for (int i = 0; i < nframes*(nframes - 1); i++)
		bB.block<3, 1>(i * 3, 0) = hand_to_eye_rotation*bB.block<3, 1>(i * 3, 0);

	d = bA - bB;

	Eigen::Vector3d hand_to_eye_translation = (C.transpose()*C).inverse()*C.transpose()*d;

	cv::Mat hand_to_eye = cv::Mat::eye(4, 4, CV_64F);

	for (int y = 0; y < 3; y++)
		for (int x = 0; x < 3; x++)
			hand_to_eye.at<double>(y, x) = hand_to_eye_rotation(y, x);

	for (int i = 0; i < 3; i++)
		hand_to_eye.at<double>(i, 3) = hand_to_eye_translation(i);

	std::cout << "hand to eye\n" << hand_to_eye << "\n";

	cv::FileStorage fs1("kinectIRHandEyeCalibration.yml", cv::FileStorage::WRITE);

	fs1 << "hand to eye" << hand_to_eye;

	fs1.release();
}

void VisionArmCombo::ThermalHandEyeCalibration() {

	cv::FileStorage fs("FlirThermalCalibration.yml", cv::FileStorage::READ);
	cv::FileNode camera_poses = fs["camera poses"];
	cv::FileNode tcp_poses = fs["TCP poses"];

	int nframes;

	fs["nframes"] >> nframes;

	std::vector<Eigen::Matrix4d*> camera_pose_vec;
	std::vector<Eigen::Matrix4d*> tcp_pose_vec;

	// iterate through a sequence using FileNodeIterator
	for (cv::FileNodeIterator it = camera_poses.begin(); it != camera_poses.end(); ++it)
	{
		cv::Mat camera_pose;
		(*it) >> camera_pose;

		Eigen::Matrix4d* transform = new Eigen::Matrix4d;

		for (int y = 0; y < 4; y++)
			for (int x = 0; x < 4; x++)
				(*transform)(y, x) = camera_pose.at<double>(y, x);

		camera_pose_vec.push_back(transform);
	}

	for (cv::FileNodeIterator it = tcp_poses.begin(); it != tcp_poses.end(); ++it)
	{
		cv::Mat tcp_pose;
		(*it) >> tcp_pose;

		Eigen::Matrix4d* transform = new Eigen::Matrix4d;

		for (int y = 0; y < 4; y++)
			for (int x = 0; x < 4; x++)
				(*transform)(y, x) = tcp_pose.at<double>(y, x);

		tcp_pose_vec.push_back(transform);
	}

	fs.release();

	// hand eye calibration
	Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
	Eigen::MatrixXd C(3 * nframes *(nframes - 1), 3);
	Eigen::VectorXd d(3 * nframes * (nframes - 1));
	Eigen::VectorXd bA(3 * nframes * (nframes - 1));
	Eigen::VectorXd bB(3 * nframes * (nframes - 1));

	int count = 0;

	for (int i = 0; i < nframes; i++)
	{
		for (int j = 0; j < nframes; j++)
		{
			if (i == j) continue;

			// TCP pose motion
			Eigen::Matrix4d A;
			A = (*tcp_pose_vec[i]).inverse() * (*tcp_pose_vec[j]);	//base to robot hand

																	// camera pose motion
			Eigen::Matrix4d B;
			B = (*camera_pose_vec[i])*(*camera_pose_vec[j]).inverse();	//camera to calibration board

																		//log Rotation
			Eigen::Matrix3d alpha, beta;

			double theta = acos(0.5*(A.block<3, 3>(0, 0).trace() - 1.));

			alpha = theta*0.5 / sin(theta)*(A.block<3, 3>(0, 0) - A.block<3, 3>(0, 0).transpose());

			theta = acos(0.5*(B.block<3, 3>(0, 0).trace() - 1.));

			beta = theta*0.5 / sin(theta)*(B.block<3, 3>(0, 0) - B.block<3, 3>(0, 0).transpose());

			M = M + beta*alpha.transpose();

			C.block<3, 3>(count * 3, 0) = Eigen::Matrix3d::Identity() - A.block<3, 3>(0, 0);
			bA.block<3, 1>(count * 3, 0) = A.block<3, 1>(0, 3);
			bB.block<3, 1>(count * 3, 0) = B.block<3, 1>(0, 3);
			count++;
		}
	}

	Eigen::EigenSolver<Eigen::Matrix3d> es(M.transpose()*M);

	Eigen::Matrix3d lambda;

	lambda = es.eigenvalues().real().cwiseSqrt().cwiseInverse().asDiagonal();

	Eigen::Matrix3d hand_to_eye_rotation = es.eigenvectors().real()*lambda*es.eigenvectors().real().inverse()*M.transpose();

	for (int i = 0; i < nframes*(nframes - 1); i++)
		bB.block<3, 1>(i * 3, 0) = hand_to_eye_rotation*bB.block<3, 1>(i * 3, 0);

	d = bA - bB;

	Eigen::Vector3d hand_to_eye_translation = (C.transpose()*C).inverse()*C.transpose()*d;

	cv::Mat hand_to_eye = cv::Mat::eye(4, 4, CV_64F);

	for (int y = 0; y < 3; y++)
		for (int x = 0; x < 3; x++)
			hand_to_eye.at<double>(y, x) = hand_to_eye_rotation(y, x);

	for (int i = 0; i < 3; i++)
		hand_to_eye.at<double>(i, 3) = hand_to_eye_translation(i);

	std::cout << "hand to eye\n" << hand_to_eye << "\n";

	cv::FileStorage fs1("FlirThermalHandEyeCalibration.yml", cv::FileStorage::WRITE);

	fs1 << "hand to eye" << hand_to_eye;

	fs1.release();

}

void VisionArmCombo::markerDetection()
{
	cv::Mat markerImage;
	//make pot grid with marker 
	int marker_img_size = 1200;	//multiple of 5+2
	int grid_width = 7;
	int grid_height = 5;

	//cv::aruco::drawMarker(marker_dictionary_, 0, marker_img_size, markerImage, 1);

	cv::Mat center_square;
	center_square.create(marker_img_size, marker_img_size, CV_8UC3);

	std::memset(center_square.ptr(), 255, marker_img_size * marker_img_size * 3);

	float square_size = marker_img_size/4.f*3.5f;
	cv::rectangle(center_square, cv::Point2f((marker_img_size - square_size)*0.5f, (marker_img_size - square_size)*0.5f),
					cv::Point2f(marker_img_size-(marker_img_size - square_size)*0.5f, marker_img_size - (marker_img_size - square_size)*0.5f), cv::Scalar(0,0,0),4);

	cv::Mat block;
	block.create(marker_img_size, marker_img_size, CV_8UC3);
	std::memset(block.ptr(), 255, marker_img_size * marker_img_size * 3);
	
	cv::circle(block, cv::Point2f(marker_img_size*0.5f, marker_img_size*0.5f), 780/2/*marker_img_size / 3*/, cv::Scalar(0, 0, 0), 4);

	cv::circle(block, cv::Point2f(marker_img_size*0.5f, marker_img_size*0.5f), 40, cv::Scalar(0, 0, 0), 40);

	//cv::imshow("block", block);

	//cv::waitKey(0);

	cv::Mat block_grid;

	block_grid.create(marker_img_size * grid_height, marker_img_size * grid_width, CV_8UC3);

	for (int y = 0; y < grid_height; y++)
	{
		for (int x = 0; x < grid_width; x++)
		{
			block.copyTo(block_grid(cv::Rect(x*marker_img_size, y*marker_img_size, marker_img_size, marker_img_size)));
		}
	}

	//cv::Mat marker_img;
	//cv::cvtColor(markerImage, marker_img, CV_GRAY2BGR);
	//marker_img.copyTo(block_grid(cv::Rect((grid_width/2)*marker_img_size, (grid_height/2)*marker_img_size, marker_img_size, marker_img_size)));
	center_square.copyTo(block_grid(cv::Rect((grid_width / 2)*marker_img_size, (grid_height / 2)*marker_img_size, marker_img_size, marker_img_size)));

	cv::imwrite("block_grid.png", block_grid);

	cv::Mat shrink;
	cv::resize(block_grid, shrink, cv::Size(), 0.1, 0.1);

	cv::imshow("block gird", shrink);
	cv::waitKey(0);

	return;
	

	/*	//generate marker images and save
	for (int i = 0; i < 50; i++)
	{
		cv::aruco::drawMarker(marker_dictionary_, i, 700, markerImage, 1);

		cv::imshow("marker", markerImage);

		cv::imwrite("Markers\\marker_" + std::to_string(i) + ".png", markerImage);

		cv::waitKey(100);
	}*/

	if (robot_arm_client_ == NULL) initRobotArmClient();

	if (kinect_thread_ == NULL)	initKinectThread();

	cv::Mat rgb;
	cv::Vec3d rot;
	cv::Vec3d tran;

	ArmConfig config;
	config.setJointPos(-86.5, -99.37, -155.46, -12.91, 90., -178.94);
	config.toRad();

	robot_arm_client_->moveHandJ(config.joint_pos_d, 0.1, 0.1, true);


	while (true)
	{
		rgb = kinect_thread_->getCurRGB();

		std::vector<int> markerIds; 
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
		cv::aruco::detectMarkers(rgb, marker_dictionary_, markerCorners, markerIds, detector_params_, rejectedCandidates);
		
		std::vector<cv::Vec3d> rvecs, tvecs;

		cv::aruco::estimatePoseSingleMarkers(markerCorners, marker_length_, kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_, rvecs, tvecs);

		cv::aruco::drawDetectedMarkers(rgb, markerCorners, markerIds);

		for (unsigned int i = 0; i < markerIds.size(); i++)
		{
			cv::aruco::drawAxis(rgb, kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_, rvecs[i], tvecs[i], marker_length_*0.5f);
			std::cout <<"id "<< i<<" rot " << rvecs[i] << " tran " << tvecs[i]<<"\n";
			rot = rvecs[i];
			tran = tvecs[i];
		}

		cv::imshow("marker", rgb);

		int key = cv::waitKey(10);

		if (key == 113)	//q
		{
			break;
		}
	}

	cv::Mat rgb_to_marker_rot_cv;

	cv::Rodrigues(rot, rgb_to_marker_rot_cv);

	for (int y = 0; y < 3; y++)
		for (int x = 0; x < 3; x++)
			cur_rgb_to_marker_(y, x) = rgb_to_marker_rot_cv.at<double>(y, x);

	for (int y = 0; y < 3; y++) cur_rgb_to_marker_(y, 3) = tran[y];

	std::cout << "rgb to marker:\n" << cur_rgb_to_marker_ << "\n";

	Eigen::Matrix4d base_to_hand;

	getCurHandPoseD(base_to_hand);

	Eigen::Matrix3d  rotate_x_pi;

	rotate_x_pi = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

	Eigen::Matrix4d marker_to_gripper = Eigen::Matrix4d::Identity();

	marker_to_gripper.topLeftCorner<3, 3>() = rotate_x_pi.matrix();

	

	Eigen::Matrix4d base_to_marker = base_to_hand*hand_to_rgb_*cur_rgb_to_marker_;

	std::cout << "base to marker:\n" << base_to_marker << "\n";

	for (int y = 2; y >= -2; y--)
	{
		for (int x = -3; x <= 3; x++)
		{
			std::getchar();

			Eigen::Matrix4d marker_to_gripper_translate = Eigen::Matrix4d::Identity();
			marker_to_gripper_translate.col(3).head(3) << x* marker_length_, y* marker_length_, -0.03;

			Eigen::Matrix4d probe_pose_eigen = base_to_marker*marker_to_gripper*marker_to_gripper_translate*probe_to_hand_;

			std::cout << "probe pose:\n" << probe_pose_eigen << "\n";

			// move arm
			double pose[6];

			eigenMat4dToArray6(probe_pose_eigen, pose);

			robot_arm_client_->moveHandL(pose, 0.1, 0.1);
		}
	}

}

void VisionArmCombo::cvTransformToEigenTransform(cv::Mat & cv_transform, Eigen::Matrix4d & eigen_transform)
{
	for (int y = 0; y < 4; y++)
		for (int x = 0; x < 4; x++)
			eigen_transform(y, x) = cv_transform.at<double>(y, x);
}

void VisionArmCombo::EigenTransformToCVTransform( Eigen::Matrix4d & eigen_transform, cv::Mat & cv_transform)
{
	for (int y = 0; y < 4; y++)
		for (int x = 0; x < 4; x++)
			cv_transform.at<double>(y, x) = eigen_transform(y, x);
}

void VisionArmCombo::scanAndProbeTest()
{
	if (robot_arm_client_ == NULL) initRobotArmClient();
	if (line_profiler_ == NULL) initLineProfiler();

	double pose[6];
	double tran[3];
	
	Eigen::Matrix4f cur_pose;
	getCurHandPose(cur_pose);
	if (cur_pose(2, 2) >= 0.)	//check z direction
	{
		std::cout << "gripper pointing down!\n";
		return;
	}

	Eigen::AngleAxisd scan_rot(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(-M_PI*0.5, Eigen::Vector3d::UnitZ()));
	Eigen::Vector3d rot_vec(scan_rot.angle()*scan_rot.axis());

	float scan_vec = 0.3;
	pose[0] = 0.62; pose[1] = 0.255-0.3; pose[2] = 0.272;
	pose[3] = rot_vec[0], pose[4] = rot_vec[1], pose[5] = rot_vec[2];
	
	tran[0] = 0.;
	tran[1] = scan_vec;
	tran[2] = 0.;

	robot_arm_client_->moveHandL(pose, 0.2, 0.2);
	//robot_arm_client_->waitTillHandReachDstPose(pose);

	Sleep(1000);
	/*std::cout << "Ready\n";
	std::getchar();*/

	PointCloudT::Ptr scan_cloud(new PointCloudT);
	PointCloudT::Ptr scan_cloud_reverse(new PointCloudT);
	
	getCurHandPose(cur_pose);

	line_profiler_->m_vecProfileData.clear();
	scanTranslateOnly(tran, scan_cloud, scan_acceleration_, scan_speed_);

	vox_.setInputCloud(scan_cloud);
	vox_.setLeafSize(0.0002, 0.0002, 0.0002);

	PointCloudT::Ptr scan_cloud_down(new PointCloudT);

	vox_.filter(*scan_cloud_down);

	PointCloudT::Ptr scan_cloud_down_base(new PointCloudT);

	pcl::transformPointCloud(*scan_cloud_down, *scan_cloud_down_base, cur_pose*handToScanner_);

	//std::cout << "Ready\n";
	//std::getchar();

	/*robot_arm_client_->getCartesianInfo(pose);
	pose[3] = 3.034, pose[4] = 0.0267, pose[5] = 0.1917;
	robot_arm_client_->moveHandL(pose, 0.5, 0.5);
	//robot_arm_client_->waitTillHandReachDstPose(pose);*/

	/*Sleep(1000);

	std::cout << "\n";

	tran[0] = -scan_vec;
	PointCloudT::Ptr scan_cloud1(new PointCloudT);

	Eigen::Matrix4f cur_pose1;
	getCurHandPose(cur_pose1);

	line_profiler_->m_vecProfileData.clear();
	scanTranslateOnly(tran, scan_cloud1, scan_acceleration_, scan_speed_);

	vox_.setInputCloud(scan_cloud1);
	vox_.setLeafSize(0.0002, 0.0002, 0.0002);

	PointCloudT::Ptr scan_cloud_down1(new PointCloudT);

	vox_.filter(*scan_cloud_down1);

	PointCloudT::Ptr scan_cloud_down_base1(new PointCloudT);

	pcl::transformPointCloud(*scan_cloud_down1, *scan_cloud_down_base1, cur_pose1*handToScanner_);

	for (int i = 0; i < scan_cloud_down_base1->points.size(); i++)
	{
		scan_cloud_down_base1->points[i].b = 0;
	}

	*scan_cloud_down_base1 += *scan_cloud_down_base;

	vox_.setInputCloud(scan_cloud_down_base1);

	vox_.filter(*scan_cloud_down_base);*/


	//viewer_->addPointCloud(scan_cloud_down_base, "0");
	//viewer_->addPointCloud(scan_cloud_down_base1, "1");
	//viewer_->spin();

	//test planes
	/*PointCloudT::Ptr metal(new PointCloudT);
	PointCloudT::Ptr table (new PointCloudT);
	pass_.setInputCloud(scan_cloud_down_base);
	pass_.setFilterFieldName("z");
	pass_.setFilterLimits(-0.1598f, -0.1582f);
	pass_.setFilterLimitsNegative(false);
	pass_.filter(*metal);

	pass_.setFilterLimits(-0.25f, -0.194f);
	pass_.filter(*table);

	for (auto &p : metal->points)
	{
		p.b = 0;
	}

	pcl::PCA<PointT> pca(*metal);

	Eigen::Matrix3d eigen_vectors = pca.getEigenVectors().cast<double>();

	std::cout << "metal:\n" << eigen_vectors<<"\n";

	pca.setInputCloud(table);

	Eigen::Matrix3d table_eigen_vectors = pca.getEigenVectors().cast<double>();

	std::cout << "table:\n" << table_eigen_vectors << "\n";

	if(eigen_vectors.col(2)(2) *table_eigen_vectors.col(2)(2) < 0.)
		std::cout<<"angle: "<<acos(eigen_vectors.col(2).dot(-table_eigen_vectors.col(2)))/M_PI*180.<<"\n";
	else
		std::cout << "angle: " << acos(eigen_vectors.col(2).dot(table_eigen_vectors.col(2))) / M_PI*180. << "\n";

	viewer_->addPointCloud(table, "0");
	viewer_->addPointCloud(metal, "1");
	viewer_->spin(); */


	//fitPotRing(scan_cloud_down_base);
#ifdef ROAD
	find3DMarker(scan_cloud_down_base);
#endif

	//robot_arm_client_->rotateJointRelative(4, -180., 0.7, 0.7);

#if 0
	pose[0] = 0.862;  pose[1] = 0.049; pose[2] = 0.255;
	robot_arm_client_->moveHandL(pose, 0.3, 0.3);
	//robot_arm_client_->waitTillHandReachDstPose(pose);

	Sleep(1000);
	
	float leaf_size = 0.001;
	PointCloudT::Ptr scan_cloud2(new PointCloudT);

	getCurHandPose(cur_pose);

	tran[1] = -0.2;

	line_profiler_->m_vecProfileData.clear();
	scanTranslateOnly(tran, scan_cloud2, scan_acceleration_, scan_speed_);

	vox_.setInputCloud(scan_cloud2);
	vox_.setLeafSize(leaf_size, leaf_size, leaf_size);

	PointCloudT::Ptr scan_cloud_down2(new PointCloudT);

	vox_.filter(*scan_cloud_down2);

	PointCloudT::Ptr scan_cloud_down_base2(new PointCloudT);

	pcl::transformPointCloud(*scan_cloud_down2, *scan_cloud_down_base2, cur_pose*handToScanner_);

	*scan_cloud_down_base += *scan_cloud_down_base2;


	pose[0] = 0.862;  pose[1] = 0.373; pose[2] = 0.255;
	robot_arm_client_->moveHandL(pose, 0.3, 0.3);
	//robot_arm_client_->waitTillHandReachDstPose(pose);

	Sleep(1000);

	PointCloudT::Ptr scan_cloud3(new PointCloudT);

	getCurHandPose(cur_pose);

	line_profiler_->m_vecProfileData.clear();
	scanTranslateOnly(tran, scan_cloud3, scan_acceleration_, scan_speed_);

	vox_.setInputCloud(scan_cloud3);
	vox_.setLeafSize(leaf_size, leaf_size, leaf_size);

	PointCloudT::Ptr scan_cloud_down3(new PointCloudT);

	vox_.filter(*scan_cloud_down3);

	PointCloudT::Ptr scan_cloud_down_base3(new PointCloudT);

	pcl::transformPointCloud(*scan_cloud_down3, *scan_cloud_down_base3, cur_pose*handToScanner_);

	*scan_cloud_down_base += *scan_cloud_down_base3;

	pose[0] = 0.373;  pose[1] = 0.373; pose[2] = 0.255;
	robot_arm_client_->moveHandL(pose, 0.3, 0.3);
	//robot_arm_client_->waitTillHandReachDstPose(pose);

	Sleep(1000);

	PointCloudT::Ptr scan_cloud4(new PointCloudT);

	getCurHandPose(cur_pose);


	line_profiler_->m_vecProfileData.clear();
	scanTranslateOnly(tran, scan_cloud4, scan_acceleration_, scan_speed_);

	vox_.setInputCloud(scan_cloud4);
	vox_.setLeafSize(leaf_size, leaf_size, leaf_size);

	PointCloudT::Ptr scan_cloud_down4(new PointCloudT);

	vox_.filter(*scan_cloud_down4);

	PointCloudT::Ptr scan_cloud_down_base4(new PointCloudT);

	pcl::transformPointCloud(*scan_cloud_down4, *scan_cloud_down_base4, cur_pose*handToScanner_);

	*scan_cloud_down_base += *scan_cloud_down_base4;
#endif

	probeScannedSceneTest(scan_cloud_down_base);
}

int VisionArmCombo::sendRoboteqVar(int id, int value)
{
	int status = motor_controller_.SetCommand(_VAR, id, value);

	// send success, but status != RQ_SUCCESS, roboteq bug
	//if ( status != RQ_SUCCESS)
	//{
		//cout << "set Roboteq VAR failed --> " << status << endl;
		//return -1;
	//}

	sleepms(100);

	//int result = -1;
	//status = motor_controller_.GetValue(_VAR, id, result);

	//if (status != RQ_SUCCESS)
//	{
	//	cout << "get var failed --> " << status << endl;
	//}

//	std::cout << "result: " << result << "\n";

	return 0;
}

void VisionArmCombo::setAndSaveParameters()
{
	cv::FileStorage fs("parameters.yml", cv::FileStorage::WRITE);

	region_grow_residual_threshold_ = 0.005f;
	region_grow_smoothness_threshold_ = 3.0 / 180.*M_PI;
	region_grow_curvature_threshold_ = 1.0;

	fs << "region_grow_residual_threshold_" << region_grow_residual_threshold_;
	fs << "region_grow_smoothness_threshold_" << region_grow_smoothness_threshold_;
	fs << "region_grow_curvature_threshold_" << region_grow_curvature_threshold_;

	fs.release();
}

bool VisionArmCombo::scanGrowthChamberWithKinect(int location_id, ArmConfig & imaging_config, bool add_to_occupancy_grid=false)
{
	if (kinect_thread_ == NULL)
	{
		std::cout << "Kinect not running\n";
		return false;
	}

	float step = 45.f;

	ArmConfig config(imaging_config);

	config.joint_pos_d[4] += step / 180.*M_PI;
	config.joint_pos_f[4] += step / 180.*M_PI;

	for (int i = 0; i <3; i++)
	{
		if (i==1 && moveToConfigGetKinectPointCloud(config, true, true, add_to_occupancy_grid))
		{
			std::cout << "Scan idx " << i + 1<<"\n";
		}

		config.joint_pos_d[4] -= step / 180.*M_PI;
		config.joint_pos_f[4] -= step / 180.*M_PI;
	}

	return true;
}

// do topview thermal imaging after laser scan
int VisionArmCombo::scanPlantCluster(cv::Vec3f &object_center, float max_z, float radius, int plant_id)
{
	PointT point;
	point.x = object_center[0] - handToScanner_(0, 3);
	point.y = object_center[1];
	point.z = max_z;

	pcl::Normal normal(0.f, 0.f, -1.f);

	std::vector<ArmConfig> solution_config_vec;
	Eigen::Matrix4d scan_start_hand_pose;
	Eigen::Vector3d hand_translation;

	//float radius = 0.5f*( (*plant_cluster_max_vec_[cluster_idx])(0) - (*plant_cluster_min_vec_[cluster_idx])(0) );

	setScanRadius(-radius);

	std::string laser_cloud_name;

	// compute laser scan pose
	if (computeCollisionFreeProbeOrScanPose(point, normal, SCANNER, solution_config_vec, scan_start_hand_pose, hand_translation))
	{
		//std::cout << "hand translation: " << hand_translation.transpose() << "\n";

		// move to the start scan pose
		if (moveToConfigGetKinectPointCloud(solution_config_vec[0], false, true))
		{
			double vec[3];
			vec[0] = hand_translation(0);
			vec[1] = hand_translation(1);
			vec[2] = hand_translation(2);

			PointCloudT::Ptr scan_cloud(new PointCloudT);

			//wait 1 sec
			//std::cout << "scan plant " << object_idx << "\n";
			Sleep(1000);

			// moveToConfig moves the arm to a pose slightly different from scan_start_hand_pose
			Eigen::Matrix4f curPose; getCurHandPose(curPose);

			//std::cout << curPose << "\n" << scan_start_hand_pose << "\n";

			scan_start_hand_pose = curPose.cast<double>();

			// Use laser scanner
			line_profiler_->m_vecProfileData.clear();
			scanTranslateOnly(vec, scan_cloud, scan_acceleration_, scan_speed_);

	/*		viewer_->addPointCloud(scan_cloud, "scan_cloud", 0);
			viewer_->spin();
			viewer_->removePointCloud("scan_cloud", 0);*/
			
			PointCloudT::Ptr scan_cloud_down(new PointCloudT);

			vox_.setLeafSize(voxel_grid_size_laser_, voxel_grid_size_laser_, voxel_grid_size_laser_);
	
			vox_.setInputCloud(scan_cloud);

			vox_.filter(*scan_cloud_down);

			sor_.setInputCloud(scan_cloud_down);

			PointCloudT::Ptr scan_cloud_down_filter(new PointCloudT);

			sor_.filter(*scan_cloud_down_filter);

			PointCloudT::Ptr scan_cloud_in_base(new PointCloudT);

			pcl::transformPointCloud(*scan_cloud_down_filter, *scan_cloud_in_base, scan_start_hand_pose.cast<float>()*handToScanner_);

			*laser_cloud_ += *scan_cloud_in_base;

			vox_.setInputCloud(laser_cloud_);

			scan_cloud_down->clear();

			vox_.filter(*scan_cloud_down);

			*laser_cloud_ = *scan_cloud_down;

		//	if(scan_cloud_down->points.size() != 0)
			//	pcl::io::savePCDFileBinary("laser_scan.pcd", *scan_cloud_in_base);

			//pp_.resetOccupancyGrid();

			pp_.addPointCloudToOccupancyGrid(laser_cloud_);

			plant_laser_pc_vec_.push_back(scan_cloud_in_base);

		//	return true;

			laser_cloud_name = "laser_cloud" + std::to_string(cv::getTickCount());

			viewer_->addPointCloud(laser_cloud_, laser_cloud_name);
		//	viewer_->spin();
		//	viewer_->removePointCloud("laser_cloud", 0);

			//showOccupancyGrid();

			// need to remove the pot
			/*	pass_.setInputCloud(scan_cloud_in_base);
			pass_.setFilterFieldName("z");
			pass_.setFilterLimits(0.08f, 1.28f);		//in lab on bench
			pass_.setFilterLimitsNegative(false);
			pass_.filter(*scan_cloud_down);
			*scan_cloud_in_base = *scan_cloud_down;
			*/

			//probeScannedSceneTest(scan_cloud_in_base);

		}
		else
			std::cout << "can not move to config for scanning plant cluster\n";
	}
	else
		std::cout << "laser scan plant cluster pose not found\n";

	if (thermocam_ == NULL) {
		std::cout << "thermal camera not initialized\n";
		return -2;
	}

	// top-view thermal imaging
	point.x = object_center[0];
	if (computeCollisionFreeProbeOrScanPose(point, normal, THERMAL, solution_config_vec, scan_start_hand_pose, hand_translation))
	{
		// move to the start scan pose
		if (moveToConfigGetKinectPointCloud(solution_config_vec[0], false, true))
		{
			cv::Mat color_map, temperature_map;

			if (thermocam_->isConnected) {

				thermocam_->snapShot(color_map, temperature_map);
				
				if (color_map.data != NULL) {

					std::string save_path(data_saving_folder_.begin(), data_saving_folder_.end());

					std::string false_color = save_path + "thermal_topview_plant_" + std::to_string(plant_id) + ".jpg";

					std::string temperature = save_path + "temperature_64F_plant_" + std::to_string(plant_id) + ".bin";
					
					cv::imwrite(false_color, color_map);

					std::ofstream out(temperature, std::ios::out | std::ios::binary);

					out.write((char*)temperature_map.data, 640*480*sizeof(double));

					out.close();

					std::string pose_file_name = save_path + "thermal_camera_pose_plant_" + std::to_string(plant_id) + ".yml";
					cv::FileStorage fs(pose_file_name, cv::FileStorage::WRITE);
					
					Eigen::Matrix4d thermal_camera_pose = scan_start_hand_pose*hand_to_thermal_d_;
					cv::Mat thermal_camera_pose_cv;
					thermal_camera_pose_cv.create(4, 4, CV_64F);

					EigenTransformToCVTransform(thermal_camera_pose, thermal_camera_pose_cv);

					fs << "thermal_camera_pose" << thermal_camera_pose_cv;

					fs.release();

					cv::Mat undistort, zero_distort;
					cv::undistort(color_map, undistort, flir_thermal_camera_matrix_cv_, flir_thermal_dist_coeffs_cv_);

					cv::imshow("thermal", undistort);
					cv::waitKey(view_time_);

					std::vector<cv::Point3f> object_points(laser_cloud_->points.size());
					for (int i = 0; i<laser_cloud_->points.size(); i++)
					{
						object_points[i].x = laser_cloud_->points[i].x;
						object_points[i].y = laser_cloud_->points[i].y;
						object_points[i].z = laser_cloud_->points[i].z;
					}

					Eigen::Matrix4d tmp_inverse = thermal_camera_pose.inverse();

					cv::Mat rot, tvec, rvec;
					rot.create(3, 3, CV_64F);
					tvec.create(3, 1, CV_64F);
					for (int y = 0; y < 3; y++)
						for (int x = 0; x < 3; x++)
							rot.at<double>(y, x) = tmp_inverse(y, x);

					tvec.at<double>(0, 0) = tmp_inverse(0, 3);
					tvec.at<double>(1, 0) = tmp_inverse(1, 3);
					tvec.at<double>(2, 0) = tmp_inverse(2, 3);

					cv::Rodrigues(rot, rvec);
					std::vector<cv::Point2f> img_points;
					cv::projectPoints(object_points, rvec, tvec, flir_thermal_camera_matrix_cv_, flir_thermal_dist_coeffs_cv_, img_points);

					for (int i = 0; i < laser_cloud_->points.size(); i++)
					{
						int x = std::round(img_points[i].x);
						int y = std::round(img_points[i].y);
						
						if (x >= 0 && x < 640 && y >= 0 && y < 480)
						{
							laser_cloud_->points[i].b = undistort.at<cv::Vec3b>(y, x).val[0];
							laser_cloud_->points[i].g = undistort.at<cv::Vec3b>(y, x).val[1];
							laser_cloud_->points[i].r = undistort.at<cv::Vec3b>(y, x).val[2];
						}
					}

					std::cout << "showing heat map registered point cloud...\n";

					if(laser_cloud_->points.size() != 0)
						pcl::io::savePCDFileBinary("laser_scan.pcd", *laser_cloud_);

					viewer_->removeAllPointClouds();
					viewer_->addPointCloud(laser_cloud_, laser_cloud_name);

					display();
				}
			}
		}
		else
			std::cout << "can not move to config for top-view thermal imaging\n";
	}
	else
		std::cout << "scan pose not found for top-view thermal imaging\n";

	return 0;
}

void VisionArmCombo::testRun()
{
#if 0
	if (!moveToConfigGetKinectPointCloud(home_config_, false, true, false))
	{
		std::cout << "can not go to home config\n";
		return;
	}
#endif

	int cur_rover_status = -1;
	motor_controller_.GetValue(_VAR, 4, cur_rover_status);

	if (only_do_probing_ != 1)
	{
		std::cout << "current rover status: " << cur_rover_status << "\n";

		std::getchar();
		sendRoboteqVar(1, 1);//go to door 1
		while (true)
		{
			//read current stop
			motor_controller_.GetValue(_VAR, 4, cur_rover_status);

			//std::cout << "cur_location_in_chamber " << cur_location_in_chamber << "\n";

			if (cur_rover_status == STOP_AT_DOOR) break;

			Sleep(500);
		}

		std::cout << "stop at door\n"; //std::getchar();
		Sleep(3000);
		sendRoboteqVar(2, 1);//enter chamber
		while (true)
		{
			//read current stop
			motor_controller_.GetValue(_VAR, 4, cur_rover_status);

			//std::cout << "cur_location_in_chamber " << cur_location_in_chamber << "\n";

			if (cur_rover_status == STOP_IN_CHAMBER) break;

			Sleep(500);
		}
		std::cout << "stop in chamber\n"; //std::getchar();
		Sleep(3000);
	}
	//if (cur_rover_status != STOP_IN_CHAMBER) return;

	int cur_location_in_chamber = -1;
	int target_stop = 1;
	//std::cout << "Hit Enter and go to location "<< target_stop<<"\n"; 	std::getchar();
	//sendRoboteqVar(3, target_stop);
	//
	//while (true)
	//{
	//	//read current stop
	//	motor_controller_.GetValue(_VAR, 5, cur_location_in_chamber);

	//	//std::cout << "cur_location_in_chamber " << cur_location_in_chamber << "\n";

	//	if (cur_location_in_chamber == target_stop) break;

	//	Sleep(500);
	//}

	//Sleep(1000);	
	//std::cout << "stop at " << cur_location_in_chamber << "\n";

	std::string cur_time_str = getCurrentDateTimeStr();

	std::wstring folder_name(cur_time_str.begin(), cur_time_str.end());

	data_saving_folder_ = L"Enviratron Data\\Chamber " + std::to_wstring(cur_chamber_id_) + L"\\" + folder_name + L"\\";

	std::cout << CreateDirectory(data_saving_folder_.c_str(), NULL);

	// Enter chamber and stop at middle position
	pot_process_status_.clear();
	pot_process_status_.resize(num_plants_);
	for (auto & s : pot_process_status_) s = false;

	plant_cluster_min_vec_.clear();
	plant_cluster_min_vec_.resize(num_plants_);
	plant_cluster_max_vec_.clear();
	plant_cluster_max_vec_.resize(num_plants_);

	
	mapWorkspaceUsingKinectArm(1, num_plants_);

#if 0
	if (!moveToConfigGetKinectPointCloud(home_config_, false, true, false))
	{
		std::cout << "can not go to home config\n";
		return;
	}

	sendRoboteqVar(1, 0);	//go home
	sendRoboteqVar(2, 2); //exit chamber
#endif

	return;

	// move arm back to a safe config before crab
	ArmConfig safe_config;
	safe_config.setJointPos(-90, -90, -90, -80, 90, -180);
	safe_config.toRad();

	if (!moveToConfigGetKinectPointCloud(safe_config, false, true, false))
	{
		return;
	}


	target_stop = 2;
	std::cout << "Hit Enter and go to location " << target_stop << "\n"; 	std::getchar();
	sendRoboteqVar(3, target_stop);

	while (true)
	{
		//read current stop
		motor_controller_.GetValue(_VAR, 5, cur_location_in_chamber);

		if (cur_location_in_chamber == target_stop) break;

		Sleep(500);
	}

	Sleep(1000);
	std::cout << "stop at " << cur_location_in_chamber << "\n";

	mapWorkspaceUsingKinectArm(2, num_plants_);

	if (!moveToConfigGetKinectPointCloud(safe_config, false, true, false))
	{
		return;
	}
	target_stop = 0;
	std::cout << "Hit Enter and go to location " << target_stop << "\n"; 	std::getchar();
	sendRoboteqVar(3, target_stop);

	while (true)
	{
		//read current stop
		motor_controller_.GetValue(_VAR, 5, cur_location_in_chamber);

		if (cur_location_in_chamber == target_stop) break;

		Sleep(500);
	}

	Sleep(1000);
	std::cout << "stop at " << cur_location_in_chamber << "\n";

	mapWorkspaceUsingKinectArm(0, num_plants_);

	std::cout << "done\n";
		

}

void VisionArmCombo::probePlateCenterTest()
{
	scanPlantCluster(plate_center_, plate_center_[2], plate_radius_);
}

void VisionArmCombo::scanPotMultiAngle()
{
	std::getchar();
	double angle = 30.;
	Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
	t.block<3, 3>(0, 0) = Eigen::AngleAxisd(angle/180.*M_PI, Eigen::Vector3d::UnitY()).matrix()*Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).matrix();
	t.col(3).head(3) << plate_center_[0], plate_center_[1], plate_center_[2];
	double pose[6];
	eigenMat4dToArray6(t, pose);
	robot_arm_client_->moveHandL(pose, move_arm_acceleration_, move_arm_speed_);
	//robot_arm_client_->waitTillHandReachDstPose(pose);

	PointCloudT::Ptr scan_cloud(new PointCloudT);
	PointCloudT::Ptr scan_cloud_in_base(new PointCloudT);
	PointCloudT::Ptr scan_cloud_in_base1(new PointCloudT);

	//wait 1 sec
	//std::cout << "scan plant " << object_idx << "\n";
	Sleep(1000);

	// moveToConfig moves the arm to a pose slightly different from scan_start_hand_pose
	Eigen::Matrix4f curPose; getCurHandPose(curPose);

	//std::cout << curPose << "\n" << scan_start_hand_pose << "\n";

	//scan_start_hand_pose = curPose.cast<double>();

	double scan_distance = 0.2;

	// Use laser scanner
	double vec[3] = {-scan_distance, 0, 0};

	line_profiler_->m_vecProfileData.clear();
	scanTranslateOnly(vec, scan_cloud, scan_acceleration_, scan_speed_);

	pcl::transformPointCloud(*scan_cloud, *scan_cloud_in_base, curPose*handToScanner_);

	t.block<3, 3>(0, 0) = Eigen::AngleAxisd(-2 * angle / 180.*M_PI, Eigen::Vector3d::UnitY()).matrix()*t.block<3, 3>(0, 0);
	t(0, 3) -= 3*scan_distance;

	eigenMat4dToArray6(t, pose);
	robot_arm_client_->moveHandL(pose, move_arm_acceleration_, move_arm_speed_);
	//robot_arm_client_->waitTillHandReachDstPose(pose);

	getCurHandPose(curPose);
	vec[0] = scan_distance;
	line_profiler_->m_vecProfileData.clear();
	scanTranslateOnly(vec, scan_cloud, scan_acceleration_, scan_speed_);

	pcl::transformPointCloud(*scan_cloud, *scan_cloud_in_base1, curPose*handToScanner_);

	scan_cloud->points.clear();
	*scan_cloud += *scan_cloud_in_base;
	*scan_cloud += *scan_cloud_in_base1;

	viewer_->addPointCloud(scan_cloud, "scan_cloud", 0);
	viewer_->spin();
	viewer_->removePointCloud("scan_cloud", 0);

	std::cout << "done\n";
}

void VisionArmCombo::TCPCalibrationErrorAnalysis(int numPoseNeeded)
{
	cv::FileStorage fs("tool_center_point_calib.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		ofstream myfile;
		myfile.open("Probe Tip Position.txt");
		
		cv::Vec3d tcp;
		fs["tcp"] >> tcp;

		Eigen::Vector4d tip_tool_frame;
		tip_tool_frame << tcp[0], tcp[1], tcp[2], 1.0;
	
		for (int i = 0; i < numPoseNeeded; i++)
		{
			cv::Vec6d cv_pose;
			fs["pose" + std::to_string(i)] >> cv_pose;

			double array6[6];

			for (int j = 0; j < 6; j++) array6[j] = cv_pose[j];

			Eigen::Matrix4d pose;

			array6ToEigenMat4d(array6, pose);

			Eigen::Vector4d tip_base_frame = pose*tip_tool_frame;

			std::cout << tip_base_frame << "\n";

			myfile << tip_base_frame(0) << "," << tip_base_frame(1) << "," << tip_base_frame(2) << "\n";
		}

		myfile.close();
	}
	fs.release();
}

void VisionArmCombo::acquireRGBStereoPair()
{
	if (robot_arm_client_ == NULL) return;
	if (kinect_thread_ == NULL)	return;

	PointCloudT::Ptr laser_cloud(new PointCloudT);

#if 1
	float scan_radius = 0.2;
	scanPlantCluster(plant_center_, plant_center_[2], scan_radius);

	plant_center_[1] += 0.1;
	scanPlantCluster(plant_center_, plant_center_[2], scan_radius);

	plant_center_[1] -= 2*0.1;
	scanPlantCluster(plant_center_, plant_center_[2], scan_radius);

	if (plant_laser_pc_vec_.size() == 0) return;

	if (plant_laser_pc_vec_[0]->points.size() == 0) return;

	for (auto & cloud : plant_laser_pc_vec_)
	{
		*laser_cloud += *cloud;
	}

	if (laser_cloud->points.size() != 0)
		pcl::io::savePCDFileBinary("Stereo dataset/plant_laser_cloud.pcd", *laser_cloud);

	//std::getchar();
#endif

	Eigen::Matrix4d rgb_pose1 = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d rgb1_to_rgb2 = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d rgb_pose2;
	Eigen::Matrix4d hand_pose1, hand_pose2;
	cv::Mat rgb1, rgb2;
	double array6[6];
	double baseline = 0.03;

	cv::Size img_size_cv;
	img_size_cv.height = kinect_thread_->cColorHeight;
	img_size_cv.width = kinect_thread_->cColorWidth;

	rgb_pose1(0, 3) = plate_center_[0];
	rgb_pose1(1, 3) = plate_center_[1];
	rgb_pose1(2, 3) = plate_center_[2];

	rgb_pose1.col(1).head<3>() << 0, -1, 0;
	rgb_pose1.col(2).head<3>() << 0, 0, -1;

	rgb1_to_rgb2(0, 3) = baseline;

	rgb_pose2 = rgb_pose1*rgb1_to_rgb2;

	//compute hand pose

	hand_pose1 = rgb_pose1*hand_to_rgb_.inverse();
	hand_pose2 = rgb_pose2*hand_to_rgb_.inverse();

	eigenMat4dToArray6(hand_pose1, array6);
	robot_arm_client_->moveHandL(array6, 0.1, 0.1);
	//robot_arm_client_->waitTillHandReachDstPose(array6);
	Sleep(1500);
	
	rgb1 = kinect_thread_->getCurRGB();


	eigenMat4dToArray6(hand_pose2, array6);
	robot_arm_client_->moveHandL(array6, 0.1, 0.1);
	//robot_arm_client_->waitTillHandReachDstPose(array6);
	Sleep(1500);

	rgb2 = kinect_thread_->getCurRGB();

	cv::Mat R, T, R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];

	R = cv::Mat::eye(3, 3, CV_64F);
	T = cv::Mat::zeros(3, 1, CV_64F);
	T.at<double>(0, 0) = baseline;

	cv::stereoRectify(kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_,
						kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_,
						img_size_cv, R, T, R1, R2, P1, P2, Q,
						cv::CALIB_ZERO_DISPARITY, 1, img_size_cv, &validRoi[0], &validRoi[1]);

	std::cout << "R1" << R1 << "\n" << "R2" << R2 << "\n" << "P1" << P1 << "\n" << "P2" << P2 << "\n";

	std::cout <<"Q" <<Q << "\n";

	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	cv::Mat rmap[2][2];
	initUndistortRectifyMap(kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_, R1, P1, img_size_cv, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_, R2, P2, img_size_cv, CV_16SC2, rmap[1][0], rmap[1][1]);

#if 0
	initUndistortRectifyMap(kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_, cv::Mat(), 
							kinect_rgb_camera_matrix_cv_, img_size_cv, CV_16SC2, rmap[0][0], rmap[0][1]);

	rmap[1][0] = rmap[0][0];
	rmap[1][1] = rmap[0][1];
	Q.at<double>(2, 3) = kinect_rgb_camera_matrix_cv_.at<double>(0, 0);
	Q.at<double>(0, 3) = -kinect_rgb_camera_matrix_cv_.at<double>(0, 2);
	Q.at<double>(1, 3) = -kinect_rgb_camera_matrix_cv_.at<double>(1, 2);
	Q.at<double>(3, 2) = -1. / baseline;

	std::cout << "Q" << Q << "\n";
#endif

	cv::Mat canvas;
	double sf;
	int w, h;
	if (!isVerticalStereo)
	{
		sf = 600. / std::max(img_size_cv.width, img_size_cv.height);
		w = cvRound(img_size_cv.width*sf);
		h = cvRound(img_size_cv.height*sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / std::max(img_size_cv.width, img_size_cv.height);
		w = cvRound(img_size_cv.width*sf);
		h = cvRound(img_size_cv.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}

	cv::Mat rimg1, rimg2;
	
	cv::remap(rgb1, rimg1, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
	cv::remap(rgb2, rimg2, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);

	cv::imwrite("Stereo dataset/im0.png", rimg1);
	cv::imwrite("Stereo dataset/im1.png", rimg2);

	cv::Mat canvasPart1 = !isVerticalStereo ? canvas(cv::Rect(0, 0, w, h)) : canvas(cv::Rect(0, h, 0, h));
	cv::Mat canvasPart2 = !isVerticalStereo ? canvas(cv::Rect(w, 0, w, h)) : canvas(cv::Rect(0, h, w, h));

	resize(rimg1, canvasPart1, canvasPart1.size(), 0, 0, cv::INTER_AREA);
	resize(rimg2, canvasPart2, canvasPart2.size(), 0, 0, cv::INTER_AREA);
	
	
	cv::Rect vroi1(cvRound(validRoi[0].x*sf), cvRound(validRoi[0].y*sf),	cvRound(validRoi[0].width*sf), cvRound(validRoi[0].height*sf));
	cv::Rect vroi2(cvRound(validRoi[1].x*sf), cvRound(validRoi[1].y*sf), cvRound(validRoi[1].width*sf), cvRound(validRoi[1].height*sf));

	cv::rectangle(canvasPart1, vroi1, cv::Scalar(0, 0, 255), 3, 8);
	cv::rectangle(canvasPart2, vroi2, cv::Scalar(0, 0, 255), 3, 8);

	if (!isVerticalStereo)
		for (int j = 0; j < canvas.rows; j += 16)
			line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);
	else
		for (int j = 0; j < canvas.cols; j += 16)
			line(canvas, cv::Point(j, 0), cv::Point(j, canvas.rows), cv::Scalar(0, 255, 0), 1, 8);
	imshow("rectified", canvas);

	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);


	sgbm->setPreFilterCap(63);
	int sgbmWinSize = 5;
	sgbm->setBlockSize(sgbmWinSize);

	int cn = rgb1.channels();
	int numberOfDisparities = 16 * 6;

	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
	sgbm->setMinDisparity(0);
	sgbm->setNumDisparities(numberOfDisparities);
	sgbm->setUniquenessRatio(10);
	sgbm->setSpeckleWindowSize(200);
	sgbm->setSpeckleRange(32);
	sgbm->setDisp12MaxDiff(1);
	
	sgbm->setMode(cv::StereoSGBM::MODE_SGBM);	//MODE_SGBM MODE_HH MODE_SGBM_3WAY

	cv::Mat disp, disp8, disp32f;
	sgbm->compute(rgb1, rgb2, disp);

	disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));

	cv::Mat disp8_small;

	cv::resize(disp8, disp8_small, cv::Size(), 0.4, 0.4, 1);
	
	imshow("disparity", disp8_small);

	disp.convertTo(disp32f, CV_32F, 1.0/16);

	cv::Mat xyz;
	cv::reprojectImageTo3D(disp32f, xyz, Q, true);


	PointCloudT::Ptr stereo_cloud(new PointCloudT);

	const double max_z = 1.5;
	
	for (int y = 0; y < xyz.rows; y++)
	{
		for (int x = 0; x < xyz.cols; x++)
		{
			cv::Vec3f point = xyz.at<cv::Vec3f>(y, x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;

			cv::Vec3b bgr = rgb1.at<cv::Vec3b>(y, x);
			PointT p;
			p.x = -point[0];
			p.y = -point[1];
			p.z = -point[2];
			p.b = bgr[0];
			p.g = bgr[1];
			p.r = bgr[2];
			//int intensity = ((int)(p.z * 10000)) % 255;
			//p.r = intensity; p.b = intensity; p.g = intensity;
			stereo_cloud->push_back(p);

			//std::cout << point << " ";
		}
	}

	cv::FileStorage fs("Stereo dataset/rgb_pose1.yml", cv::FileStorage::WRITE);

	cv::Mat rgb_pose1_cv;
	rgb_pose1_cv.create(4, 4, CV_64F);
	for (int y = 0; y < 4; y++)
	{
		for (int x = 0; x < 4; x++)
		{
			rgb_pose1_cv.at<double>(y, x) = rgb_pose1(y, x);
		}
	}

	fs << "rgb_pose1" << rgb_pose1_cv;

	fs << "Q" << Q;

	fs.release();

	cv::waitKey(100);

	pcl::transformPointCloud(*stereo_cloud, *stereo_cloud, rgb_pose1);
	PointCloudT::Ptr tmp_cloud(new PointCloudT);
	pass_.setInputCloud(stereo_cloud);
	pass_.setFilterFieldName("z");
	pass_.setFilterLimits(shelf_z_, 1.5f);
	pass_.setFilterLimitsNegative(false);
	pass_.filter(*tmp_cloud);

	stereo_cloud->points.clear();
	vox_.setLeafSize(voxel_grid_size_laser_, voxel_grid_size_laser_, voxel_grid_size_laser_);
	vox_.setInputCloud(tmp_cloud);
	vox_.filter(*stereo_cloud);

	viewer_->addPointCloud(stereo_cloud, "stereocloud", 0);
	viewer_->removeAllShapes();
	viewer_->addCube(min_point_AABB_(0), max_point_AABB_(0),
					min_point_AABB_(1), max_point_AABB_(1),
					min_point_AABB_(2), max_point_AABB_(2));
	viewer_->spin();

	clock_t tic = clock();

	int stereo_cloud_size = stereo_cloud->points.size();
	int laser_cloud_size = laser_cloud->points.size();


	float* stereo_cloud_xyz = new float[stereo_cloud_size*3];
	float* laser_cloud_xyz = new float[laser_cloud_size*3];

	for (int i = 0; i < stereo_cloud_size; i++)
	{
		stereo_cloud_xyz[3 * i] = stereo_cloud->points[i].x;
		stereo_cloud_xyz[3 * i + 1] = stereo_cloud->points[i].y;
		stereo_cloud_xyz[3 * i + 2] = stereo_cloud->points[i].z;
	}

	for (int i = 0; i < laser_cloud_size; i++)
	{
		laser_cloud_xyz[3 * i] = laser_cloud->points[i].x;
		laser_cloud_xyz[3 * i + 1] = laser_cloud->points[i].y;
		laser_cloud_xyz[3 * i + 2] = laser_cloud->points[i].z;
	}

	flann::Matrix<float> laser_cloud_mat = flann::Matrix<float>(laser_cloud_xyz, laser_cloud_size, 3);
	flann::Matrix<float> stereo_cloud_mat = flann::Matrix<float>(stereo_cloud_xyz, stereo_cloud_size, 3);

	flann::Index<flann::L2<float>> flann_index(laser_cloud_mat, flann::KDTreeSingleIndexParams(1));

	flann_index.buildIndex();

	// neighbor index
	flann::Matrix<int> stereo_neighbor_indices_mat(new int[stereo_cloud_size], stereo_cloud_size, 1);

	// distance 
	flann::Matrix<float> stereo_neighbor_dists_mat(new float[stereo_cloud_size], stereo_cloud_size, 1);

	flann_index.knnSearch(stereo_cloud_mat, stereo_neighbor_indices_mat, stereo_neighbor_dists_mat, 1, flann::FLANN_CHECKS_UNLIMITED);


	clock_t toc = clock();

	printf("compare clouds Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

	ofstream file;
	file.open("Stereo dataset/cloud-to-cloud distances.txt");
	for (int i = 0; i < stereo_cloud_size; i++)
		file << stereo_neighbor_dists_mat.ptr()[i] << ",";
	file.close();

	//viewer_->addPointCloud(plant_laser_pc_vec_[0], "lasercloud", 0);
	//viewer_->spin();

	std::cout << "Done\n";

	delete[] stereo_cloud_xyz;
	delete[] laser_cloud_xyz;

}

bool VisionArmCombo::switchBetweenImagingAndProbing(int target_mode) {

	Eigen::Matrix4f pose;

	getCurHandPose(pose);

	if (target_mode == IMAGING) {


		if (pose(2, 3) > 0.f) {	//hand point up


		}

	}
	else if (target_mode == PROBING) {


	}
	else {

		return false;
	}




}

void VisionArmCombo::controlChamberDoor(int chamber_id, int action)
{
	
	boost::asio::io_service io_service;
	boost::asio::ip::udp::socket socket(io_service);
	boost::asio::ip::udp::endpoint remote_endpoint;

	socket.open(boost::asio::ip::udp::v4());

	remote_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string("10.25.215.211"), 10000);

	boost::system::error_code err;

	std::string command;

	if (action == OPEN_DOOR)
		command = "open door " + std::to_string(chamber_id) + " rover";
	else if(action == CLOSE_DOOR)
		command = "close door " + std::to_string(chamber_id) + " rover";
	else if (action == OPEN_CURTAIN)
		command = "open curtain " + std::to_string(chamber_id) + " rover";
	else if (action == CLOSE_CURTAIN)
		command = "close curtain " + std::to_string(chamber_id) + " rover";

	socket.send_to(boost::asio::buffer(command, command.size()), remote_endpoint, 0, err);

	socket.close();
}