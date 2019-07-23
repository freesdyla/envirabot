#include "VisionArmCombo.h"

#define ENABLE_PP	//path planner

VisionArmCombo::VisionArmCombo()
{
	//pp_.PRMCEPreprocessing(); pp_.savePathPlanner("pp");

	viewer_.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer_->addCoordinateSystem(0.3);
	viewer_->setSize(1366, 768);
	viewer_->setPosition(0, 100);
	viewer_->registerPointPickingCallback(&VisionArmCombo::pp_callback, *this);
	view_time_ = 50000000;

	initVisionCombo();

#if 0
	PointCloudT::Ptr cloud0(new PointCloudT);
	PointCloudT::Ptr cloud1(new PointCloudT);
	pcl::io::loadPCDFile("cloud0.pcd", *cloud0);
	pcl::io::loadPCDFile("cloud1.pcd", *cloud1);

	//	*cloud1 += *cloud0;

	//for (auto & p : cloud1->points)	p.x *= -1;

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	//transform.col(3) << 0.4f, 0.f, 0.f, 1.f;

	transform.block<3, 3>(0, 0) = Eigen::AngleAxisf(pcl::deg2rad(-180.), Eigen::Vector3f::UnitZ()).matrix();

	pcl::transformPointCloud(*cloud0, *cloud1, transform);

	std::vector<pcl::PointXYZRGBNormal> probe_pn_vec;
	std::vector<pcl::PointIndices> leaf_cluster_indices_vec;
	std::vector<std::vector<Eigen::Matrix4d*>> hyperscan_hand_pose_sequence_vec;
	std::vector<std::vector<ArmConfig>> hyperscan_arm_config_sequence_vec;
	std::vector<int> hyperscan_leaf_id_vec;

	extractLeafProbingPointsAndHyperspectralScanPoses(cloud1, 
		leaf_cluster_indices_vec,
		hyperscan_hand_pose_sequence_vec,
		hyperscan_arm_config_sequence_vec,
		hyperscan_leaf_id_vec, 0, SIDE_VIEW);

	return;
#endif
	

#ifdef ENABLE_PP
	if (!pp_.loadPathPlanner("pp"))	std::cout << "load path planner fail\n";
	//for (int i = 444; i<pp_.num_nodes_; i += 2) viewPlannedPath(pp_.random_nodes_buffer_ + (i - 1) * 6, pp_.random_nodes_buffer_ + i * 6); std::getchar(); close();
#endif

	//auto upload_thread = std::async(&ServerManager::uploadDirectory, &data_server_manager_, "c7_2018_7_17_16_7_15_492", "experiment"); upload_thread.get();  close();
	//	float Fo, Fm, FvFm, qP, qL, qN, NPQ, YNPQ, YNO, F, FmPrime, PAR, YII, ETR, FoPrime;
	//	MiniPamActPlusYield(Fo, Fm, FvFm, qP, qL, qN, NPQ, YNPQ, YNO, F, FmPrime, PAR, YII, ETR, FoPrime);

	//	initRaman();
	/*	raman_->getSpectrum(0, 7000);
	raman_->saveLastSpectrum("raman.csv");

	raman_->getSpectrum(1, 7000);
	raman_->saveLastSpectrum("raman_ir.csv");
	*/

	if (enable_scanner_) initLineProfiler();
	if (enable_hyperspectral_) initHyperspectralCam();	//white board, best exposure  no more than 17 ms
	if (enable_thermo_) initThermoCam();
	if (enable_rgb_) initRGBCamera();
	if (enable_tof_) initTOFCamera();	
	if(enable_arm_) initRobotArmClient();
}

VisionArmCombo::~VisionArmCombo()
{

}

void VisionArmCombo::initVisionCombo()
{
	home_config_.setJointPos(-201.25, 6.4, -141.1, -46.5, 110.91, -179.73);
	home_config_.toRad();
	
	sideview_home_config_.setJointPos(16.65,-88.05,-142.42,14.82,75.43,-171.09);
	sideview_home_config_.toRad();

	// load line scanner hand eye calibration matrix
	guessHandToScanner_ = Eigen::Matrix4f::Identity();
	guessHandToScanner_.block<3, 3>(0, 0) = Eigen::AngleAxisf(-0.5*M_PI, Eigen::Vector3f::UnitZ()).matrix();
	guessHandToScanner_.col(3).head(3) << 0.076073, -0.00502, 0.09425;

	handToScanner_ = guessHandToScanner_;

	hand_to_scanner_ = guessHandToScanner_.cast<double>();

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
				handToScanner_.row(i)(j) = *(ptr);
				hand_to_scanner_.row(i)(j) = *(ptr);
			}
		std::cout << "handToScanner:\n" << handToScanner_ << "\n";
	}
	else std::cout << "lineScannerHandEyeCalibration load fail\n";
	file.close();

	// hyperspectral camera pose
	hand_to_hyperspectral_d_ = Eigen::Matrix4d::Identity();
	hand_to_hyperspectral_d_.block<3, 3>(0, 0) = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitZ()).matrix();
	hand_to_hyperspectral_d_.col(3).head(3) << 0.046736, 0.062186, 0.11;
	hand_to_hyperspectral_ = hand_to_hyperspectral_d_.cast<float>();

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

	probe_to_hand_pam_ = probe_to_hand_pam_.inverse();

	fs.open("tool_center_point_calib_RAMAN_532.yml", cv::FileStorage::READ);

	if (fs.isOpened())
		fs["tcp"] >> tcp;

	fs.release();

	for (int i = 0; i < 3; i++) tool_center_point_raman_532_(i) = tcp[i];

	probe_to_hand_raman_532_ = Eigen::Matrix4d::Identity();

	probe_to_hand_raman_532_.col(3).head<3>() = tool_center_point_raman_532_.cast<double>();

	probe_to_hand_raman_532_ = probe_to_hand_raman_532_.inverse();

	fs.open("tool_center_point_calib_RAMAN_1064.yml", cv::FileStorage::READ);

	if (fs.isOpened())
		fs["tcp"] >> tcp;

	fs.release();

	for (int i = 0; i < 3; i++) tool_center_point_raman_1064_(i) = tcp[i];

	probe_to_hand_raman_1064_ = Eigen::Matrix4d::Identity();

	probe_to_hand_raman_1064_.col(3).head<3>() = tool_center_point_raman_1064_.cast<double>();

	probe_to_hand_raman_1064_ = probe_to_hand_raman_1064_.inverse();


	fs.open("RGBCalibration.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["camera_matrix"] >> rgb_camera_matrix_cv_;

		fs["distortion_coefficients"] >> rgb_dist_coeffs_cv_;
	}

	fs.release();

	fs.open("RGBHandEyeCalibration.yml", cv::FileStorage::READ);

	if (fs.isOpened())
		fs["hand to eye"] >> rgb_hand_to_eye_cv_;

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

	fs.open("IRCalibration.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["camera_matrix"] >> infrared_camera_matrix_cv_;

		fs["distortion_coefficients"] >> infrared_dist_coeffs_cv_;

		fs.release();
	}

	fs.open("IRHandEyeCalibration.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["hand to eye"] >> infrared_hand_to_eye_cv_;

		fs.release();
	}

	cvTransformToEigenTransform(infrared_hand_to_eye_cv_, hand_to_depth_);

	cvTransformToEigenTransform(rgb_hand_to_eye_cv_, hand_to_rgb_);

	cvTransformToEigenTransform(flir_thermal_hand_to_eye_cv_, hand_to_thermal_d_);

	hand_to_thermal_ = hand_to_thermal_d_.cast<float>();

	readOrUpdateChamberPotConfigurationFile(READ_POT_CONFIG);

	// entry full
	fs.open("parameters.yml", cv::FileStorage::READ);
	if (fs.isOpened())
	{
		fs["shelf_z_"] >> shelf_z_;
		fs["pot_height_"] >> pot_height_;
		fs["voxel_grid_size_laser_"] >> voxel_grid_size_laser_;
		fs["normal_estimation_radius_"] >> normal_estimation_radius_;
		fs["region_grow_residual_threshold_"] >> region_grow_residual_threshold_;
		fs["region_grow_smoothness_threshold_"] >> region_grow_smoothness_threshold_;
		region_grow_smoothness_threshold_ = region_grow_smoothness_threshold_ / 180.*M_PI;
		fs["region_grow_curvature_threshold_"] >> region_grow_curvature_threshold_;
		fs["region_grow_num_neighbors_"] >> region_grow_num_neighbors_;
		fs["region_grow_min_cluster_size_"] >> region_grow_min_cluster_size_;
		fs["region_grow_max_cluster_size_"] >> region_grow_max_cluster_size_;
		double f;
		fs["laser_scan_frequency_"] >> f;
		laser_scan_period_ = 1.0 / f;
		fs["scan_speed_"] >> scan_speed_;
		fs["scan_acceleration_"] >> scan_acceleration_;
		fs["move_arm_speed_"] >> move_arm_speed_;
		fs["move_arm_acceleration_"] >> move_arm_acceleration_;
		fs["move_joint_speed_"] >> move_joint_speed_;
		fs["move_joint_acceleration_"] >> move_joint_acceleration_;
		fs["seed_resolution_"] >> seed_resolution_;
		fs["spatial_importance_"] >> spatial_importance_;
		fs["normal_importance_"] >> normal_importance_;
		fs["sor_mean_k_"] >> sor_mean_k_;
		fs["sor_std_"] >> sor_std_;
		fs["ror_radius_"] >> ror_radius_;
		fs["ror_num_"] >> ror_num_;
		fs["hyperspectral_imaging_dist_"] >> hyperspectral_imaging_dist_;
		fs["max_samples_per_leaf_"] >> max_samples_per_leaf_;
		fs["max_samples_per_plant_"] >> max_samples_per_plant_;
		fs["probe_patch_max_curvature_"] >> probe_patch_max_curvature_;
		fs["probing_patch_rmse_"] >> probing_patch_rmse_;
		fs["hyperscan_slice_thickness_"] >> hyperscan_slice_thickness_;
		fs["enable_arm_"] >> enable_arm_;
		fs["enable_probing_"] >> enable_probing_;
		fs["vis_z_range_"] >> vis_z_range_;
		fs["enable_thermo_"] >> enable_thermo_;
		fs["enable_tof_"] >> enable_tof_;
		fs["enable_rgb_"] >> enable_rgb_;
		fs["enable_hyperspectral_"] >> enable_hyperspectral_;
		fs["enable_scanner_"] >> enable_scanner_;
		fs["remap_pot_position_"] >> remap_pot_position_;
		fs["only_do_probing_"] >> only_do_probing_;
		fs["view_time_"] >> view_time_;
		fs["multi_work_position_"] >> multi_work_position_;
		fs["hyperspectral_topview_"] >> hyperspectral_topview_;
		fs["leaf_tracing_hyperspectral_"] >> leaf_tracing_hyperspectral_;
		fs["enable_imaging_"] >> enable_imaging_;
		fs["start_time_"] >> start_time_;
		fs["scheduled_minutes_per_chamber_"] >> scheduled_minutes_per_chamber_;
		fs["sideview_camera_x_pos_"] >> sideview_camera_x_pos_;
		fs["close_door_when_rover_inside_"] >> close_door_when_rover_inside_;
		fs["charing_time_2_work_time_ratio_"] >> charing_time_2_work_time_ratio_;
	}
	fs.release();

	fs.open("TaskProgress.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["inside_or_outside_chamber_"] >> inside_or_outside_chamber_;
		fs["rover_run_index_in_chamber_"] >> rover_run_index_in_chamber_;
		fs["rover_pos_in_chamber_"] >> rover_pos_in_chamber_;
		fs["cur_chamber_id_"] >> cur_chamber_id_;
		fs["data_collection_mode_"] >> data_collection_mode_;
		fs["imaging_or_probing_"] >> imaging_or_probing_;
		fs["data_collection_done_"] >> data_collection_done_;
		fs["plant_imaging_stage_"] >> plant_imaging_stage_;
		fs["data_folder_"] >> data_folder_;
		fs["experiment_id_"] >> experiment_id_;
		fs["pot_processed_map_"] >> pot_processed_map_;
		fs["pot_sensor_map_"] >> pot_sensor_map_;
	}

	fs.release();

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

	tof_cloud_.reset(new PointCloudT);

	laser_cloud_.reset(new PointCloudT);

	chamber_occupancy_cloud_.reset(new PointCloudT);

	probe_points_.reset(new PointCloudT);

	sor_.setMeanK(sor_mean_k_);
	sor_.setStddevMulThresh(sor_std_);
	ror_.setRadiusSearch(ror_radius_);
	ror_.setMinNeighborsInRadius(ror_num_);
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

void VisionArmCombo::initHyperspectralCam() {

	hypercam_ = new HyperspectralCamera();

	hypercam_->init();
}

void VisionArmCombo::initThermoCam() {

	ShellExecute(NULL, TEXT("open"), TEXT("C:\\Users\\lietang123\\Documents\\RoAdFiles\\FlirThermoCamServer\\FlirThermoCamServer\\bin\\Release\\FlirThermoCamServer.exe"), NULL, NULL, SW_SHOWDEFAULT);

	//Sleep(6000);

	thermocam_ = new FlirThermoCamClient();
}

void VisionArmCombo::initRaman()
{
	//raman_ = new Raman();
}

float VisionArmCombo::DDERequest(DWORD idInst, HCONV hConv, wchar_t* szItem)
{
	HSZ hszItem = DdeCreateStringHandle(idInst, szItem, 0);
	HDDEDATA hData = DdeClientTransaction(NULL, 0, hConv, hszItem, CF_TEXT,
		XTYP_REQUEST, 2000, NULL);
	if (hData == NULL)
	{
		printf("Request failed: %ls\n", szItem);
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
		DdeUninitialize(idInst);
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
			printf("Request failed: %ls\n", variable_name);
		else
		{
			char szResult[4];
			DdeGetData(hData, (unsigned char *)szResult, 4, 0);

			//printf("%s\n", szResult);

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
			printf("Request failed: %ls\n", variable_name);
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
	double dist = magnitudeVec3(vec3d);

	if (dist < 0.05) return;

	if (acceleration < 1e-7 || speed < 1e-7)
	{
		std::cout << "acceleration or speed = 0\n";
		return;
	}

	if (robot_arm_client_ == NULL || line_profiler_ == NULL)
	{
		std::cout << "robot arm or line profiler not initialized \n";
		return;
	}

	line_profiler_->m_vecProfileData.clear();

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

	const int num_profiles = line_profiler_->m_vecProfileData.size();

	const double sync_speed = sqrt(tcp_sync_speed[0] * tcp_sync_speed[0] + tcp_sync_speed[1] * tcp_sync_speed[1] + tcp_sync_speed[2] * tcp_sync_speed[2]);

	std::cout <<"tcp sync speed: "<<sync_speed<< "\n";

	std::cout << "num profiles: " << num_profiles << std::endl;

	Eigen::Matrix4d startPose;
	Eigen::Matrix4d endPose;

	array6ToEigenMat4d(curPoseD, startPose);
	array6ToEigenMat4d(endPoseD, endPose);

	double sync_distance = robot_arm_client_->EuclideanDistance(curPoseD, sync_pose);

	std::cout << "sync distance: " << sync_distance << "\n";

	//std::cout << "calculated end pose\n" << endPose << "\n\n";

	/*double testPoseD[6];
	robot_arm_client_->getCartesianInfo(testPoseD);
	Eigen::Matrix4f testPose;
	array6ToEigenMat4(testPoseD, testPose);

	std::cout << "true end pose\n" << testPose << "\n\n";*/


	// express motion vector in base frame
	Eigen::Vector3d motionVector( endPoseD[0] - curPoseD[0],
							      endPoseD[1] - curPoseD[1],
								  endPoseD[2] - curPoseD[2] );

	//std::cout << "motion vector in base frame:" << motionVector << std::endl;

	// transform motion vector in scanner frame
	/*Eigen::Matrix3f m;
	m = startPose.block<3, 3>(0,0);
	// A'*B' = (BA)' where A and B are rotation matrices. vec_senosr = Te2s*Tb2e*vec_base
	m = m*Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitZ());
	motionVector = m.transpose()*motionVector;*/

	motionVector = (hand_to_scanner_.block<3,3>(0,0)).transpose()*(startPose.block<3, 3>(0, 0)).transpose()*motionVector;

	//std::cout << "motion vector in sensor frame:" << motionVector << std::endl;

	const double magnitude = motionVector.norm();

	Eigen::Vector3d motionDelta = motionVector / (num_profiles - 1);

	//std::cout << "motion delta:" << motionDelta << std::endl;

	//std::cout << "x start: " << line_profiler_->m_profileInfo.lXStart << " pitch: " << line_profiler_->m_profileInfo.lXPitch << "\n";

	double distance = 0.f;

	double time = 0.f;

	time = sync_speed / acceleration;

	std::cout << "start time:" << time << "\n";

	time = sqrt(sync_distance * 2 / acceleration);

	std::cout << "start time from dist: " << time << "\n";

	const double start_cruise_time = speed/acceleration;

	const double stop_cruise_time = start_cruise_time + (magnitude -start_cruise_time*start_cruise_time*acceleration)/speed;

	const double cruise_time = stop_cruise_time - start_cruise_time;

	const double total_time = start_cruise_time + stop_cruise_time;

	const double acceleration_total_distance = 0.5f*acceleration*start_cruise_time*start_cruise_time;
	
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
		Eigen::Vector3d displacement = motionVector*(distance + speed_correction_);

		for (int j = 0; j < 800; j++)
		{
			PointT point;

			point.z = line_profiler_->m_vecProfileData[i].m_pnProfileData[j] * (-1e-8f);

			if (abs(point.z) < 0.148f)
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

	std::cout << "laser cloud size: " << cloud->size() << std::endl;
}

void VisionArmCombo::scanMoveL(Eigen::Matrix4d & dst_scanner_pose, PointCloudT::Ptr cloud, float acceleration, float speed)
{
	robot_arm_client_->laserScannerLightControl(true);

	double array6d[6];

	robot_arm_client_->getCartesianInfo(array6d);

	line_profiler_->m_vecProfileData.clear();
	robot_arm_client_->timestamp_pose_vec_.clear();
	robot_arm_client_->ur_timestamp_vec_.clear();

	double expected_sync_speed = 0.001;

	Eigen::Matrix4d dst_hand_pose;

	dst_hand_pose = dst_scanner_pose*hand_to_scanner_.inverse();

	eigenMat4dToArray6(dst_hand_pose, array6d);

	robot_arm_client_->moveHandL(array6d, acceleration, speed, false);
	
	robot_arm_client_->record_poses_.store(true);

	Sleep(100);

	line_profiler_->start(20);

	LARGE_INTEGER profiler_start_count; QueryPerformanceCounter(&profiler_start_count);

	robot_arm_client_->waitTillHandReachDstPose(array6d);

	line_profiler_->stop();

	robot_arm_client_->record_poses_.store(false);

	//wait for stop recording pose in the other robot arm client thread
	Sleep(1000);

	const double ur_reference_time = robot_arm_client_->ur_timestamp_vec_[0];

	for (auto & t : robot_arm_client_->ur_timestamp_vec_)
		t = (t-ur_reference_time)*1000.;
	
	// register point cloud
	cloud->clear();

	const int num_profiles = line_profiler_->m_vecProfileData.size();

	std::cout << "num_proflies: " << num_profiles << std::endl;

	int start_idx = 1;

	LARGE_INTEGER frequency; QueryPerformanceFrequency(&frequency);

	// time unit millisecond, use the count of the first recorded pose as the reference
	const LONGLONG ur_first_pose_count = robot_arm_client_->timestamp_pose_vec_.front().timestamp.QuadPart;
	const double start_scan_time = ( (profiler_start_count.QuadPart- ur_first_pose_count )*1000)/(double)(frequency.QuadPart);	// ms

	std::cout << "start_scan_time " << start_scan_time<<"\n";

	for (int line_idx = 0; line_idx < num_profiles; line_idx++)
	{
		double line_time = start_scan_time + line_idx*0.5;//sampling_period;

		for (int pose_idx = start_idx; pose_idx < robot_arm_client_->ur_timestamp_vec_.size(); pose_idx++)
		{
			double pose_time = robot_arm_client_->ur_timestamp_vec_[pose_idx];
			
			if (line_time <= pose_time)
			{
				start_idx = pose_idx;

				Eigen::Matrix4d cur_scanner_pose, hand_pose0, hand_pose1;

				array6ToEigenMat4d(robot_arm_client_->timestamp_pose_vec_[pose_idx-1].pose, hand_pose0);

				array6ToEigenMat4d(robot_arm_client_->timestamp_pose_vec_[pose_idx].pose, hand_pose1);
				
				// interpolate 3D pose
				Eigen::Quaterniond q0(hand_pose0.topLeftCorner<3,3>());

				Eigen::Quaterniond q1(hand_pose1.topLeftCorner<3, 3>());

				if (q1.dot(q0) < 0.) std::cout << "quaternon dot product negative\n";

				const double pre_pose_time = robot_arm_client_->ur_timestamp_vec_[pose_idx-1];

				const double t = (pose_time - line_time)/(pose_time - pre_pose_time);

				cur_scanner_pose = Eigen::Matrix4d::Identity();
				cur_scanner_pose.topLeftCorner<3,3>() = q1.slerp(t, q0).toRotationMatrix();
				cur_scanner_pose.col(3).head(3) = (1. - t)*hand_pose1.col(3).head(3) + t*hand_pose0.col(3).head(3);

				cur_scanner_pose = cur_scanner_pose * hand_to_scanner_;

				for (int pixel_idx = 0; pixel_idx < 800; pixel_idx++)
				{
					Eigen::Vector4d point;

					point(2) = line_profiler_->m_vecProfileData[line_idx].m_pnProfileData[pixel_idx] * (-1e-8);

					point(3) = 1.0;

					if (std::abs(point(2)) < 0.149)
					{
						point(0) = (double)(line_profiler_->m_profileInfo.lXStart + pixel_idx*line_profiler_->m_profileInfo.lXPitch)*(1e-8);
						point(1) = 0.;
						point(2) += 0.3;
						point = cur_scanner_pose*point;

						PointT p;
						p.x = point(0);
						p.y = point(1);
						p.z = point(2);

						p.r = ((int)(point(2) * 10000)) % 255;//(uint8_t)(255.f - 255.f*(point.z + 0.141f) / 0.282f);
						p.g = p.r;
						p.b = 255;

						cloud->push_back(p);
					}
				}

				break;
			}
		}
	}

	std::cout << "laser cloud size: " << cloud->size() << std::endl;
}

// assuming already moved to the first pose
void VisionArmCombo::scanLeafWithHyperspectral(std::vector<Eigen::Matrix4d*> & valid_scan_hand_pose_sequence,
												std::vector<ArmConfig> & valid_arm_config_sequence, int plant_id, int imaging_mode) 
{
	if (valid_scan_hand_pose_sequence.size() == 0) 
	{
		std::cout << "valid_scan_hand_pose_sequence size 0\n";
		return;
	}

	if (robot_arm_client_ == NULL || hypercam_ == NULL)
	{
		std::cout << "robot arm or line profiler not initialized \n";
		return;
	}

	// reverse valid_scan_hand_pose_sequence if not moving in negative x direction of base frame
	if (valid_scan_hand_pose_sequence.size() > 1) 
	{
		Eigen::Vector3d general_moving_dir = valid_scan_hand_pose_sequence.back()->col(3).head(3)
			- valid_scan_hand_pose_sequence.front()->col(3).head(3);

		if (general_moving_dir(0) > 0) 
		{
			std::reverse(std::begin(valid_scan_hand_pose_sequence), std::end(valid_scan_hand_pose_sequence));
			std::reverse(std::begin(valid_arm_config_sequence), std::end(valid_arm_config_sequence));
			std::cout << "reversed trajectory\n";
		}
	}

	double array6[6];
	std::vector<cv::Mat> hyperspectral_frames;

	if (imaging_mode == TOP_VIEW)
	{
		if (!moveToConfigGetPointCloud(valid_arm_config_sequence[0])) {

			std::cout << "Path to hyperspectral imaging pose not found\n";
			return;
		}
	}
	else if (imaging_mode == SIDE_VIEW)
	{
	//	robot_arm_client_->moveHandJ(valid_arm_config_sequence[0].joint_pos_d, move_joint_speed_, move_joint_acceleration_);
	}

	if (cur_chamber_config_.light_status == 1)
		controlChamber(cur_chamber_id_, PAUSE_LIGHT);
	
	// move to first pose
	eigenMat4dToArray6(*valid_scan_hand_pose_sequence[0], array6);
	robot_arm_client_->moveHandL(array6, 0.05, 0.05);

	//start imaging
	if (hypercam_->start(LAST_FRAME) < 0)
	{
		std::cout << "start last frame fail\n";

		hypercam_->stop();
	}

	while (hypercam_->frame_count_.load() == 0)
		Sleep(10);

	// collect the first frame
	hypercam_->dropBufferedFrames();
	hyperspectral_frames.push_back(hypercam_->getLastestFrame());

	for (int i = 1; i < valid_scan_hand_pose_sequence.size(); i++) 
	{
		eigenMat4dToArray6(*valid_scan_hand_pose_sequence[i], array6);
		robot_arm_client_->moveHandL(array6, 0.05, 0.05);
		hypercam_->dropBufferedFrames();
		Sleep(100);
		hyperspectral_frames.push_back(hypercam_->getLastestFrame());
	}

	hypercam_->dropBufferedFrames();
	hypercam_->stop();

	controlChamber(cur_chamber_id_, RESUME_LIGHT);

	//save data
	std::string time_str = getCurrentDateTimeStr();

	std::string folder(data_saving_folder_.begin(), data_saving_folder_.end());

	std::string camera_pose_file_name = folder + "hs_poses_" + "_" + std::to_string(plant_id)
										+ "_" + time_str + ".yml";
	
	cv::FileStorage fs(camera_pose_file_name, cv::FileStorage::WRITE);

	std::string img_file_name = folder + "hs_lt_" + std::to_string(plant_id) + "_f_" + std::to_string(hyperspectral_frames.size())
								+ "_w_" + std::to_string(hyperspectral_frames[0].cols) + "_h_" + std::to_string(hyperspectral_frames[0].rows)
								+ time_str + ".bin";

	std::ofstream out(img_file_name, std::ios::out | std::ios::binary);

	for (int i = 0; i < valid_scan_hand_pose_sequence.size(); i++) {

		out.write((char*)hyperspectral_frames[i].data, hypercam_->frame_data_size_);

		std::string cam_pose_str = "pose_" + std::to_string(i);

		Eigen::Matrix4d came_pose = (*valid_scan_hand_pose_sequence[i])*hand_to_hyperspectral_d_;

		cv::Mat cam_pose_cv;

		EigenTransformToCVTransform(came_pose, cam_pose_cv);

		fs << cam_pose_str << cam_pose_cv;
	}

	fs.release();

	out.close();

	//blue 440 nm 35, green 540 nm 110, red 600 nm 155
	int R_row = std::round(155 * 0.125);
	int G_row = std::round(110 * 0.125);
	int B_row = std::round(35 * 0.125);

	cv::Mat img_r_64, img_g_64, img_b_64, img_64;

	img_64.create(hyperspectral_frames.size(), hypercam_->spatial_size_, CV_64F);
	img_r_64.create(hyperspectral_frames.size(), hypercam_->spatial_size_, CV_64F);
	img_g_64.create(hyperspectral_frames.size(), hypercam_->spatial_size_, CV_64F);
	img_b_64.create(hyperspectral_frames.size(), hypercam_->spatial_size_, CV_64F);

	for (int i = 0; i < hyperspectral_frames.size(); i++) 
	{
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
	cv::moveWindow("color", 1330, 10);
	cv::waitKey(view_time_);
}

/*
assume robot arm already at start pose
vec3d: motion vector in base frame
cloud: PCL point cloud to save the data
*/
void VisionArmCombo::scanTranslateOnlyHyperspectral(double * vec3d, cv::Vec6d & start_scan_hand_pose, float acceleration, float speed, int option)
{

	if (robot_arm_client_ == NULL || hypercam_ == NULL)
	{
		std::cout << "robot arm or line profiler not initialized \n";
		return;
	}

	//start imaging
	hypercam_->start(option);


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

	robot_arm_client_->moveHandL(endPoseD, acceleration, speed, false);
	const double final_speed = robot_arm_client_->waitTillTCPMove(speed*0.99);

//	robot_arm_client_->moveHandL(endPoseD, acceleration, speed, true); hypercam_->stop(); return;

	unsigned int start_frame_count = hypercam_->frame_count_.load();

	robot_arm_client_->getCartesianInfo(sync_pose);
	robot_arm_client_->getTCPSpeed(tcp_sync_speed);
	std::cout << "syn_pose: " << sync_pose[0] << std::endl;

	robot_arm_client_->waitTillHandReachDstPose(endPoseD);

	unsigned int stop_frame_count = hypercam_->frame_count_.load();

	hypercam_->stop();

	for (int i = 0; i < 6; i++)
		start_scan_hand_pose[i] = sync_pose[i];

	int num_profiles = hypercam_->scanlines_.size();

	std::cout << "num frames: " << num_profiles << std::endl;

	std::cout << "final_speed " << final_speed << std::endl;

	std::cout << "start frame count: " << start_frame_count << " stop frame count: "<<stop_frame_count<<std::endl;

	// trim end
	hypercam_->scanlines_.erase(hypercam_->scanlines_.begin() + stop_frame_count - 1, hypercam_->scanlines_.end());

	// trim beginning
	hypercam_->scanlines_.erase(hypercam_->scanlines_.begin(), hypercam_->scanlines_.begin() + start_frame_count - 1);

	std::cout << "after erase num frames: " << hypercam_->scanlines_.size() << "\n";

	return; 

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
		
		std::memcpy(hypercam_->last_img_.ptr<unsigned char>(), hypercam_->frames_[i].data(), hypercam_->frame_data_size_);

		cv::Mat scan_line;
		
		cv::reduce(hypercam_->last_img_, scan_line, 0, CV_REDUCE_AVG, CV_64F);

		std::memcpy(img_64.ptr<double>(i), scan_line.ptr<double>(), sizeof(double)*hypercam_->spatial_size_);

		cv::Mat R_channel, G_channel, B_channel;

		hypercam_->last_img_.row(R_row).convertTo(R_channel, CV_64F);
		hypercam_->last_img_.row(G_row).convertTo(G_channel, CV_64F);
		hypercam_->last_img_.row(B_row).convertTo(B_channel, CV_64F);

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

	std::sprintf(currentTime, "%d_%d_%d_%d_%d_%d_%d", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);

	return std::string(currentTime);
}


// assume rover already in chamber
int VisionArmCombo::mapWorkspace(int rover_position, int option, int data_collection_mode)
{
	if (option != IMAGING && option != PROBING)
		return -1;

	if (data_collection_mode != TOP_VIEW && data_collection_mode != SIDE_VIEW)
		return -2;

	//if (option == IMAGING) controlChamber(cur_chamber_id_, RESUME_LIGHT);

#ifdef ENABLE_PP
	if (data_collection_mode == TOP_VIEW)
	{
		pp_.resetOccupancyGrid();
		if (rover_position != 1)
		{
			PointCloudT::Ptr object_cloud(new PointCloudT);

			Eigen::Vector3f min_side_panel, max_side_panel, min_side_window, max_side_window;

			float sign;

			if (rover_position == 0)
				sign = 1.0f;
			else if (rover_position == 2)
				sign = -1.0f;

			// window side 
			min_side_window << -0.3f - 0.06f, sign*1.2f - 0.3f - work_pos_offset_map_.at(rover_position), 0.45f - 0.8f;
			max_side_window << -0.3f + 0.06f, sign*1.2f + 0.3f - work_pos_offset_map_.at(rover_position), 0.45f + 0.8f;

			// chamber side panel
			min_side_panel << -1.f, sign*1.24f - 0.05f - work_pos_offset_map_.at(rover_position), 0.17f - 1.4f;
			max_side_panel << 0.f, sign*1.24f + 0.05f - work_pos_offset_map_.at(rover_position), 0.17f + 1.4f;

			// window side
			createBoxCloud(min_side_window, max_side_window, 0.01f, object_cloud);

			// chamber side wall
			createBoxCloud(min_side_panel, max_side_panel, 0.01f, object_cloud);

			pp_.addPointCloudToOccupancyGrid(object_cloud);
		}
		else
		{
			PointCloudT::Ptr object_cloud(new PointCloudT);

			Eigen::Vector3f min_side_panel, max_side_panel, min_side_window, max_side_window;

			// right window side 
			min_side_window << -0.3f - 0.06f, 0.98f, 0.45f - 0.8f;
			max_side_window << -0.3f + 0.06f, 0.98f + 0.2f, 0.45f + 0.8f;
			createBoxCloud(min_side_window, max_side_window, 0.01f, object_cloud);

			// left window side
			min_side_window << -0.3f - 0.06f, -0.98f - 0.2f, 0.45f - 0.8f;
			max_side_window << -0.3f + 0.06f, -0.98f, 0.45f + 0.8f;
			createBoxCloud(min_side_window, max_side_window, 0.01f, object_cloud);

			// right chamber side panel
			min_side_panel << -1.f, 1.24f, 0.17f - 1.4f;
			max_side_panel << 0.f, 1.24f + 0.1f, 0.17f + 1.4f;
			createBoxCloud(min_side_panel, max_side_panel, 0.01f, object_cloud);

			// left chamber side panel
			min_side_panel << -1.f, -1.24f - 0.1f, 0.17f - 1.4f;
			max_side_panel << 0.f, -1.24f, 0.17f + 1.4f;
			createBoxCloud(min_side_panel, max_side_panel, 0.01f, object_cloud);

			pp_.addPointCloudToOccupancyGrid(object_cloud);
		}

		loadOrUpdateChamberOccupancyData(rover_position, LOAD_OCCUPANCY);

		//showOccupancyGrid();
	}
#endif

	tof_cloud_->clear();

	// remap pot positions if remap_pot_position_ is 1 in the parameter file
	if (remap_pot_position_ && rover_position == 1 && rover_run_index_in_chamber_ == 0 && data_collection_mode == TOP_VIEW && option == IMAGING)
	{
		if (tof_cam_ != NULL) tof_cam_->setPower(3000);
		
		if (cur_chamber_config_.light_status == 1)
			controlChamber(cur_chamber_id_, PAUSE_LIGHT);

		PointCloudT::Ptr chamber_cloud(new PointCloudT);

		ArmConfig mapping_config;

		Eigen::Matrix4d tof_cam_pose;

		tof_cam_pose.col(0) << 0., -1., 0., 0.;
		tof_cam_pose.col(1) << -1., 0., 0., 0.;
		tof_cam_pose.col(2) << 0., 0., -1., 0.;
		tof_cam_pose.col(3) << -0.7, 0., 0.7, 1.;

		float step = 0.6f;

		for (float y = -step; y <= step*1.1f; y += step) {

			tof_cam_pose(1, 3) = y;

			Eigen::Matrix4d hand_pose = tof_cam_pose*hand_to_depth_.inverse();

			if (checkHandPoseReachable(hand_pose, mapping_config, ONLY_CHECK_DESTINATION_SELFCOLLISION))
			{
				moveToConfigGetPointCloud(mapping_config, GET_POINT_CLOUD | SKIP_PATH_PLAN);

				*chamber_cloud += *tof_cloud_;
			}
		}
		
		controlChamber(cur_chamber_id_, RESUME_LIGHT);

		if (chamber_cloud->size() > 100)
		{
			viewer_->addPointCloud(chamber_cloud, "chamber_cloud");
			mapPotPosition(chamber_cloud);
		}
		else
		{
			Utilities::to_log_file("map pot no point cloud");
			close();
		}
	}
	
	if (tof_cam_ != NULL)
	{
		if (data_collection_mode == TOP_VIEW)
			tof_cam_->setPower(300);
		else if (data_collection_mode == SIDE_VIEW)
			tof_cam_->setPower(1000);
	}

	// move arm high above plant
	ArmConfig ref_config;

	if (data_collection_mode == TOP_VIEW)
	{
		ref_config.setJointPos(-180., -87.3, -79.15, -103.4, 90., -180.);
		ref_config.toRad();
		robot_arm_client_->moveHandJ(ref_config.joint_pos_d, move_joint_speed_, move_joint_acceleration_);
	}
	else if (data_collection_mode == SIDE_VIEW)
	{
		//TODO not sure where to get white reference data in side view mode
	}
#if 0
	// only collect hyperspectral reference data in the center position
	if (rover_position == 1 && hypercam_ != NULL && data_collection_mode == TOP_VIEW && rover_run_index_in_chamber_ != 0)
	{
		if (option == IMAGING || leaf_tracing_hyperspectral_ == 1)
		{
			if(rover_run_index_in_chamber_ == 2)	//white reference of line light
				controlChamber(cur_chamber_id_, PAUSE_LIGHT);

			std::string path(data_saving_folder_.begin(), data_saving_folder_.end());

			getReferenceHyperspectralData(path, rover_position, option);

			if (rover_run_index_in_chamber_ == 2)
				controlChamber(cur_chamber_id_, RESUME_LIGHT);
		}
	}
#endif

	imagePots(rover_position, option, data_collection_mode);

	controlChamber(cur_chamber_id_, RESUME_LIGHT);

	return SUCCESS;
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

		std::string name = "cube" + std::to_string(j);
		viewer_->addCube(cube_coeff,name);
		//if(j < 12) 
		viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, name);

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

	//std::cout << "vox cloud size: " << cloud->points.size() << "\n";
	viewer_->removeAllCoordinateSystems();
	viewer_->addCoordinateSystem(0.3);
	viewer_->removePointCloud("grid");
	viewer_->addPointCloud(cloud, "grid");

	if(spin)
		display();
}

void VisionArmCombo::viewPlannedPath(float* start_pose, float* goal_pose, bool only_display_start)
{
	std::vector<PathPlanner::RefPoint> ref_points;
	std::vector<Eigen::Matrix3f> rot_mats;
	std::vector<PathPlanner::OBB> arm_obbs;

	std::cout << "start pose\n";

	// visualization
	pp_.computeReferencePointsOnArm(start_pose, ref_points, rot_mats);
	pp_.getArmOBBModel(ref_points, rot_mats, arm_obbs); addOBBArmModelToViewer(arm_obbs);

	if (!only_display_start)
	{
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
	}

	showOccupancyGrid();
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
	for (int i = 0; i < 6; i++) array6_d[i] = (double) array6_f[i];
}

void VisionArmCombo::double2float(double* array6_d, float* array6_f)
{
	for (int i = 0; i < 6; i++) array6_f[i] = (float) array6_d[i];
}

bool VisionArmCombo::moveToConfigGetPointCloud(ArmConfig & config, int options)
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

	// parse options, do not use "?:" operator to initialize boolean variable. does not work with msvc compiler so far
	bool smooth_path = !(options & DISABLE_SMOOTH_PATH);

	bool try_direct_path = !(options & DISABLE_DIRECT_PATH);

	bool view_path = options & VIEW_PATH;

	bool move = !(options & DISABLE_MOVE);

	bool get_cloud = options & GET_POINT_CLOUD;

	bool add_cloud_to_occupancy_grid = !(options & SKIP_ADD_OCCUPANCY_GRID);

	bool skip_path_plan = options & SKIP_PATH_PLAN;

#ifdef ENABLE_PP
	if (!skip_path_plan && pp_.collisionCheckForSingleConfig(config.joint_pos_f))
#else 
	if (pp_.selfCollision(config.joint_pos_f))
#endif
	{
		//std::cout << "target config collision found\n";
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

	//recordTime();

	if (start_to_goal_L2 > 1e-3)	//away from goal
	{
#ifdef ENABLE_PP
		if (!skip_path_plan)
		{
			if (!pp_.planPath(start_pose, config.joint_pos_f, smooth_path, try_direct_path))
			{
				//std::cout << "collsion-free path not found\n";
				if (view_path) viewPlannedPath(start_pose, config.joint_pos_f);
				return false;
			}

			//std::cout << "collsion-free path found\n";
			if (view_path) viewPlannedPath(start_pose, config.joint_pos_f);

			// this is for just checking if there is a path
			if (!move)
				return true;

			for (int i = 0; i < pp_.shortest_path_index_vec_.size(); i++)
			{
				float config_f[6];
				std::memcpy(config_f, pp_.random_nodes_buffer_ + pp_.shortest_path_index_vec_[i] * pp_.num_joints_, pp_.num_joints_*sizeof(float));

				double config_d[6];
				float2double(config_f, config_d);
				robot_arm_client_->moveHandJ(config_d, move_joint_speed_, move_joint_acceleration_, true);
			}
		}

#endif

		//std::cout << "joints: " << config.joint_pos_d[0] << ", " << config.joint_pos_d[1] << ", " << config.joint_pos_d[2] << ", "
			//<< config.joint_pos_d[3] << ", " << config.joint_pos_d[4] << ", " << config.joint_pos_d[5] << std::endl; std::getchar();
		robot_arm_client_->moveHandJ(config.joint_pos_d, move_joint_speed_, move_joint_acceleration_, true);
	}

	//printTime("pp time");

	if (get_cloud)
	{
		Sleep(1000);
		// get point cloud from tof camera
		PointCloudT::Ptr point_cloud(new PointCloudT);

		tof_cam_->getPointCloud(point_cloud);
		
	#if 0
		viewer_->removeAllPointClouds();
		viewer_->addPointCloud(point_cloud);
		display();
	#endif

		Eigen::Vector4f box_min(-1.2f, -2.f, -0.65f, 1.0f);
		Eigen::Vector4f box_max(-0.1f, 2.f, 1.28f, 1.0f);
		preprocessTOFCloud(point_cloud, box_min, box_max);

#ifdef ENABLE_PP
		if(add_cloud_to_occupancy_grid)	pp_.addPointCloudToOccupancyGrid(tof_cloud_);
#endif

	//	printTime("add to occupancy");

#if 0
		viewer_->removeAllPointClouds();

		viewer_->addPointCloud(tof_cloud_, "cloud", 0);

		viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

		display();
#endif
	}

	return true;
}

void VisionArmCombo::preprocessTOFCloud(PointCloudT::Ptr cloud, Eigen::Vector4f & box_min, Eigen::Vector4f & box_max)
{
	// get robot hand pose
	Eigen::Matrix4d hand_pose;

	getCurHandPoseD(hand_pose);

	Eigen::Matrix4d cam_pose = hand_pose*hand_to_depth_;

	PointCloudT::Ptr cloud_in_base(new PointCloudT);

	pcl::transformPointCloud(*cloud, *cloud_in_base, cam_pose);

	for (auto & p : cloud_in_base->points)
		p.r = p.g = p.b = (unsigned char)(std::max((p.z + 0.7f) / vis_z_range_ * 255.f, 0.f));

	pcl::CropBox<PointT> crop_box;
	crop_box.setInputCloud(cloud_in_base);
	crop_box.setMin(box_min);
	crop_box.setMax(box_max);
	std::vector<int> indices;
	crop_box.filter(indices);

	ror_.setInputCloud(cloud_in_base);
	ror_.setIndices(boost::make_shared<std::vector<int>>(indices));
	ror_.filter(*tof_cloud_);
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

#if 0
	std::string name = "line";
	viewer_->removeAllShapes();
	viewer_->addLine(line, name, 0); 
	display();
#endif

	// favor hand y pointing forward
	Eigen::Vector3d init_y_dir;
	Eigen::Matrix4d pose_to_hand;
	if (sensor_type == PROBE)
	{
		init_y_dir << 0.0, -1.0, 0.0;
		pose_to_hand = probe_to_hand_;
	}
	else if(sensor_type == SCANNER)
	{
		init_y_dir << 1.0, 0.0, 0.0;	// scanner
		pose_to_hand = scan_start_to_hand_;
	}

	// project to plane
	init_y_dir = init_y_dir - z_dir*(init_y_dir.dot(z_dir));

	init_y_dir.normalize();

	//std::cout << "init_y_dir " << init_y_dir.transpose() << "\n";

	double step_size = 20. / 180.*M_PI;

	solution_config_vec.clear();

	std::vector<double> y_value_vec;

	double min_x = 1.0f;

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

		Eigen::Vector3d x_dir = y_dir.cross(z_dir);

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
		for (int rot_angle_id = 0; rot_angle_id < 4; rot_angle_id++)
		{
			// robot hand pose
			Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

			pose.col(0).head<3>() = x_dir;
			pose.col(1).head<3>() = y_dir;
			pose.col(2).head<3>() = z_dir;
			pose.col(3).head<3>() << point.x, point.y, point.z;

			double rot_x_angle = 30.;

			if (rot_angle_id % 2 == 0 ) rot_x_angle = -30.;

			Eigen::Vector3d rot_axis = Eigen::Vector3d::UnitX();

			if (rot_angle_id > 1) rot_axis = Eigen::Vector3d::UnitY();

			pose.block<3,3>(0,0) = pose.block<3,3>(0,0) * Eigen::AngleAxisd(rot_x_angle / 180.*M_PI, rot_axis).matrix();

			pose = pose * pose_to_hand;

			std::vector<int> ik_sols_vec;

			pp_.inverseKinematics(pose, ik_sols_vec);

			//if (ik_sols_vec.size() == 0) std::cout << "no ik solution\n";

			for (auto idx : ik_sols_vec)
			{
				float sol_f[6];

				double2float(pp_.ik_sols_ + idx * num_joints_, sol_f);
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
				collision = pp_.collisionCheckForSingleConfig(sol_f);
				//ArmConfig config;
				//collision = checkHandPoseReachable(pose, config, CHECK_PATH);
#else
				collision = pp_.selfCollision(sol_f);
#endif
				if (!collision)
				{
					//std::cout << "no collision\n";		

					if (pose(0, 1) <= min_x)
					{
						optimal_hand_pose = pose;

						optimal_angle = angle;

						min_x = pose(0, 1);

						optimal_config.setJointPos(pp_.ik_sols_[idx*num_joints_], pp_.ik_sols_[idx*num_joints_ + 1],
							pp_.ik_sols_[idx*num_joints_ + 2], pp_.ik_sols_[idx*num_joints_ + 3],
							pp_.ik_sols_[idx*num_joints_ + 4], pp_.ik_sols_[idx*num_joints_ + 5]);

						solution_found = true;
					}
				}

				//else std::cout << "single config collision\n";
			}
		}
	}

	if (!solution_found)
	{
		//std::cout << "no collision free pose found\n";
		return solution_found;
	}

	std::vector<PathPlanner::RefPoint> ref_points;
	std::vector<Eigen::Matrix3f> rot_mats;
	std::vector<PathPlanner::OBB> arm_obbs;

	// visualization
#if 1
	pp_.computeReferencePointsOnArm(optimal_config.joint_pos_f, ref_points, rot_mats);
	pp_.getArmOBBModel(ref_points, rot_mats, arm_obbs); addOBBArmModelToViewer(arm_obbs);
#endif

	solution_config_vec.push_back(optimal_config);

	scan_start_or_probe_hand_pose = optimal_hand_pose;

	if (sensor_type == PROBE) return solution_found;
	
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

#if 0
	Eigen::Affine3f affine_pose;
	affine_pose.matrix() = pose.cast<float>();
	viewer_->removeAllCoordinateSystems();
	viewer_->addCoordinateSystem(0.2, affine_pose, "co", 0);
	display();
#endif

	std::vector<int> ik_sols_vec;

	pp_.inverseKinematics(pose, ik_sols_vec);

	solution_found = false;

	float config[6];

	for (auto idx : ik_sols_vec)
	{
		float sol_f[6];

		double2float(pp_.ik_sols_ + idx * num_joints_, sol_f);

#ifdef ENABLE_PP
		if (!pp_.collisionCheckForSingleConfig(sol_f))
#else
		if (!pp_.selfCollision(sol_f))
#endif
		{
			double2float(pp_.ik_sols_ + idx * num_joints_, config);

			solution_found = true;
		}
	}

	if (!solution_found)
	{
		solution_config_vec.clear();
		//std::cout << "scan end pose not found\n";
		return solution_found;
	}

	// visualization
	//std::cout << "scan end pose\n";
#if 0
	pp_.computeReferencePointsOnArm(config, ref_points, rot_mats);
	pp_.getArmOBBModel(ref_points, rot_mats, arm_obbs); addOBBArmModelToViewer(arm_obbs);
#endif

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


void VisionArmCombo::extractLeafProbingPointsAndHyperspectralScanPoses(PointCloudT::Ptr cloud_in,
												std::vector<pcl::PointIndices> & leaf_cluster_indices_vec, 
												std::vector<std::vector<Eigen::Matrix4d*>> & hyperscan_hand_pose_sequence_vec,
												std::vector<std::vector<ArmConfig>> & hyperscan_arm_config_sequence_vec,
												std::vector<int> & hyperscan_leaf_id_vec, int plant_id)
{
	leaf_cluster_indices_vec.clear();
	hyperscan_hand_pose_sequence_vec.clear();
	hyperscan_arm_config_sequence_vec.clear();
	hyperscan_leaf_id_vec.clear();

	leaf_probing_pn_vector_.clear();
	leaf_cluster_order_.clear();

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

	//std::cout << "Region growing time: " << (t1 - t0) / cv::getTickFrequency() << "\n";

	std::cout << "num clusters found: " << leaf_cluster_indices_vec.size() << "\n";

	if (leaf_cluster_indices_vec.size() == 0) return;

	PointCloudT::Ptr colored_cloud = reg.getColoredCloud();
	viewer_->removeAllPointClouds();
	viewer_->addPointCloud(colored_cloud, "region", 0); viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "region");
	display();
		
	for (int j = 0; j < leaf_cluster_indices_vec.size(); j++)
	{
		int cluster_size = leaf_cluster_indices_vec[j].indices.size();

		pcl::PointIndices::Ptr inliers(new pcl::PointIndices(leaf_cluster_indices_vec[j]));

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

		Eigen::Vector3f leaf_length_dir = major_vector;

		std::vector<pcl::Supervoxel<PointT>::Ptr> potential_probe_supervoxels;
		std::vector<pcl::PointXYZRGBNormal> probing_point_normal_vec;

		//extract imaging poses for hyperspectral camera
		Eigen::Matrix4f transform_to_origin = Eigen::Matrix4f::Identity();

		transform_to_origin.col(0).head(3) = major_vector;
		transform_to_origin.col(1).head(3) = middle_vector;
		transform_to_origin.col(2).head(3) = minor_vector;
		transform_to_origin.col(3).head(3) = mass_center;

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
		const float length = max_pt(0) - min_pt(0);

		pcl::PassThrough<PointT> pass;
		pass.setFilterFieldName("x");
		pass.setInputCloud(cloud_origin);

		std::vector<std::vector<int>> slices_indices;

		float step_size = 0.005f;
		float half_slice_width = hyperscan_slice_thickness_*0.5f;

		for (float x = min_pt(0) + half_slice_width; x <= length - half_slice_width; x += step_size) 
		{
			std::vector<int> indices;
			pass.setFilterLimits(x - half_slice_width, x + half_slice_width);
			pass.filter(indices);
			slices_indices.push_back(indices);
		}

		int cnt = 0;

		pcl::PCA<PointT> pca;
		pca.setInputCloud(cloud);

		std::string cloud_name = std::to_string(cv::getTickCount());
		//viewer_->addPointCloud(cloud, cloud_name);
		//viewer_->addCoordinateSystem(0.3);
		//display();

		std::vector<Eigen::Matrix4d*> valid_scan_hand_pose_sequence;
		std::vector<ArmConfig> valid_arm_config_sequence;
		std::vector<Eigen::Matrix4f*> scanner_poses;

		pcl::KdTreeFLANN<PointT> kdtree;
		kdtree.setInputCloud(cloud);

		for (auto & indices : slices_indices) 
		{
			if (indices.size() < 50)	//100 ~= 10 cm^2
				continue;

			uint32_t rgb;

			if (cnt++ % 2 == 0)
				rgb = (uint32_t)255 << 16;
			else
				rgb = (uint32_t)255 << 8;

			float rgb_f = *reinterpret_cast<float*>(&rgb);

			for (auto & id : indices)
				cloud->points[id].rgb = rgb_f;

			//viewer_->updatePointCloud(cloud, cloud_name); 
				
			//display();
				
			pca.setIndices(boost::make_shared<std::vector<int>>(indices));
			Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
			Eigen::Vector4f mean = pca.getMean();

			if (std::abs(eigenvectors(2,2)) < 0.1) // leaf almost vertical, not safe to scan
				continue;

			std::vector<int> pointIdxNKNSearch(1);
			std::vector<float> pointNKNSquaredDistance(1);
			PointT searchPoint;
			searchPoint.x = mean(0);
			searchPoint.y = mean(1);
			searchPoint.z = mean(2);

			if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
			{
				PointT p0 = cloud->points[pointIdxNKNSearch[0]];

				Eigen::Matrix4f *scanner_pose_ptr(new Eigen::Matrix4f);

				*scanner_pose_ptr = Eigen::Matrix4f::Identity();

				// save scan pose
				// z axis
				if (eigenvectors(2, 2) > 0.f)
					scanner_pose_ptr->col(2).head(3) = -1.0f*eigenvectors.col(2);
				else
					scanner_pose_ptr->col(2).head(3) = eigenvectors.col(2);

				Eigen::Vector3f cam_center = p0.getVector3fMap() - hyperspectral_imaging_dist_*scanner_pose_ptr->col(2).head(3);

				// determine y axis
				Eigen::Vector3f slice_dir = leaf_length_dir.cross(scanner_pose_ptr->col(2).head<3>());

				//align with base y axis direction/
				if (slice_dir(1) < 0.f)
					slice_dir *= -1.f;

				if (data_collection_mode_ == TOP_VIEW)
				{
					if (cam_center(1) > 0.f) // camera on right hand side
						//check slice_dir x
						if (slice_dir(0) >= 0.f)
							scanner_pose_ptr->col(0).head(3) = -slice_dir;
						else
							scanner_pose_ptr->col(0).head(3) = slice_dir;
					else // camera on left hand side
						if (slice_dir(0) > 0.f)
							scanner_pose_ptr->col(0).head(3) = slice_dir;
						else
							scanner_pose_ptr->col(0).head(3) = -slice_dir;
				}
				else if (data_collection_mode_ == SIDE_VIEW)
				{
					//check slice_dir x
					scanner_pose_ptr->col(0).head(3) = -slice_dir;
				}

				scanner_pose_ptr->col(1).head(3) = scanner_pose_ptr->col(2).head<3>().cross(scanner_pose_ptr->col(0).head<3>());

				scanner_pose_ptr->col(3).head(3) = p0.getVector3fMap() - hyperspectral_imaging_dist_*scanner_pose_ptr->col(2).head(3);

				bool leaf_tracing_scan_possible = false;

				Eigen::Matrix4d* hand_pose(new Eigen::Matrix4d);
				*hand_pose = scanner_pose_ptr->cast<double>()*hand_to_hyperspectral_d_.inverse();

				std::vector<int> ik_sols_vec;

				pp_.inverseKinematics(*hand_pose, ik_sols_vec, data_collection_mode_);

				//if (ik_sols_vec.size() == 0) std::cout << "no ik solution\n";

				for (auto idx : ik_sols_vec)
				{
					float sol_f[6];

					double2float(pp_.ik_sols_ + idx * num_joints_, sol_f);

#ifdef ENABLE_PP
					if ((data_collection_mode_ == TOP_VIEW && !pp_.collisionCheckForSingleConfig(sol_f))
						|| ((data_collection_mode_ == SIDE_VIEW) && !pp_.selfCollision(sol_f))
						)
#else
					if (!pp_.selfCollision(sol_f))
#endif
					{
						//std::cout << "collision: " << pp_.selfCollision(sol_f) << std::endl;
						ArmConfig config;
						config.setJointPos(sol_f[0], sol_f[1], sol_f[2], sol_f[3], sol_f[4], sol_f[5]);
						leaf_tracing_scan_possible = true;

						if (valid_arm_config_sequence.size() > 0)
						{
							if (!pp_.selfCollisionBetweenTwoConfigs(valid_arm_config_sequence.back().joint_pos_f, sol_f))
							{
								valid_scan_hand_pose_sequence.push_back(hand_pose);
								valid_arm_config_sequence.push_back(config);
							}
						}

						//Eigen::Affine3f affine_pose;
						//affine_pose.matrix() = *scanner_pose_ptr;
						//viewer_->addCoordinateSystem(hyperspectral_imaging_dist_, affine_pose, std::to_string(cv::getTickCount()));

#if 0
						// visualization
						std::vector<PathPlanner::RefPoint> ref_points;
						std::vector<Eigen::Matrix3f> rot_mats;
						std::vector<PathPlanner::OBB> arm_obbs;
						pp_.computeReferencePointsOnArm(config.joint_pos_f, ref_points, rot_mats);
						pp_.getArmOBBModel(ref_points, rot_mats, arm_obbs); addOBBArmModelToViewer(arm_obbs);
						//display();
#endif
						break;
					}
					//else std::cout << "self collision\n";
				}
				
				// separate slice into two clusters
				std::vector<std::vector<int>> cluster_indices_vec(2);

				for (auto idx : indices)
					if ((cloud->points[idx].getVector3fMap()-mean.head(3)).dot(scanner_pose_ptr->col(0).head(3)) > 0.f)
						cluster_indices_vec[0].push_back(idx);
					else
						cluster_indices_vec[1].push_back(idx);

				for (auto & cluster_indices : cluster_indices_vec)
				{
					if (cluster_indices.size() < 20) continue;

					pcl::PointXYZRGBNormal probe_point;

					probe_point.r = probe_point.g = probe_point.b = 0;

					// favor probing points that can also be scanned by hyperspectral 
					if (leaf_tracing_scan_possible)
						probe_point.r = 1;

					pca.setIndices(boost::make_shared<std::vector<int>>(cluster_indices));

					probe_point.getVector3fMap() = pca.getMean().head(3);

					probe_point.getNormalVector3fMap() = pca.getEigenVectors().col(2);

					Eigen::Vector3f eigenvalues = pca.getEigenValues();

					probe_point.curvature = eigenvalues(0) / (eigenvalues.sum());

					probing_point_normal_vec.push_back(probe_point);
				}
			}
		}

		if (valid_scan_hand_pose_sequence.size() != 0) 
		{
			hyperscan_hand_pose_sequence_vec.push_back(valid_scan_hand_pose_sequence);
			hyperscan_arm_config_sequence_vec.push_back(valid_arm_config_sequence);
			hyperscan_leaf_id_vec.push_back(j);
		}
		
		LeafIDnX leaf_id_x;
		leaf_id_x.id = j;
		leaf_id_x.x = mass_center(0);
		leaf_cluster_order_.push_back(leaf_id_x);
		
		//visualize probing point normal
	//	viewer_->removeAllCoordinateSystems();	viewer_->addCoordinateSystem(0.3);
#if 0
		for (auto & pn0 : probing_point_normal_vec)
		{
			pcl::ModelCoefficients line; line.values.resize(6);
			line.values[0] = pn0.x;
			line.values[1] = pn0.y;
			line.values[2] = pn0.z;
			float sign = pn0.normal_z >= 0.f ? 1.0f : -1.0f;
			line.values[3] = pn0.normal_x*0.4f*sign;
			line.values[4] = pn0.normal_y*0.4f*sign;
			line.values[5] = pn0.normal_z*0.4f*sign;

			viewer_->addLine(line, "l" + std::to_string(cv::getTickCount()));

			pcl::ModelCoefficients sphere; sphere.values.resize(4);
			sphere.values[0] = pn0.x; sphere.values[1] = pn0.y; sphere.values[2] = pn0.z;
			sphere.values[3] = 0.002;

			viewer_->addSphere(sphere, "sphere"+std::to_string(cv::getTickCount()));
		}

		display();
#endif

		viewer_->removeAllShapes();

		leaf_probing_pn_vector_.push_back(probing_point_normal_vec);
	}

	std::sort(leaf_cluster_order_.begin(), leaf_cluster_order_.end(), VisionArmCombo::leaf_x_comparator);

	for (int leaf_idx = 0; leaf_idx < leaf_probing_pn_vector_.size(); leaf_idx++)
		std::sort(leaf_probing_pn_vector_[leaf_idx].begin(), leaf_probing_pn_vector_[leaf_idx].end(), VisionArmCombo::probing_rank_comparator);
}

int VisionArmCombo::saveProbingData(PointT & probe_point, pcl::Normal & normal, int probe_id, int plant_id)
{
	std::string time = getCurrentDateTimeStr();

	cv::Vec3d probing_point_cv(probe_point.x, probe_point.y, probe_point.z);

	cv::Vec3d probing_point_normal_cv(normal.normal_x, normal.normal_y, normal.normal_z);

	std::string save_path(data_saving_folder_.begin(), data_saving_folder_.end());

	if (probe_id == PAM) 
	{

#if 0
		float Fo, Fm, FvFm, qP, qL, qN, NPQ, YNPQ, YNO, F, FmPrime, PAR, YII, ETR, FoPrime;

		if (MiniPamActPlusYield(Fo, Fm, FvFm, qP, qL, qN, NPQ, YNPQ, YNO, F, FmPrime, PAR, YII, ETR, FoPrime) == 0)
		{
			std::string pam_file = +"PAM_plant_" + std::to_string(plant_id) + "_time_" + time + ".yml";

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
			//bad measurement, possibly probed the wrong place
			//if (YII < 0.1f)	return -1;

			std::string pam_file = save_path + "PAM_" + std::to_string(plant_id) + "_" + time + ".yml";

			cv::FileStorage fs(pam_file, cv::FileStorage::WRITE);

			fs << "probing_point" << probing_point_cv;
			fs << "probing_normal" << probing_point_normal_cv;
			fs << "YII" << YII;
			//fs << "PAR" << PAR;

			fs.release();
		}
	}
#if 0
	else if (probe_id == RAMAN_1064 || probe_id == RAMAN_532) 
	{
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

		if (status == 0) 
		{

			std::string raman_csv = save_path + "raman" + wavelength_str + "_" + std::to_string(plant_id) + "_" + time + ".csv";

			raman_->saveLastSpectrum(raman_csv);

			std::string raman_pose_path = save_path + "raman" + wavelength_str + "_" + std::to_string(plant_id) + "_" + time + ".yml";

			cv::FileStorage fs(raman_pose_path, cv::FileStorage::WRITE);

			fs << "probing_point" << probing_point_cv;

			fs << "probing_normal" << probing_point_normal_cv;

			fs.release();
		}
	}
#endif
	return 0;
}

bool VisionArmCombo::probeLeaf(PointT & probe_point, pcl::Normal & normal, int probe_id, int plant_id)
{
	std::vector<ArmConfig> solution_config_vec_refine;
	Eigen::Matrix4d probe_hand_pose_final;
	Eigen::Vector3d hand_translation_refine;

	if (probe_id == PAM)
		probe_to_hand_ = probe_to_hand_pam_;
	else if (probe_id == RAMAN_532)
		probe_to_hand_ = probe_to_hand_raman_532_;
	else if (probe_id == RAMAN_1064)
		probe_to_hand_ = probe_to_hand_raman_1064_;

	// try probe
	if (computeCollisionFreeProbeOrScanPose(probe_point, normal, PROBE, solution_config_vec_refine,
											probe_hand_pose_final, hand_translation_refine))
	{
#if 0
		// visualization
		std::cout << "showing prepare probe pose\n";
		Eigen::Affine3f affine_pose;
		affine_pose.matrix() = probe_hand_pose_final.cast<float>();
		viewer_->removeAllCoordinateSystems();
		viewer_->addCoordinateSystem(0.2, affine_pose, "prepare probe pose", 0);
		display();
#endif

		// ik
		std::vector<int> ik_sols_vec;

		pp_.inverseKinematics(probe_hand_pose_final, ik_sols_vec);

		bool solution_found = false;

		float config[6];

		ArmConfig prepare_probe_config;

		for (auto idx : ik_sols_vec)
		{
			float sol_f[6];

			double2float(pp_.ik_sols_ + idx * num_joints_, sol_f);

#ifdef ENABLE_PP
			if (!pp_.collisionCheckForSingleConfig(sol_f))
#else
			if (!pp_.selfCollision(sol_f))
#endif
			{
				double2float(pp_.ik_sols_ + idx * num_joints_, config);

				prepare_probe_config.setJointPos(pp_.ik_sols_[idx*num_joints_], pp_.ik_sols_[idx*num_joints_ + 1],
												pp_.ik_sols_[idx*num_joints_ + 2], pp_.ik_sols_[idx*num_joints_ + 3],
												pp_.ik_sols_[idx*num_joints_ + 4], pp_.ik_sols_[idx*num_joints_ + 5]);

				solution_found = true;
			}
		}

		//std::cout << "try to move to prepare probe pose\n";

		if (!solution_found)
		{
			//std::cout << "probe_hand_pose_final ik solution not found\n";
			return false;
		}

		ArmConfig target_config;
		if (!checkHandPoseReachable(probe_hand_pose_final, target_config, CHECK_PATH))
		{
			return false;
		}

		if (!moveToConfigGetPointCloud(prepare_probe_config))
		{
			//std::cout << "could not reach prepare probe pose\n";
			return false;
		}

		double pose[6];
		
		eigenMat4dToArray6(probe_hand_pose_final, pose);

		robot_arm_client_->moveHandL(pose, 0.04, 0.04, false);

		// extend probe
		robot_arm_client_->probeCylinderControl(true);

		bool success = saveProbingData(probe_point, normal, probe_id, plant_id) == SUCCESS;

		robot_arm_client_->probeCylinderControl(false);

		//display();

		return success;
	}

	return false;
}

void VisionArmCombo::display()
{
	if (view_time_ == 0) viewer_->spin();
	else viewer_->spinOnce(view_time_);
}

void VisionArmCombo::calibrateRGBCamera(int nframes)
{
	if (robot_arm_client_ == NULL || rgb_cam_ == NULL) {

		std::cerr<<"Not connected to UR10 or RGB camera\n";
		return;
	}

	if (nframes % 2 != 0 || nframes < 6)
	{
		std::cout << "number of frames not even or not enough\n";
		return;
	}

	cv::Mat cameraMatrix, distCoeffs;
	std::vector<cv::Mat> image_vec;
	std::vector<Eigen::Matrix4d*> tcp_pose_vec; tcp_pose_vec.resize(nframes);
	cv::Size boardSize, imageSize;
	float squareSize;

	std::vector<std::vector<cv::Point2f>> imagePoints;

	// IMPORTANT
	cv::SimpleBlobDetector::Params params;
	params.maxArea = 400 * 400;
	params.minArea = 20 * 20;
	cv::Ptr<cv::FeatureDetector> blobDetect = cv::SimpleBlobDetector::create(params);

	imageSize.width = rgb_cam_->img_width_;
	imageSize.height = rgb_cam_->img_height_;

	boardSize.width = 4;
	boardSize.height = 11;

	squareSize = 0.02f;

	for (int i = 0; i < nframes; i++)
	{
		cv::Mat view, viewGray;

		std::vector<cv::Point2f> pointbuf;

		while (true)
		{
			view = rgb_cam_->getRGB();

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

			cv::resize(view_copy, shrinked, cv::Size(), 0.4, 0.4);

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

	cv::FileStorage fs("RGBCalibration.yml", cv::FileStorage::WRITE);

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
	float squareSize;

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

// IR of odos time-of-flight camera
void VisionArmCombo::calibrateIRCamera(int nframes)
{
	if (tof_cam_ == NULL || robot_arm_client_ == NULL)
	{
		std::cerr << "TOF camera or UR10 not connected\n";
		return;
	}

	if (nframes % 2 != 0 || nframes < 6)
	{
		std::cout << "number of frames not even or not enough\n";
		return;
	}

	cv::Mat cameraMatrix, distCoeffs;
	std::vector<cv::Mat> image_vec;
	std::vector<Eigen::Matrix4d*> tcp_pose_vec; tcp_pose_vec.resize(nframes);
	cv::Size boardSize, imageSize;
	float squareSize;

	std::vector<std::vector<cv::Point2f>> imagePoints;

	// IMPORTANT
	cv::SimpleBlobDetector::Params params;
	params.maxArea = 50 * 50;
	params.minArea = 5 * 5;
	cv::Ptr<cv::FeatureDetector> blobDetect = cv::SimpleBlobDetector::create(params);

	imageSize.width = tof_cam_->img_width_;
	imageSize.height = tof_cam_->img_height_;

	boardSize.width = 4;
	boardSize.height = 11;

	squareSize = 0.02f;

	for (int i = 0; i < nframes; i++)
	{
		cv::Mat view, viewGray;

		std::vector<cv::Point2f> pointbuf;

		while (true)
		{
			view = tof_cam_->getIR();

			//cv::normalize(view, viewGray, 0, 255, cv::NORM_MINMAX, CV_8UC1);

			pointbuf.clear();

			// ASYMMETRIC CIRCLE GRID PATTERN
			bool found = findCirclesGrid(view, boardSize, pointbuf, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetect);

			cv::Mat view_copy;

			view.copyTo(view_copy);

			cv::cvtColor(view, view_copy, CV_GRAY2BGR);

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

	cv::FileStorage fs("IRCalibration.yml", cv::FileStorage::WRITE);

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
void VisionArmCombo::RGBHandEyeCalibration()
{
	cv::FileStorage fs("RGBCalibration.yml", cv::FileStorage::READ);
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

	Eigen::Matrix4d hand_to_eye;

	solveHandEyeCalibration(camera_pose_vec, tcp_pose_vec, hand_to_eye);

	cv::Mat hand_to_eye_cv;

	EigenTransformToCVTransform(hand_to_eye, hand_to_eye_cv);

	cv::FileStorage fs1("RGBHandEyeCalibration.yml", cv::FileStorage::WRITE);

	fs1 << "hand to eye" << hand_to_eye_cv;

	fs1.release();
}

void VisionArmCombo::IRHandEyeCalibration()
{
	cv::FileStorage fs("IRCalibration.yml", cv::FileStorage::READ);
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

	cv::FileStorage fs1("IRHandEyeCalibration.yml", cv::FileStorage::WRITE);

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

int VisionArmCombo::markerDetection(int rgb_or_ir, float & nearest_marker_dist, int & marker_id, float max_dist_thresh, bool search_zero)
{
	if (rgb_or_ir != BASLER_RGB && rgb_or_ir != IR) {

		std::cout << "wrong camera type\n";
		return -1;
	}

	cv::Mat img_rgb, img_gray, img_tmp;

	//while (true)//for debug
	{

		if (rgb_or_ir == BASLER_RGB) {
			
			img_tmp = rgb_cam_->getRGB();

			cv::resize(img_tmp, img_rgb, cv::Size(), 0.5, 0.5);
		}
		else {

			img_tmp = tof_cam_->getIR();

			cv::cvtColor(img_tmp, img_rgb, CV_GRAY2BGR);
		}

		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
		cv::aruco::detectMarkers(img_rgb, marker_dictionary_, markerCorners, markerIds, detector_params_, rejectedCandidates);

		std::vector<cv::Vec3d> rvecs, tvecs;

		if(rgb_or_ir == BASLER_RGB)
			cv::aruco::estimatePoseSingleMarkers(markerCorners, marker_length_, rgb_camera_matrix_cv_, rgb_dist_coeffs_cv_, rvecs, tvecs);
		else
			cv::aruco::estimatePoseSingleMarkers(markerCorners, marker_length_, infrared_camera_matrix_cv_, infrared_dist_coeffs_cv_, rvecs, tvecs);


		cv::aruco::drawDetectedMarkers(img_rgb, markerCorners, markerIds);

		// find the closest marker less than distance threshold
		float min_dist = std::numeric_limits<float>().max();
		int nearest_marker_idx = -1;

		if (!search_zero) {
			for (unsigned int i = 0; i < markerIds.size(); i++)
			{
				float dist = std::sqrt(tvecs[i].val[0] * tvecs[i].val[0] + tvecs[i].val[1] * tvecs[i].val[1] + tvecs[i].val[2] * tvecs[i].val[2]);

				if (dist < min_dist && dist < max_dist_thresh) {

					nearest_marker_idx = i;
					min_dist = dist;
				}

			}
		}
		else
		{	
			for (unsigned int i = 0; i < markerIds.size(); i++)
			{
				if (markerIds[i] == 0) {

					nearest_marker_idx = i;
					min_dist = std::sqrt(tvecs[i].val[0] * tvecs[i].val[0] + tvecs[i].val[1] * tvecs[i].val[1] + tvecs[i].val[2] * tvecs[i].val[2]);
					break;
				}
			}

		}

		if (nearest_marker_idx >= 0) {

			if (rgb_or_ir == BASLER_RGB)
				cv::aruco::drawAxis(img_rgb, rgb_camera_matrix_cv_, rgb_dist_coeffs_cv_, rvecs[nearest_marker_idx], tvecs[nearest_marker_idx], marker_length_*0.5f);
			else
				cv::aruco::drawAxis(img_rgb, infrared_camera_matrix_cv_, infrared_dist_coeffs_cv_, rvecs[nearest_marker_idx], tvecs[nearest_marker_idx], marker_length_*0.5f);
			//std::cout << "id " << i << " rot " << rvecs[i] << " tran " << tvecs[i] << "\n";
			
			nearest_marker_dist = min_dist;
			marker_id = markerIds[nearest_marker_idx];

			//std::cout << "dist: " << min_dist << "\n";
		}
		else {
			nearest_marker_dist = -1.f;
			marker_id = -1;
		}

		//cv::imshow("marker", img_rgb);	int key = cv::waitKey(10);
	}

	return 0;

	/*	//generate marker images and save
	for (int i = 0; i < 50; i++)
	{
	cv::aruco::drawMarker(marker_dictionary_, i, 700, markerImage, 1);

	cv::imshow("marker", markerImage);

	cv::imwrite("Markers\\marker_" + std::to_string(i) + ".png", markerImage);

	cv::waitKey(100);
	}*/

#if 0	// how to use opencv marker pose 
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
#endif
}

void VisionArmCombo::cvTransformToEigenTransform(cv::Mat & cv_transform, Eigen::Matrix4d & eigen_transform)
{
	for (int y = 0; y < 4; y++)
		for (int x = 0; x < 4; x++)
			eigen_transform(y, x) = cv_transform.at<double>(y, x);
}

void VisionArmCombo::EigenTransformToCVTransform( Eigen::Matrix4d & eigen_transform, cv::Mat & cv_transform)
{
	cv_transform.create(4, 4, CV_64F);
	for (int y = 0; y < 4; y++)
		for (int x = 0; x < 4; x++)
			cv_transform.at<double>(y, x) = eigen_transform(y, x);
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

	std::string laser_cloud_name;

	// compute laser scan pose
	if (computeCollisionFreeProbeOrScanPose(point, normal, SCANNER, solution_config_vec, scan_start_hand_pose, hand_translation))
	{
		//std::cout << "hand translation: " << hand_translation.transpose() << "\n";

		// move to the start scan pose
		if (moveToConfigGetPointCloud(solution_config_vec[0]))
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
		if (moveToConfigGetPointCloud(solution_config_vec[0]))
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

// assume rover is at the chamber center facing the plants
int VisionArmCombo::doOneDataAcquisitionRunInChamber(int last_run)
{
	if (rover_pos_in_chamber_ == 2) goto CHMABER_LEFT_POSITION;

	if (rover_pos_in_chamber_ == 0) goto CHMABER_RIGHT_POSITION;
		
	std::cout << "move arm in\n";
	moveArmInOrOutChamber(MOVE_ARM_IN_CHAMBER, data_collection_mode_);

	//collect data at the center position
	mapWorkspace(1, imaging_or_probing_, data_collection_mode_);

	const bool only_work_at_center = (!work_at_pos_0_needed_) && (!work_at_pos_2_needed_);

	if (only_work_at_center) goto FINISH;

	std::cout << "move arm out\n";
	moveArmInOrOutChamber(MOVE_ARM_OUT_CHAMBER, data_collection_mode_);

	if (multi_work_position_)
	{
		if (data_collection_mode_ == TOP_VIEW)
		{
			if (mir_.reverseTopView() != 0) close();
		}
		else
		{
			if (mir_.reverseSideView() != 0) close();
		}

		//go to right position
		if (cur_chamber_id_ == 2)
		{
			if (mir_.stepBackward25CMInChamber() != 0) close();
		}

		if(mir_.goToChamebrVLMarker(cur_chamber_id_)!=0) close();

		if (data_collection_mode_ == TOP_VIEW)
		{
			if (mir_.moveToTopView() != 0) close();
		}
		else
		{
			if (mir_.moveToSideView() != 0) close();
		}

		rover_pos_in_chamber_ = 0; saveTaskProgressToFile();

	CHMABER_RIGHT_POSITION:
		moveArmInOrOutChamber(MOVE_ARM_IN_CHAMBER, data_collection_mode_);

		mapWorkspace(0, imaging_or_probing_, data_collection_mode_);

		moveArmInOrOutChamber(MOVE_ARM_OUT_CHAMBER, data_collection_mode_);

		if (data_collection_mode_ == TOP_VIEW)
		{
			if (mir_.reverseTopView() != 0) close();
		}
		else
		{
			if (mir_.reverseSideView() != 0) close();
		}

		// go to left position
		if(mir_.stepForward1MInChamber() != 0) close();

		if (data_collection_mode_ == TOP_VIEW)
		{
			if (mir_.moveToTopView() != 0) close();
		}
		else
		{
			if (mir_.moveToSideView() != 0) close();
		}

		rover_pos_in_chamber_ = 2; saveTaskProgressToFile();

	CHMABER_LEFT_POSITION:
		moveArmInOrOutChamber(MOVE_ARM_IN_CHAMBER, data_collection_mode_);

		mapWorkspace(2, imaging_or_probing_, data_collection_mode_);

		moveArmInOrOutChamber(MOVE_ARM_OUT_CHAMBER, data_collection_mode_);

	FINISH:
		if (last_run == 0)	//not last run
		{
			if (only_work_at_center) return SUCCESS;

			if (data_collection_mode_ == TOP_VIEW)
			{
				if (mir_.reverseTopView() != 0) close();
			}
			else
			{
				if (mir_.reverseSideView() != 0) close();
			}

			if(mir_.stepBackward75CMInChamber() != 0) close();

			if(mir_.goToChamebrVLMarker(cur_chamber_id_) != 0) close();

			if(mir_.stepForwardInChamber() != 0) close();

			if (data_collection_mode_ == TOP_VIEW)
			{
				if (mir_.moveToTopView() != 0) close();
			}
			else
			{
				if (mir_.moveToSideView() != 0) close();
			}

			rover_pos_in_chamber_ = 1; saveTaskProgressToFile();
		}
		else // last run
		{
			if (only_work_at_center)
			{
				std::cout << "move arm out\n";
				moveArmInOrOutChamber(MOVE_ARM_OUT_CHAMBER, data_collection_mode_);

				if (data_collection_mode_ == TOP_VIEW)
				{
					if (mir_.moveToTopView() != 0) close();	
				}
				else
				{
					if (mir_.moveToSideView() != 0) close();
				}

				if (mir_.stepBackwardInChamber() != 0) close();

				if (mir_.moveToSideView() != 0) close();

				return SUCCESS;
			}

			if (data_collection_mode_ == SIDE_VIEW)
			{
				if (mir_.turn180() != 0) close();	//make mir head facing door
			}
		}
	}
	return SUCCESS;
}

// assumes rover is not inside any chamber
int VisionArmCombo::sendRoverToChamber(int target_chamber_id)
{
	if (target_chamber_id < 0 || target_chamber_id > 8) 
	{
		std::cout << "invalid chamber id\n";
		return -1;
	}

	getChamberConfig(target_chamber_id, cur_chamber_config_);

	if (only_do_probing_ != 1 && inside_or_outside_chamber_ == "outside")
	{
		rover_run_index_in_chamber_ = 0; rover_pos_in_chamber_ = 1; plant_imaging_stage_ = 0;
		imaging_or_probing_ = 0; saveTaskProgressToFile();

		moveArmToHome();
		// dont need to go to door front first. longer planning distances work better
		controlChamber(target_chamber_id, OPEN_DOOR);

		if (enable_tof_)
		{
			tof_cam_->start();
			tof_cam_->setPower(256);
		}

		if (mir_.goToDoor(target_chamber_id) != 0) close();

		//if the chamber door did not open, skip to next chamber
		if (mir_.goToChamberPosition(target_chamber_id, 2) != 0)
		{
			Utilities::to_log_file("chamber door did not open");
			mir_.clearError();
			mir_.continueMissionQueue();
			//wait 1 min and let the scheduler try again
			Sleep(1000*60);
			return -1;
		}

		if(mir_.goToChamberPosition(target_chamber_id, 1) != 0) close();

		// go marker first before closing door because the mir robot is less likely to fail to dock with the door open
		if(mir_.goToChamebrVLMarker(target_chamber_id) != 0) close();

		
		pot_processed_map_.create(pot_position_vec_[target_chamber_id - 1].rows, pot_position_vec_[target_chamber_id - 1].cols, CV_8U);
		pot_processed_map_ = 0;
		pot_sensor_map_ = cv::Mat::ones(pot_processed_map_.size(), CV_8U);

		rover_run_index_in_chamber_ = 0;

		std::string folder_name = "c" + std::to_string(target_chamber_id) + "_" + getCurrentDateTimeStr();

		std::ofstream file;
		file.open("data_folders_to_upload.txt", std::ios::app);

		file << folder_name << "," << experiment_id_ << std::endl;
		file.close();

		folder_name = "Enviratron_Data\\chamber_" + std::to_string(target_chamber_id) + "\\" + folder_name + "\\";

		data_folder_ = folder_name; saveTaskProgressToFile();

		std::wstring folder_name_w(folder_name.begin(), folder_name.end());

		data_saving_folder_ = folder_name_w;

		std::cout << CreateDirectory(data_saving_folder_.c_str(), NULL);

		if (close_door_when_rover_inside_ == 1) controlChamber(target_chamber_id, CLOSE_DOOR);

		if(mir_.stepForwardInChamber() != 0) close();

		if (data_collection_mode_ == TOP_VIEW)
		{
			if (mir_.moveToTopView() != 0) close();
		}
		else
		{
			if (mir_.moveToSideView() != 0) close();
		}

		// only collect hyperspectral reference data in the center position
		std::string data_folder(data_saving_folder_.begin(), data_saving_folder_.end());
		//getReferenceHyperspectralData(data_folder, rover_pos_in_chamber_, PROBING);	//depreciated, no leaf tracing hyperspectral imaging any more

		controlChamber(target_chamber_id, OPEN_CURTAIN);
		
		checkCurtainOpenOrClosed(target_chamber_id, OPEN_CURTAIN, data_collection_mode_);

		if(cur_chamber_config_.light_status == 1)
			getReferenceHyperspectralData(data_folder, rover_pos_in_chamber_, IMAGING);

		// if rover outside, reset the other state variables
		cur_chamber_id_ = target_chamber_id;
		pot_processed_map_ = 0;
		data_collection_done_ = 1; 
		inside_or_outside_chamber_ = "inside"; saveTaskProgressToFile();
	}
	else // check if there is remaining work in chamber due to program crash
	{
		std::wstring data_saving_folder_w(data_folder_.begin(), data_folder_.end());
		data_saving_folder_ = data_saving_folder_w;
		if (enable_tof_) tof_cam_->start();
	}
	
	// assuming robot is in topview position or side view position now
	if(enable_imaging_)
	{
		data_collection_done_ = 0;
		pot_sensor_map_ = cur_chamber_config_.pot_sensor_map.clone();
		saveTaskProgressToFile();

		// check if rover needs to work at left and right positions
		cv::Mat pot_pos = pot_position_vec_[cur_chamber_id_ - 1];
		work_at_pos_2_needed_ = false;
		work_at_pos_0_needed_ = false;
		std::cout << "pot_sensor_map_:\n" << pot_sensor_map_ << std::endl;
		for (int y = 0; y < pot_pos.rows; y++)
			for (int x = 0; x < pot_pos.cols; x++)
			{
				if (pot_sensor_map_.at<uchar>(y, x) == 0) continue;

				cv::Vec3f pot_xyz = pot_pos.at<cv::Vec3f>(y, x);

				if (pot_xyz[1] < -topview_y_access_limit_)
					work_at_pos_2_needed_ = true;

				if (pot_xyz[1] > topview_y_access_limit_)
					work_at_pos_0_needed_ = true;
			}

		std::cout << "work_at_pos_2_needed_: " << work_at_pos_2_needed_ << "  work_at_pos_0_needed_: " << work_at_pos_0_needed_ << std::endl;

		if (pot_sensor_map_.rows != pot_pos.rows || pot_sensor_map_.cols != pot_pos.cols)
		{
			Utilities::to_log_file("pot_sensor_map and pot_pos mismatch");
			close();
		}

		for (; rover_run_index_in_chamber_ < 3; rover_run_index_in_chamber_++)
		{
			saveTaskProgressToFile();

			if (rover_run_index_in_chamber_ == 0)
			{
				// depth, thermal, and RGB
				doOneDataAcquisitionRunInChamber();
			}
			else if (rover_run_index_in_chamber_ == 1)
			{
				//hyperspectral imaging
				plant_imaging_stage_ = 3; saveTaskProgressToFile();
				// skip if during night
				if (cur_chamber_config_.light_status == 1)
					doOneDataAcquisitionRunInChamber();
			}
			else if (rover_run_index_in_chamber_ == 2)
			{
				// laser scan and probing
				imaging_or_probing_ = 1; saveTaskProgressToFile(); //probing
				const int last_run = 1;
				doOneDataAcquisitionRunInChamber(last_run);
			}
		}

		data_collection_done_ = 1; saveTaskProgressToFile();
	}
	
	if (only_do_probing_ != 1)	//exit chamber
	{
		controlChamber(target_chamber_id, CLOSE_CURTAIN);

		if (enable_tof_) tof_cam_->stop();

		Sleep(12000);

		controlChamber(target_chamber_id, OPEN_DOOR);

		if(mir_.goToDoor(target_chamber_id) != 0) close();

		controlChamber(target_chamber_id, CLOSE_DOOR);

		inside_or_outside_chamber_ = "outside"; saveTaskProgressToFile();
	}

	return 0;
}

void VisionArmCombo::run()
{		
	std::vector<std::string> time_str;
	boost::split(time_str, start_time_, boost::is_any_of(":"));

	if (time_str.size() != 2)
	{
		Utilities::to_log_file("start time error");
		close();
	}

	int start_hour = std::stoi(time_str[0]);

	if (start_hour < 0 || start_hour > 23)
	{
		Utilities::to_log_file("start time hour wrong");
		close();
	}

	int start_min = std::stoi(time_str[1]);

	if (start_min < 0 || start_min > 60)
	{
		Utilities::to_log_file("start time minute wrong");
		close();
	}

	const double one_round_work_time = scheduled_minutes_per_chamber_*8.;
	const double one_round_workk_plus_charging_time = one_round_work_time*(1. + charing_time_2_work_time_ratio_);
	const int max_num_runs_per_day = (int)(1440. / one_round_workk_plus_charging_time);
	const double total_one_round_time = 1440. / max_num_runs_per_day;
	const double extra_minutes_per_run = total_one_round_time - one_round_work_time*(1. + charing_time_2_work_time_ratio_);

	time_t now;
	struct tm daily_rover_start_tm, end_of_day_tm;

	int target_chamber_id = cur_chamber_id_;

	if (inside_or_outside_chamber_ == "inside")
	{
		sendRoverToChamber(target_chamber_id);
		if (target_chamber_id == 8)	mir_.goToChargingStation();
	}
	else if (inside_or_outside_chamber_ == "outside")
	{
		
	}
	else
	{
		Utilities::to_log_file("inside or outside chamber parameter error");
		return;
	}

	int pre_chamber_id = target_chamber_id;
	
	while (1)
	{
		time(&now);
		daily_rover_start_tm = *localtime(&now);
		daily_rover_start_tm.tm_hour = start_hour;
		daily_rover_start_tm.tm_min = start_min;

		end_of_day_tm = *localtime(&now);
		end_of_day_tm.tm_hour = 23;
		end_of_day_tm.tm_min = 59;
		end_of_day_tm.tm_sec = 60;

		const double seconds_dayend_minus_rover_start = difftime(mktime(&end_of_day_tm), mktime(&daily_rover_start_tm));

		double seconds_now_minus_rover_start = difftime(now, mktime(&daily_rover_start_tm));
		if (seconds_now_minus_rover_start < 0.)	
			seconds_now_minus_rover_start += seconds_dayend_minus_rover_start 
												+ daily_rover_start_tm.tm_hour * 3600.
												+ daily_rover_start_tm.tm_min * 60.
												+ daily_rover_start_tm.tm_sec;
		
		double time_in_current_round = seconds_now_minus_rover_start / (60.*total_one_round_time);
		time_in_current_round = (time_in_current_round - std::floor(time_in_current_round))*total_one_round_time;

		double since_chamber_i_scheduled_start_time;

		// 1: enter chamber
		int action = 0;

		for (int i = 1; i <= 8; i++)
		{
			// find which chamber operation time now is
			if (i*scheduled_minutes_per_chamber_ > time_in_current_round)
			{
				since_chamber_i_scheduled_start_time = time_in_current_round - (i - 1)*scheduled_minutes_per_chamber_;

				if (since_chamber_i_scheduled_start_time >= 0. && i != cur_chamber_id_)
				{
					target_chamber_id = i;
					action = 1;
					break;
				}
				
				action = 2;

				break;
			}
		}
		
		if (action == 1)
		{
			std::string msg = "time to enter chamber " + std::to_string(target_chamber_id);
			Utilities::to_log_file(msg);

			sendRoverToChamber(target_chamber_id);
			if(target_chamber_id == 8) mir_.goToChargingStation();			
		}
		else if (action == 2)
		{
			//missed entering the target chamber, wait for
			Sleep(1000);
		}
		else if (action == 0)
		{
			//chariging and upload data
			mir_.goToChargingStation();	// this function would check if rover is charging already
			if (total_one_round_time - time_in_current_round > 20.)
			{
				data_server_manager_.uploadOneDataFolderFromTheSavedFile();

				// free up space
				data_server_manager_.deleteOutdatedData();
			}

			Sleep(1000);
		}

		//std::cout << "action " << action << std::endl;
	}
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
	PointCloudT::Ptr laser_cloud(new PointCloudT);

#if 1
	float scan_radius = 0.2;
//	scanPlantCluster(plant_center_, plant_center_[2], scan_radius);

	plant_center_[1] += 0.1;
//	scanPlantCluster(plant_center_, plant_center_[2], scan_radius);

	plant_center_[1] -= 2*0.1;
//	scanPlantCluster(plant_center_, plant_center_[2], scan_radius);

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
	img_size_cv.height = rgb_cam_->img_height_;
	img_size_cv.width = rgb_cam_->img_width_;

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
	
	rgb1 = rgb_cam_->getRGB();


	eigenMat4dToArray6(hand_pose2, array6);
	robot_arm_client_->moveHandL(array6, 0.1, 0.1);
	//robot_arm_client_->waitTillHandReachDstPose(array6);
	Sleep(1500);

	rgb2 = rgb_cam_->getRGB();

	cv::Mat R, T, R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];

	R = cv::Mat::eye(3, 3, CV_64F);
	T = cv::Mat::zeros(3, 1, CV_64F);
	T.at<double>(0, 0) = baseline;

	cv::stereoRectify(rgb_camera_matrix_cv_, rgb_dist_coeffs_cv_,
						rgb_camera_matrix_cv_, rgb_dist_coeffs_cv_,
						img_size_cv, R, T, R1, R2, P1, P2, Q,
						cv::CALIB_ZERO_DISPARITY, 1, img_size_cv, &validRoi[0], &validRoi[1]);

	std::cout << "R1" << R1 << "\n" << "R2" << R2 << "\n" << "P1" << P1 << "\n" << "P2" << P2 << "\n";

	std::cout <<"Q" <<Q << "\n";

	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	cv::Mat rmap[2][2];
	initUndistortRectifyMap(rgb_camera_matrix_cv_, rgb_dist_coeffs_cv_, R1, P1, img_size_cv, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(rgb_camera_matrix_cv_, rgb_dist_coeffs_cv_, R2, P2, img_size_cv, CV_16SC2, rmap[1][0], rmap[1][1]);

#if 0
	initUndistortRectifyMap(rgb_camera_matrix_cv_, rgb_dist_coeffs_cv_, cv::Mat(), 
							rgb_camera_matrix_cv_, img_size_cv, CV_16SC2, rmap[0][0], rmap[0][1]);

	rmap[1][0] = rmap[0][0];
	rmap[1][1] = rmap[0][1];
	Q.at<double>(2, 3) = rgb_camera_matrix_cv_.at<double>(0, 0);
	Q.at<double>(0, 3) = -rgb_camera_matrix_cv_.at<double>(0, 2);
	Q.at<double>(1, 3) = -rgb_camera_matrix_cv_.at<double>(1, 2);
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


void VisionArmCombo::controlChamber(int chamber_id, int action)
{
	boost::asio::io_service io_service;
	boost::asio::ip::udp::socket socket(io_service);
	boost::asio::ip::udp::endpoint remote_endpoint;

	socket.open(boost::asio::ip::udp::v4());

	remote_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string("10.25.215.211"), 10000);

	boost::system::error_code err;

	std::string command;

	if (action == OPEN_DOOR)
		command = "open door ";
	else if(action == CLOSE_DOOR)
		command = "close door ";
	else if (action == OPEN_CURTAIN)
		command = "open curtain ";
	else if (action == CLOSE_CURTAIN)
		command = "close curtain ";
	else if (action == PAUSE_LIGHT)
		command = "pause lighting ";
	else if (action == RESUME_LIGHT)
		command = "resume lighting ";

	command += std::to_string(chamber_id) + " rover";

	if (action == DIM_LIGHT)
		command = "dim-lighting " + std::to_string(chamber_id) + " 1 rover";

	char recv_buffer[128];
	int send_success = 0;
	for (int try_id = 0; try_id < 5 && send_success == 0; try_id++)
	{
		socket.send_to(boost::asio::buffer(command, command.size()), remote_endpoint, 0, err);

		for (int i = 0; i < 10; i++)
		{
			int num_bytes = socket.available();

			if (num_bytes > 0)
			{
				int num_bytes = socket.receive_from(boost::asio::buffer(recv_buffer, 1024), remote_endpoint, 0, err);	//blocking read
				std::string result(recv_buffer, recv_buffer + num_bytes);
				std::cout << result << std::endl;
				send_success = 1;
				break;
			}
			else
			{
				//no response from server
				if (i == 5)
				{
					Utilities::to_log_file(command);
				}
			}

			Sleep(1000);
		}

		Sleep(3000);
	}

	socket.close();
}

void VisionArmCombo::hyperspectralCameraCalibration()
{

	//if (hypercam_ == NULL)
	//{
	//	std::cout << "hyperspectral camera not initialized!";
	//	return;
	//}

	//if (robot_arm_client_ == NULL)
	//{
	//	std::cout << "robot arm not initialized!";
	//	return;
	//}

	//double step_size = 0.001;//mm

	//int num_success_scan = 0;

	std::vector<cv::Vec6d> hand_poses;
	std::vector<std::vector<cv::Point2d>> image_points_vec;

	cv::Size boardSize, imageSize;
	double squareSize = 0.02;

	//std::vector<std::vector<cv::Point2d>> imagePoints;

	// IMPORTANT
	cv::SimpleBlobDetector::Params params;
	params.maxArea = 200 * 200;
	params.minArea = 20 * 20;
	cv::Ptr<cv::FeatureDetector> blobDetect = cv::SimpleBlobDetector::create(params);

	boardSize.width = 4;
	boardSize.height = 11;

	std::vector<cv::Point3d> corners;

	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(cv::Point3d(((2 * j + i % 2)*squareSize), (i*squareSize), 0.));

	const int num_points = boardSize.width*boardSize.height;

	const int num_frames = 20;

	

	for (int frame_id = 0; frame_id < num_frames; )
	{
		std::cout << "Move to START scan pose " << std::to_string(frame_id) << " then press any key.\n"; std::getchar();

		Eigen::Matrix4d camera_pose; getCurHandPoseD(camera_pose);

		camera_pose *= hand_to_hyperspectral_d_;

		cv::Vec6d start_scan_pose;
		cv::Mat thumbnail;

		tiltLinearScan(camera_pose, thumbnail, start_scan_pose);

		cv::Mat img = showHyperspectralImage(hypercam_->scanlines_);

		std::vector<cv::Point2f> pointbuf;

		// ASYMMETRIC CIRCLE GRID PATTERN
		bool found = findCirclesGrid(img, boardSize, pointbuf, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetect);

		cv::Mat color;
		cv::cvtColor(img, color, CV_GRAY2BGR);

		if (found)
		{
			cv::drawChessboardCorners(color, boardSize, cv::Mat(pointbuf), found);
			cv::circle(color, pointbuf[0], 30, cv::Scalar(0, 255, 0), 4);
			cv::circle(color, pointbuf[1], 30, cv::Scalar(0, 0, 255), 4);

			std::ofstream out("hyperspectral_calibration_data\\data.csv", std::ios::out | std::ios::app);

			for (int point_id = 0; point_id < pointbuf.size(); point_id++)
			{
				int y = (int)std::roundf(pointbuf[point_id].y);

				int64_t ts = hypercam_->timestamps_[y].QuadPart;

				int best_idx = -1;
				int64_t min_diff = std::numeric_limits<int64_t>().max();

				for (int i = 0; i < robot_arm_client_->timestamp_pose_vec_.size(); i++)
				{
					int64_t tmp = std::abs(robot_arm_client_->timestamp_pose_vec_[i].timestamp.QuadPart - ts);

					if (tmp < min_diff)
					{
						min_diff = tmp;
						best_idx = i;
					}
				}

				if (best_idx != -1)
				{
					out << point_id << "," << pointbuf[point_id].x <<","<< std::setprecision(std::numeric_limits<long double>::digits10 + 1) 
						<< robot_arm_client_->timestamp_pose_vec_[best_idx].pose[0] << ","
						<< robot_arm_client_->timestamp_pose_vec_[best_idx].pose[1] << ","
						<< robot_arm_client_->timestamp_pose_vec_[best_idx].pose[2] << ","
						<< robot_arm_client_->timestamp_pose_vec_[best_idx].pose[3] << ","
						<< robot_arm_client_->timestamp_pose_vec_[best_idx].pose[4] << ","
						<< robot_arm_client_->timestamp_pose_vec_[best_idx].pose[5] <<std::endl;
				}
				else
					std::cerr << "best idx = -1" << std::endl;
			}

			out.close();

			frame_id++;
		}

		cv::imshow("img", color);	cv::waitKey(100);

	}

	return;

	std::cout << "Alway scan from left to right\n";

	std::string name = "hyperspectral_calibration_data\\hand_poses.yml";
	cv::FileStorage fs0(name, cv::FileStorage::WRITE);
	fs0 << "hand_poses" << "[";

	for (int frame_id = 0; frame_id < num_frames; )
	{
#if 1
		double start_pose[] = {0.1, -0.6, 0.6, M_PI, 0., 0.};
		robot_arm_client_->moveHandL(start_pose, 0.05, 0.1);
		
		std::cout << "Move to START scan pose " << std::to_string(frame_id)<<" then press any key.\n"; std::getchar();

		Eigen::Matrix4d hand_pose;
		getCurHandPoseD(hand_pose);

		//std::vector<double> end_pose(robot_arm_client_->cur_cartesian_info_array, robot_arm_client_->cur_cartesian_info_array + 6);

		//std::cout << "Move START scan pose, then press any key.\n";	std::getchar();
		//std::vector<double> start_pose(robot_arm_client_->cur_cartesian_info_array, robot_arm_client_->cur_cartesian_info_array + 6);

		//std::vector<double> cur_pose(start_pose.data(), start_pose.data() + 6);

		//compute translation
		Eigen::Vector3d translation = hand_pose.col(0).head(3);

		translation *= -0.35;

		// negative x 
		double vec3[3] = { translation(0), translation(1), translation(2)};

		cv::Vec6d start_scan_hand_pose;

		scanTranslateOnlyHyperspectral(vec3, start_scan_hand_pose, 0.1, 0.01, CONTINUOUS);

		std::cout << "start_scan_hand_pose " << start_scan_hand_pose << std::endl;

	//	double distance = translation.norm();

//		Eigen::Vector3d moving_dir = translation.normalized();

		//std::vector<cv::Mat> scanline_vec;

//		int step_count = 0;

		cv::Mat waterfall;
		waterfall.create(hypercam_->scanlines_.size(), hypercam_->spatial_size_, CV_64F);

		for (int i = 0; i < hypercam_->scanlines_.size(); i++)
			hypercam_->scanlines_.at(i).copyTo(waterfall.row(i));

		double min_val, max_val;

		cv::minMaxLoc(waterfall, &min_val, &max_val);

		std::cout << "min " << min_val << "  max " << max_val << std::endl;
		///(v - min)/(max-min)*255 = v*(255/(max-min)) - 255*min/(max-min)
		//min_val = 170.;

		cv::Mat waterfall_8u;
		waterfall.convertTo(waterfall_8u, CV_8U, 255. / (max_val - min_val), -255 * min_val / (max_val - min_val));

		cv::Mat img = waterfall_8u;
#endif


#if 0
		return;



		while (1)
		{
			double progress = (double)step_count*step_size;

			//check if reach end pose
			if (progress >= distance) {

				std::cout << "Reached end pose\n";

				break;
			}

			//scan line
			cv::Mat cur_image = hypercam_->getLastestFrame();

			cv::Mat cur_image_64f;
			cur_image.convertTo(cur_image_64f, CV_64F);

			cv::Mat cur_line;
			cv::reduce(cur_image_64f, cur_line, 0, CV_REDUCE_AVG);

			double min_val, max_val;

			cv::minMaxLoc(cur_line, &min_val, &max_val);

			///(v - min)/(max-min)*255 = v*(255/(max-min)) - 255*min/(max-min)

			//min_val = 170.;

			cv::Mat cur_line_8u;
			cur_line.convertTo(cur_line_8u, CV_8U, 255. / (max_val - min_val), -255.*min_val / (max_val - min_val));

			scanline_vec.push_back(cur_line_8u);

			if (scanline_vec.size() > 900) {
				std::cout << "exceed image\n";
				break;
			}


			//for (int y = 0; y < scanline_vec.size(); y++)
				//scanline_vec[y].copyTo(waterfall.row(y));
			scanline_vec[scanline_vec.size() - 1].copyTo(waterfall.row(scanline_vec.size() - 1));

			cv::imshow("scanline grayscale", waterfall);

			cv::waitKey(4);


			//move a little
			cur_pose[0] = start_pose[0] + progress*moving_dir[0];
			cur_pose[1] = start_pose[1] + progress*moving_dir[1];
			cur_pose[2] = start_pose[2] + progress*moving_dir[2];

			robot_arm_client_->moveHandL(cur_pose.data(), 0.005, 0.005);

			step_count++;



		}


		img.create(scanline_vec.size(), hypercam_->spatial_size_, CV_8U);

		for (int y = 0; y < scanline_vec.size(); y++)
			scanline_vec[y].copyTo(img.row(y));
#endif

//		std::string filename = "hyperspectral_calibration_data\\" + std::to_string(frame_id) + ".png";
//		cv::Mat img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
		std::vector<cv::Point2f> pointbuf;

		//cv::flip(img, img, 0);

		// ASYMMETRIC CIRCLE GRID PATTERN
		bool found = findCirclesGrid(img, boardSize, pointbuf, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetect);

		cv::Mat color;
		cv::cvtColor(img, color, CV_GRAY2BGR);

		if (found)
		{
			cv::drawChessboardCorners(color, boardSize, cv::Mat(pointbuf), found);
			cv::circle(color, pointbuf[0], 30, cv::Scalar(0, 255, 0), 4);
			cv::circle(color, pointbuf[1], 30, cv::Scalar(0, 0, 255), 4);

			//for (int i = 0; i < pointbuf.size(); i++) std::cout << "u=" << pointbuf[i].x << "  v=" << pointbuf[i].y << "   a=" << corners[i].x << "  b=" << corners[i].y << std::endl;

			//cv::Vec6d start_scan_hand_pose(0,0,0,0,0,0);
			
			hand_poses.push_back(start_scan_hand_pose);

			std::vector<cv::Point2d> image_points;

			for (auto & p : pointbuf)
				image_points.push_back(cv::Point2d(p.x, p.y));

			image_points_vec.push_back(image_points);

			std::string filename = "hyperspectral_calibration_data\\" + std::to_string(frame_id) + ".png";
			cv::imwrite(filename, img);

			fs0 << start_scan_hand_pose;

			solveLinearCameraCalibration(image_points_vec, corners);

			frame_id++;
		}

		cv::imshow("img", color);	cv::waitKey(100);
	}

	fs0 << "]";
	fs0.release();

		//	return;

	std::vector<Eigen::VectorXd> homography_vec;

	std::vector<Eigen::MatrixXd> M_vec;

	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_frames * 2, 3 + num_frames);

	//solve homography
	for (int frame_id = 0; frame_id < num_frames; frame_id++)
	{
		Eigen::MatrixXd lh = Eigen::MatrixXd::Zero(2 * num_points, 12);

		for (int point_id = 0; point_id < num_points; point_id++)
		{
			const double a = corners[point_id].x;
			const double b = corners[point_id].y;
			const double u = image_points_vec[frame_id][point_id].x;
			const double v = image_points_vec[frame_id][point_id].y;
			const double aa = a*a;
			const double bb = b*b;
			const double ab = a*b;
			const double au = a*u;
			const double bu = b*u;
			const double av = a*v;
			const double bv = b*v;

			lh(point_id * 2, 0) = a; lh(point_id * 2, 1) = b; lh(point_id * 2, 2) = 1.0; lh(point_id * 2, 9) = -au; lh(point_id * 2, 10) = -bu; lh(point_id * 2, 11) = -u;
			lh(point_id * 2 + 1, 3) = a; lh(point_id * 2 + 1, 4) = b; lh(point_id * 2 + 1, 5) = 1.0; lh(point_id * 2 + 1, 6) = aa; lh(point_id * 2 + 1, 7) = bb; lh(point_id * 2 + 1, 8) = ab; lh(point_id * 2 + 1, 9) = -av; lh(point_id * 2 + 1, 10) = -bv; lh(point_id * 2 + 1, 11) = -v;

			//std::cout << point_id << "  a=" << a << "  b=" << b << "  u=" << u << "  v=" << v << std::endl;
		}

		Eigen::JacobiSVD<Eigen::MatrixXd> svd(lh.transpose()*lh, Eigen::ComputeThinU | Eigen::ComputeThinV);

	//	cout << "Its singular values are:" << endl << svd.singularValues() << endl;
	//	cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
	//	cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;

				//last column corresponds to the "zero" singular value, is the nontrivial solution to Ax=0
		Eigen::VectorXd h = svd.matrixV().col(11);

		h = h / h(11);	//important

		homography_vec.push_back(h);

		Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
		H(0, 0) = h(0); H(0, 1) = h(1); H(0, 2) = h(2);
		H(1, 0) = h(3); H(1, 1) = h(4); H(1, 2) = h(5); H(1, 3) = h(6); H(1, 4) = h(7); H(1, 5) = h(8);
		H(2, 0) = h(9); H(2, 1) = h(10); H(2, 2) = h(11);

		double homograph_error = 0.;

		for (int point_id = 0; point_id < num_points; point_id++)
		{
			const double a = corners[point_id].x;
			const double b = corners[point_id].y;
			const double u = image_points_vec[frame_id][point_id].x;
			const double v = image_points_vec[frame_id][point_id].y;
			const double aa = a*a;
			const double bb = b*b;
			const double ab = a*b;
			
			Eigen::VectorXd r(6);
			r << a, b, 1, aa, bb, ab;

			Eigen::Vector3d p = (H*r);// .normalized().head(2);
			Eigen::Vector2d uv; uv << u - p(0)/p(2), v-p(1)/p(2);
			homograph_error += uv(0)*uv(0) + uv(1)*uv(1);

			//std::cout << uv.transpose() << "  " << p.transpose()<<std::endl;
			//std::cout<<u<<" "<<v<<"     "<<(H*r).transpose()<<std::endl;
		}

		homograph_error = std::sqrt(homograph_error/ num_points);

		std::cout << "homograph_error "<<homograph_error << std::endl;

		//std::cout << "h31=" << h(9) << "  h32=" << h(10) << std::endl;

		Eigen::MatrixXd M(3, 2);
		M.row(0) << h(0), h(1);
		M.row(1) << h(3) - h(9)*h(5), h(4) - h(10)*h(5);
		//M.row(1) << (h(11)*h(3) - h(9)*h(5))/(h(11)*h(11)), (h(11)*h(4) - h(10)*h(5)) / (h(11)*h(11));
		//M.row(1) << h(6)/h(9), h(7)/h(10);	//2011
		M.row(2) << h(9), h(10);

		M_vec.push_back(M);

		A(frame_id * 2, 0) = M(0, 0)*M(0,1);
		A(frame_id * 2, 1) = M(0, 0)*M(2, 1) + M(0,1)*M(2,0);
		A(frame_id * 2, 2) = M(2, 0)*M(2,1);
		A(frame_id * 2, 3 + frame_id) = M(1, 0)*M(1, 1);
		A(frame_id * 2 + 1, 0) = M(0, 0)*M(0, 0) - M(0, 1)*M(0, 1);
		A(frame_id * 2 + 1, 1) = 2.*(M(0,0)*M(2,0) - M(0,1)*M(2,1));
		A(frame_id * 2 + 1, 2) = M(2, 0)*M(2, 0) - M(2, 1)*M(2, 1);
		A(frame_id * 2 + 1, 3 + frame_id) = M(1, 0)*M(1, 0) - M(1, 1)*M(1, 1);

	}

	// solve Ax = 0
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A.transpose()*A, Eigen::ComputeThinU | Eigen::ComputeThinV);

	//cout << "Its singular values are:" << endl << svd.singularValues() << endl;
	//cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
	//cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;

	Eigen::VectorXd x = svd.matrixV().col(2+num_frames);

	//principal point
	const double u0 = -x(1) / x(0);

	// focal length
	const double f = std::sqrt(x(2) / x(0) - u0*u0);

	// solve 1/s^2 and ti3^2
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(num_frames*2, 1+num_frames);
	for (int frame_id = 0; frame_id < num_frames; frame_id++)
	{
		Eigen::MatrixXd M = M_vec[frame_id];
		B(frame_id * 2, 0) = M(1, 0)*M(1, 0);
		B(frame_id * 2 + 1, frame_id + 1) = (M(0,1)*M(0,1)-M(2,1)*M(0,1)*u0+M(2,1)*M(2,1)*(u0*u0+f*f)-M(0,1)*M(2,1)*u0) / (f*f);
	}

	Eigen::VectorXd b = Eigen::VectorXd::Ones(2 * num_frames);

	Eigen::VectorXd st = B.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	st = st.cwiseSqrt();

	std::cout << st << std::endl;

	const double s = 1./ st(0);

	// intrinsic matrix
	Eigen::Matrix3d K = Eigen::Matrix3d::Zero();

	K(0, 0) = f; 
	K(0, 2) = u0;
	K(1, 1) = s;
	K(2, 2) = 1.;

	Eigen::Matrix3d K_inverse = K.inverse();
		
	std::cout << "u0: " << u0 << "    f: " << f << "    s: "<<s<<std::endl; 

	// camera poses in calibration pattern coordinate system
	std::vector<Eigen::Matrix4d*> camera_pose_vec;
	for (int frame_id = 0; frame_id < num_frames; frame_id++)
	{
		double t3 = st(frame_id + 1);
		Eigen::VectorXd hi = homography_vec[frame_id]*t3;

		Eigen::Vector3d t;
		t << (hi(2) - u0*t3) / f, hi(5) / (t3*s), t3;

		Eigen::Matrix3d R;
		R(0, 0) = (hi(0) - u0*hi(9)) / f;
		R(0, 1) = (hi(1) - u0*hi(10)) / f;
		R(1, 0) = hi(6) / (hi(9)*s);
		R(1, 1) = hi(7) / (hi(10)*s);
		R(2, 0) = hi(9);
		R(2, 1) = hi(10);

		R.col(2) = R.col(0).cross(R.col(1));


		Eigen::JacobiSVD<Eigen::Matrix3d> svd_R(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Matrix3d U = svd_R.matrixU();
		Eigen::Matrix3d V = svd_R.matrixV().transpose();
		Eigen::Matrix3d tmp = Eigen::Matrix3d::Identity();
		tmp(2, 2) = (U*V).determinant();
		R = U*tmp*V;

		//R = svd.matrixU() * svd.matrixV().transpose();

		Eigen::Matrix4d *T = new Eigen::Matrix4d;
		*T = Eigen::Matrix4d::Identity();
		(*T).block<3, 3>(0, 0) = R;
		(*T).col(3).head(3) = t ;

		std::cout << "Frame " << frame_id << std::endl<<*T<<std::endl;

		camera_pose_vec.push_back(T);
	}

	std::getchar();


	std::vector<Eigen::Matrix4d*> tcp_pose_vec;

	for (auto & p : hand_poses)
	{
		Eigen::Matrix4d * tcp_pose = new Eigen::Matrix4d;

		array6ToEigenMat4d(p.val, *tcp_pose);

		std::cout << "tcp_pose\n" << *tcp_pose<<std::endl;

		tcp_pose_vec.push_back(tcp_pose);
	}

	Eigen::Matrix4d T;

	solveHandEyeCalibration(camera_pose_vec, tcp_pose_vec, T);

	cv::Mat hand_to_eye;

	EigenTransformToCVTransform(T, hand_to_eye);
		
	cv::FileStorage fs("HyperspectralCalibration.yml", cv::FileStorage::WRITE);

	fs << "hand_to_eye" << hand_to_eye;
	fs << "principal_point" << u0;
	fs << "focal_length" << f;

	fs.release();
}

void VisionArmCombo::initTOFCamera()
{
	tof_cam_ = new TOF_Swift();
}

void VisionArmCombo::initRGBCamera()
{
	rgb_cam_ = new BaslerRGB();
}


void VisionArmCombo::readOrUpdateChamberPotConfigurationFile(int operation)
{
	if (operation == READ_POT_CONFIG)
	{
		cv::FileStorage fs("chamber_pots_configuration.yml", cv::FileStorage::READ);

		fs["pot_diameter_vec_"] >> pot_diameter_vec_;

		//std::cout << pot_diameter_vec_ << "\n";

		pot_position_vec_.resize(num_chambers_);

		float max_plant_z = 0.f;

		for (int i = 0; i < num_chambers_; i++) {

			std::string name = "pot_position_" + std::to_string(i);

			fs[name] >> pot_position_vec_[i];

			//	std::cout << pot_position_vec_[i] << "\n";
		}

		fs.release();
	}
	else if (operation == UPDATE_POT_CONFIG)
	{
		cv::FileStorage fs("chamber_pots_configuration.yml", cv::FileStorage::WRITE);

		fs << "pot_diameter_vec_ " << pot_diameter_vec_;

		for (int i = 0; i < num_chambers_; i++) {

			std::string name = "pot_position_" + std::to_string(i);

			fs << name << pot_position_vec_[i];
		}

		fs.release();
	}
}

int VisionArmCombo::imagePots(int rover_position, int option, int data_collection_mode)
{
	if (option != IMAGING && option != PROBING)
		return -1;

	cv::Mat pot_pos = pot_position_vec_[cur_chamber_id_ - 1];

	const float pot_diameter = pot_diameter_vec_[cur_chamber_id_ - 1];

	ArmConfig imaging_config;

#if 1
	// image pots column by column
	if (option == IMAGING)
	{
		for (int x = 0; x < pot_pos.cols; x++)
		{
			// traverse pots like a snake, faster
			for (int y = 0; y < pot_pos.rows; y++)
			{
				int y_snake = x % 2 == 0 ? y : pot_pos.rows - y - 1;

				if (pot_sensor_map_.at<unsigned char>(y_snake, x) == 0) continue;

				if (pot_processed_map_.at<unsigned char>(y_snake, x) > rover_run_index_in_chamber_)
					continue;

				cv::Vec3f pot_xyz = pot_pos.at<cv::Vec3f>(y_snake, x);

				if (multi_work_position_)
				{
					// shift pot y cooridinate
					if (rover_position != 1)
						pot_xyz[1] -= work_pos_offset_map_.at(rover_position);

					//check pot position, skip if out of range
					if ((data_collection_mode == TOP_VIEW && std::abs(pot_xyz[1]) > topview_y_access_limit_)
						|| (data_collection_mode == SIDE_VIEW && std::abs(pot_xyz[1]) > 0.3f)	// robot hand is y motion is more limited in sideview
						)
					{
						std::cout << "pot_xyz[0]: " << pot_xyz[0] << std::endl;
						continue;
					}
				}

				int plant_id = y_snake*pot_pos.cols + x + 1;

				if (data_collection_mode == TOP_VIEW)
					topViewImagingRoutinePerPlant(rover_position, x, y_snake, plant_id, pot_xyz);
				else
					sideViewImagingRoutinePerPlant(plant_id, pot_xyz[1], rover_position);

				std::cout << "\n";
				pot_processed_map_.at<unsigned char>(y_snake, x) = rover_run_index_in_chamber_ + 1; saveTaskProgressToFile();

				std::cout << "pot_processed_map_:\n" << pot_processed_map_ << std::endl;
			}
		}
	}
#endif
	
	if (option != PROBING) return SUCCESS;
#if 1
	if (line_profiler_ == NULL) return SUCCESS;

	// image pots column by column
	for (int x = 0; x < pot_pos.cols; x++)
	{
		// traverse pots like a snake, faster
		for (int y = 0; y<pot_pos.rows; y++)
		{
			leaf_probing_pn_vector_.clear();
			leaf_cluster_order_.clear();

			int y_snake = x % 2 == 0 ? y : pot_pos.rows - y - 1;

			if (pot_sensor_map_.at<unsigned char>(y_snake, x) == 0) continue;

			if (pot_processed_map_.at<unsigned char>(y_snake, x) > rover_run_index_in_chamber_)
				continue;

			cv::Vec3f pot_xyz = pot_pos.at<cv::Vec3f>(y_snake, x);

			if (multi_work_position_)
			{
				// shift pot y cooridinate
				if (rover_position != 1)
					pot_xyz[1] -= work_pos_offset_map_.at(rover_position);

				//check pot position, skip if out of range
				if (std::abs(pot_xyz[1]) > topview_y_access_limit_)
				{
					std::cout << "pot_xyz[1]: " << pot_xyz[1] << std::endl;
					continue;
				}
			}

			//int plant_id = Utilities::gridID2PotID(x, y, pot_pos.rows);
			int plant_id = y_snake*pot_pos.cols + x + 1;

			if (data_collection_mode == TOP_VIEW)
				topViewProbingRoutinePerPlant(rover_position, plant_id, pot_xyz);
			else
				sideViewProbingRoutinePerPlant(plant_id, pot_xyz[1], rover_position);
		
			pot_processed_map_.at<unsigned char>(y_snake, x) = rover_run_index_in_chamber_ + 1; saveTaskProgressToFile();

			std::cout << "pot_processed_map_:\n" << pot_processed_map_ << std::endl;
		}
	}

#endif	

	return SUCCESS;
}

void VisionArmCombo::createBoxCloud(Eigen::Vector3f min, Eigen::Vector3f max, float resolution, PointCloudT::Ptr cloud)
{
	for (float x = min(0); x <= max(0); x += resolution)
	{
		for (float y = min(1); y <= max(1); y += resolution)
		{
			for (float z = min(2); z <= max(2); z += resolution)
			{
				PointT p;
				p.x = x;
				p.y = y;
				p.z = z;
				p.r = p.g = p.b = 255;
				cloud->push_back(p);
			}
		}
	}
}

bool VisionArmCombo::computeImageConfigforPot(cv::Vec3f & pot_xyz, float pot_diameter, cv::Mat & camera_intrinsic, Eigen::Matrix4d & hand_to_camera, ArmConfig & imaging_config, double & dist_to_pot, Eigen::Matrix4d & target_hand_pose, int data_collection_mode)
{
	bool imaging_pose_found = false;

	double best_config_array6[6];

	if (data_collection_mode == TOP_VIEW)
	{
		Eigen::Vector3f camera_moving_dir(chamber_center_x_-pot_xyz[0], -pot_xyz[1], 0.);
		camera_moving_dir.normalize();

		dist_to_pot = 0.75*pot_diameter*std::max(camera_intrinsic.at<double>(0, 0) / camera_intrinsic.at<double>(0, 2),
			camera_intrinsic.at<double>(1, 1) / camera_intrinsic.at<double>(1, 2));

		// minimum camera to plant distance 0.15 m
		dist_to_pot = std::max(dist_to_pot, 0.15);

		// try multiple camera poses. Start from right above, then move away from the walls
		for (float offset = 0.f; offset < pot_diameter && !imaging_pose_found; offset += 0.05f)
		{
			//try rotate around z axis
			float best_angle = std::numeric_limits<float>().max();

			for (float angle = -140. / 180.*M_PI; angle <= 141. / 180.*M_PI; angle += 10. / 180.*M_PI)
			{
				//std::cout << "try angle " << angle << std::endl;
				Eigen::Matrix4d rot = Eigen::Matrix4d::Identity();
				rot.block<3, 3>(0, 0) = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).matrix();

				Eigen::Matrix4d camera_pose = Eigen::Matrix4d::Identity();
				camera_pose(0, 0) = 0.; 
				camera_pose(1, 0) = -1.;
				camera_pose(1, 1) = 0.;
				camera_pose(0, 1) = -1.;
				camera_pose(2, 2) = -1.;
				camera_pose(0, 3) = pot_xyz[0] + camera_moving_dir(0)*offset;
				camera_pose(1, 3) = pot_xyz[1] + camera_moving_dir(1)*offset;;
				camera_pose(2, 3) = pot_xyz[2] + dist_to_pot;

				camera_pose = camera_pose*rot;

				//limit hand z upper limit
				if (camera_pose(2, 3) > 0.96 - 0.19)
					camera_pose(2, 3) = 0.96 - 0.19;

				Eigen::Matrix4d hand_pose = camera_pose*hand_to_camera.inverse();

				ArmConfig arm_config;

				if (checkHandPoseReachable(hand_pose, arm_config, ONLY_CHECK_DESTINATION))
				{
					if (std::abs(angle) < std::abs(best_angle))
					{
						best_angle = angle;

						std::memcpy((uchar*)best_config_array6, (uchar*)arm_config.joint_pos_d, sizeof(double)*num_joints_);

						target_hand_pose = hand_pose;

						imaging_pose_found = true;
					}
				}
			}
		}
	}
	else if (data_collection_mode == SIDE_VIEW)
	{
		double best_base_joint_angle = 0.;
		const double target_base_joint_angle = pcl::deg2rad(-90.);

		for (double y = -0.4; y < 0.1; y += 0.01)
		{
			Eigen::Matrix4d camera_pose = Eigen::Matrix4d::Identity();
			camera_pose(1, 3) = y;
			camera_pose(0, 3) = 0.1;
			camera_pose(2, 3) = 0.9;

			// camera z point to plant top
			camera_pose.col(2).head(3) << pot_xyz(0), pot_xyz(1), pot_xyz(2);
			camera_pose.col(2).head(3) -= camera_pose.col(3).head(3);
			camera_pose.col(2).head(3).normalize();

			// camera x 
			camera_pose.col(0).head(3) = Eigen::Vector3d::UnitZ().cross(camera_pose.col(2).head<3>());

			//camera y
			camera_pose.col(1).head(3) = camera_pose.col(2).head<3>().cross(camera_pose.col(0).head<3>());

			// rotate around camera x by half of the y FOV
			camera_pose.block<3, 3>(0, 0) = camera_pose.block<3, 3>(0, 0)
					*Eigen::AngleAxisd(std::atan(camera_intrinsic.at<double>(1, 2) / camera_intrinsic.at<double>(1, 1)), camera_pose.col(0).head(3)).matrix();

			Eigen::Matrix4d hand_pose = camera_pose*hand_to_camera.inverse();

			if (checkHandPoseReachable(hand_pose, imaging_config, ONLY_CHECK_DESTINATION))
			{
				if (std::abs(best_base_joint_angle - target_base_joint_angle) > std::abs(imaging_config.joint_pos_d[0] - target_base_joint_angle) )
				{
					best_base_joint_angle = imaging_config.joint_pos_d[0];

					std::memcpy((uchar*)best_config_array6, (uchar*)imaging_config.joint_pos_d, sizeof(double)*num_joints_);

					target_hand_pose = hand_pose;

					imaging_pose_found = true;

					Eigen::Vector3d plant_top(pot_xyz(0), pot_xyz(1), pot_xyz(2));

					dist_to_pot = (plant_top - camera_pose.col(3).head(3)).norm();

					//viewPlannedPath(imaging_config.joint_pos_f, imaging_config.joint_pos_f, true);
				}
			}
		}
	}

	if (imaging_pose_found)
	{
		imaging_config.setJointPos(best_config_array6[0], best_config_array6[1], best_config_array6[2],
			best_config_array6[3], best_config_array6[4], best_config_array6[5]);
	}

	return imaging_pose_found;
}

int VisionArmCombo::mapPotPosition(PointCloudT::Ptr cloud_in_arm_base)
{
	// keep points above shelf 
	PointCloudT::Ptr cloud(new PointCloudT);

	pcl::CropBox<PointT> crop_box;
	crop_box.setInputCloud(cloud_in_arm_base);
	crop_box.setMin(Eigen::Vector4f(-1.1f, -1.15f, shelf_z_, 1.f));
	crop_box.setMax(Eigen::Vector4f(-0.4f, 1.15f, 1.28f, 1.f));
	crop_box.filter(*cloud);

	std::string save_path(data_saving_folder_.begin(), data_saving_folder_.end());
	save_path.append("top_view_mapping.pcd");

	if (cloud->size() > 0)
		pcl::io::savePCDFileBinary(save_path, *cloud);
	else
	{
		Utilities::to_log_file("top_view_mapping cloud empty");
		return EMPTY_POINT_CLOUD;
	}

	pcl::VoxelGrid<PointT> vox;
	vox.setInputCloud(cloud);
	vox.setLeafSize(0.02f, 0.02f, 0.02f);
	PointCloudT downsample_cloud;
	vox.filter(downsample_cloud);

	// opencv kmeans
	std::vector<cv::Point3f> cv_points(downsample_cloud.points.size());

	int index = 0;
	for (auto & p:downsample_cloud.points)
	{
		cv::Point3f point;
		point.x = p.x;
		point.y = p.y;
		point.z = -0.55f;		// distance between soil and base, project point cloud to x-y plane to get pot center (x,y)
		cv_points[index++] = point;
	}

	int num_plants = pot_position_vec_[cur_chamber_id_ - 1].cols*pot_position_vec_[cur_chamber_id_ - 1].rows;

	cv::Mat labels, pot_centers_cv;

	//pot_centers dimension rows: number of plants, cols: 3 (x,y,z)
	cv::kmeans(cv_points, num_plants, labels, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001), 10, cv::KMEANS_PP_CENTERS, pot_centers_cv);

	// copy from cv::Mat to std::vector
	std::vector<cv::Vec3f> pot_center_vec_;
	pot_center_vec_.resize(num_plants);
	for (int i = 0; i < pot_center_vec_.size(); i++)
		pot_center_vec_[i] = pot_centers_cv.at<cv::Vec3f>(i, 0);

	int pot_rows = pot_position_vec_[cur_chamber_id_ - 1].rows;
	int pot_cols = pot_position_vec_[cur_chamber_id_ - 1].cols;

	std::sort(pot_center_vec_.begin(), pot_center_vec_.end(), VisionArmCombo::pot_center_x_comparator);

	for (int i = 0; i < pot_rows; i++)
		std::sort(pot_center_vec_.begin() + pot_cols*i, pot_center_vec_.begin() + pot_cols*(i + 1), VisionArmCombo::pot_center_y_comparator);

	// update 
	pot_position_vec_[cur_chamber_id_ - 1].create(pot_rows, pot_cols, CV_32FC3);

	crop_box.setInputCloud(cloud);

	for (int i = 0; i < pot_center_vec_.size(); i++) {
		int x = i % pot_cols;
		int y = i / pot_cols;

		cv::Vec3f pot_xyz = pot_center_vec_[i];

		pcl::ModelCoefficients coeffs;
		coeffs.values.resize(4);
		coeffs.values[0] = pot_xyz[0];
		coeffs.values[1] = pot_xyz[1];
		coeffs.values[2] = pot_xyz[2];
		coeffs.values[3] = 0.05;

		viewer_->addSphere(coeffs, "sphere" + std::to_string(cv::getTickCount()));

		pot_position_vec_[cur_chamber_id_ - 1].at<cv::Vec3f>(y, x) = pot_xyz;

		const float radius = pot_diameter_vec_[cur_chamber_id_ - 1] * 0.5f;

		crop_box.setMin(Eigen::Vector4f(pot_xyz[0] - radius, pot_xyz[1] - radius, pot_xyz[2] - 1.0, 1.f));

		crop_box.setMax(Eigen::Vector4f(pot_xyz[0] + radius, pot_xyz[1] + radius, pot_xyz[2] + 1.0, 1.f));

		std::vector<int> pot_point_indices;

		crop_box.filter(pot_point_indices);

		Eigen::Vector4f min, max;

		pcl::PointIndices indices;
		indices.indices = pot_point_indices;

		pcl::getMinMax3D(*cloud, indices, min, max);

		// update pot center z
		pot_position_vec_[cur_chamber_id_ - 1].at<cv::Vec3f>(y, x)[2] = max(2);

		unsigned char r, g, b;
		r = std::rand() / 255;
		g = std::rand() / 255;
		b = std::rand() / 255;

		for (auto idx : pot_point_indices)
		{
			cloud->points[idx].r = r;
			cloud->points[idx].g = g;
			cloud->points[idx].b = b;
		}
	}

	readOrUpdateChamberPotConfigurationFile(UPDATE_POT_CONFIG);

	viewer_->removeAllPointClouds();
	viewer_->addPointCloud(cloud, "cloud");
	viewer_ -> spin();
	display();
	viewer_->removeAllShapes();

	return SUCCESS;
}

int VisionArmCombo::saveThermalImageData(int plant_id, Eigen::Matrix4d & camera_pose, cv::Mat & color_map, cv::Mat & temperature_map, int image_id)
{
	std::string time_str = getCurrentDateTimeStr();

	std::string save_path(data_saving_folder_.begin(), data_saving_folder_.end());

	std::string false_color = save_path + "thermo_" + std::to_string(plant_id) + "_f_" + std::to_string(image_id) + "_" + time_str + ".jpg";

	std::string temperature = save_path + "thermo_64F_" + std::to_string(plant_id) + "_f_" + std::to_string(image_id) + "_" + time_str + ".bin";

	cv::imwrite(false_color, color_map);

	std::ofstream out(temperature, std::ios::out | std::ios::binary);

	out.write((char*)temperature_map.data, 640 * 480 * sizeof(double));

	out.close();

	cv::Mat camera_pose_cv; EigenTransformToCVTransform(camera_pose, camera_pose_cv);

	std::string pose_file_name = save_path + "thermo_pose_" + std::to_string(plant_id) + "_f_" + std::to_string(image_id) + "_" + time_str + ".yml";
	cv::FileStorage fs(pose_file_name, cv::FileStorage::WRITE);

	fs << "thermal_camera_pose" << camera_pose_cv;

	fs.release();

	return SUCCESS;
}

int VisionArmCombo::saveRGBImageData(int plant_id, Eigen::Matrix4d & camera_pose, cv::Mat & rgb_img, int image_id)
{
	std::string time_str = getCurrentDateTimeStr();

	std::string save_path(data_saving_folder_.begin(), data_saving_folder_.end());

	std::string rgb_name = save_path + "rgb_" + std::to_string(plant_id) + "_f_" + std::to_string(image_id) + "_"+ time_str + ".jpg";

	cv::imwrite(rgb_name, rgb_img);

	cv::Mat camera_pose_cv; EigenTransformToCVTransform(camera_pose, camera_pose_cv);

	std::string pose_file_name = save_path + "rgb_pose_" + std::to_string(plant_id) + "_f_" + std::to_string(image_id) + "_" + time_str + ".yml";
	cv::FileStorage fs(pose_file_name, cv::FileStorage::WRITE);

	fs << "rgb_camera_pose" << camera_pose_cv;

	fs.release();

	return SUCCESS;
}

int VisionArmCombo::saveTOFImageData(int plant_id, Eigen::Matrix4d & camera_pose, cv::Mat & ir_img_16u, cv::Mat & depth_img_16u, int image_id)
{
	std::string time_str = getCurrentDateTimeStr();

	std::string save_path(data_saving_folder_.begin(), data_saving_folder_.end());

	std::string ir_name = save_path + "ir_" + std::to_string(plant_id) + "_f_" + std::to_string(image_id) + "_" + time_str + ".bin";

	std::string depth_name = save_path + "depth_" + std::to_string(plant_id) + "_f_" + std::to_string(image_id) + "_" + time_str + ".bin";

	std::ofstream out(ir_name, std::ios::out | std::ios::binary);

	out.write((char*)ir_img_16u.data, tof_cam_->img_size_ * sizeof(unsigned short));

	out.close();

	out.open(depth_name, std::ios::out | std::ios::binary);

	out.write((char*)depth_img_16u.data, tof_cam_->img_size_ * sizeof(unsigned short));

	out.close();

	cv::Mat camera_pose_cv; EigenTransformToCVTransform(camera_pose, camera_pose_cv);

	std::string pose_file_name = save_path + "depth_pose_" + std::to_string(plant_id) + "_f_" + std::to_string(image_id) + "_" + time_str + ".yml";

	cv::FileStorage fs(pose_file_name, cv::FileStorage::WRITE);

	fs << "depth_camera_pose" << camera_pose_cv;

	fs.release();

	return SUCCESS;
}

bool VisionArmCombo::checkHandPoseReachable(Eigen::Matrix4d & hand_pose, ArmConfig & target_config, int option)
{
	if (option != CHECK_PATH && option != ONLY_CHECK_DESTINATION && option != ONLY_CHECK_DESTINATION_SELFCOLLISION)
	{
		std::cout << "Wrong options in checkHandPoseReachable" << std::endl;
		return false;
	}

	std::vector<int> ik_sols_vec;

	pp_.inverseKinematics(hand_pose, ik_sols_vec);

	for (auto idx : ik_sols_vec)
	{
		double* sol_d = pp_.ik_sols_ + idx*num_joints_;

		target_config.setJointPos(sol_d[0], sol_d[1], sol_d[2], sol_d[3], sol_d[4], sol_d[5]);

		if (option == CHECK_PATH)
		{
			// test if there is a path to go there
			if (moveToConfigGetPointCloud(target_config, DISABLE_MOVE | DISABLE_SMOOTH_PATH
					//	| VIEW_PATH
				))
				return true;
		}
		else if (option == ONLY_CHECK_DESTINATION)
		{
			if (!pp_.collisionCheckForSingleConfig(target_config.joint_pos_f))
				return true;
			//else viewPlannedPath(target_config.joint_pos_f, target_config.joint_pos_f, true);
		}
		else if (option == ONLY_CHECK_DESTINATION_SELFCOLLISION)
		{
			if (!pp_.selfCollision(target_config.joint_pos_f))
				return true;
		}
	}

	return false;
}

//  pot_xyz assume in base frame of the current rover position in chamber
bool VisionArmCombo::rotateLaserScanPot(cv::Vec3f& pot_xyz, double angular_fov, PointCloudT::Ptr cloud, std::string filename, int rover_position)
{
	const double sensor_2_object_dist = 0.2;

	Eigen::Matrix4d scan_center_pose, scan_start_hand_pose, scan_end_hand_pose, scanner_end_pose, test_hand_pose, final_end_hand_pose;

	Eigen::Matrix4d rot = Eigen::Matrix4d::Identity();

	scan_center_pose.col(0) << 1., 0., 0., 0.; //x->y_base
	scan_center_pose.col(1) << 0., -1., 0., 0.;	//y->x_base
	scan_center_pose.col(2) << 0., 0., -1., 0.;	//z->-z_base
	scan_center_pose.col(3) << pot_xyz[0], pot_xyz[1], pot_xyz[2] + sensor_2_object_dist, 1.0;

	rot.block<3, 3>(0, 0) = Eigen::AngleAxisd(angular_fov*0.5, Eigen::Vector3d::UnitX()).matrix();

	scan_start_hand_pose = scan_center_pose*rot;

	scanner_end_pose = scan_center_pose*rot.inverse();

	// actually scan start hand pose
	scan_start_hand_pose = scan_start_hand_pose*hand_to_scanner_.inverse();
	scan_end_hand_pose = scanner_end_pose*hand_to_scanner_.inverse();

	ArmConfig start_config, end_config;
	bool start_pose_reachable = false;
	bool end_pose_reachable = false;
	
	//limit y based on rover position
	if (rover_position == 0)	//right
	{
		scan_start_hand_pose(1, 3) = std::max(scan_start_hand_pose(1, 3), -0.29);
		scan_end_hand_pose(1, 3) = std::max(scan_end_hand_pose(1, 3), -0.29);
	}
	else if (rover_position == 2)	//left
	{
		scan_start_hand_pose(1, 3) = std::min(scan_start_hand_pose(1, 3), 0.4);
		scan_end_hand_pose(1, 3) = std::min(scan_end_hand_pose(1, 3), 0.4);
	}

	//std::cout << "initial scanner start hand pose\n" << scan_start_hand_pose << "\nintial scan end hand pose\n" << scan_end_hand_pose << std::endl;;

	Eigen::Matrix4d tmp_hand_pose;
	checkHandPoseReachableAlongAxis(scan_start_hand_pose, 0.01, 0.1, tmp_hand_pose, "y", ONLY_CHECK_DESTINATION_SELFCOLLISION);
	scan_start_hand_pose = tmp_hand_pose;
	//std::cout << "scan_start_hand_pose\n" << scan_start_hand_pose << std::endl;
	start_pose_reachable = checkHandPoseReachable(scan_start_hand_pose, start_config, ONLY_CHECK_DESTINATION_SELFCOLLISION);

	checkHandPoseReachableAlongAxis(scan_end_hand_pose, -0.01, 0.1, tmp_hand_pose, "y", ONLY_CHECK_DESTINATION_SELFCOLLISION);
	scan_end_hand_pose = tmp_hand_pose;
	//std::cout << "scan_end_hand_pose\n" << scan_end_hand_pose << std::endl;
	end_pose_reachable = checkHandPoseReachable(scan_end_hand_pose, end_config, ONLY_CHECK_DESTINATION_SELFCOLLISION);

	std::cout << "start_pose_reachable " << start_pose_reachable << " \nend_pose_reachable " << end_pose_reachable << std::endl;

	if (!start_pose_reachable || !end_pose_reachable) return false;

	//move to a place higher than start config
	Eigen::Matrix4d prepare_pose = scan_start_hand_pose;
	Eigen::Matrix4d finish_pose = scan_end_hand_pose;
	ArmConfig prepare_config, finish_config;

	prepare_pose(2, 3) = max_plant_z_ + 0.25;
	finish_pose(2, 3) = max_plant_z_ + 0.25;

	//std::cout << "intial prepare pose\n" << prepare_pose << std::endl;
	checkHandPoseReachableAlongAxis(prepare_pose, 0.05, 0.4, tmp_hand_pose, "z", ONLY_CHECK_DESTINATION);

	prepare_pose = tmp_hand_pose;
	//std::cout << "then prepare pose\n" << prepare_pose << std::endl;
	if (!checkHandPoseReachable(prepare_pose, prepare_config, ONLY_CHECK_DESTINATION_SELFCOLLISION))
	{
		std::cout << "scan prepare pose NOT reachable" << std::endl;
		return false;
	}

	checkHandPoseReachableAlongAxis(finish_pose, 0.05, 0.4, tmp_hand_pose, "z", ONLY_CHECK_DESTINATION);

	finish_pose = tmp_hand_pose;

	if (!checkHandPoseReachable(finish_pose, finish_config, ONLY_CHECK_DESTINATION_SELFCOLLISION))
	{
		std::cout << "scan finish pose NOT reachable" << std::endl;
		return false;
	}

	//moveToConfigGetPointCloud(prepare_config, SKIP_PATH_PLAN);
	//robot_arm_client_->moveHandJ(prepare_config.joint_pos_d, 0.5, 1.0);
	double array6[6];
	eigenMat4dToArray6(prepare_pose, array6);
	robot_arm_client_->moveHandL(array6, 0.2, 0.5);	//safer when doing plants near chamber inner wall

	// ignore collision with plants
	if (cur_chamber_config_.light_status == 1)
		controlChamber(cur_chamber_id_, PAUSE_LIGHT);

	eigenMat4dToArray6(scan_start_hand_pose, array6);
	robot_arm_client_->moveHandL(array6, 0.05, 0.3);
	Sleep(1000);

	PointCloudT::Ptr tmp_cloud(new PointCloudT);

	// tmp_cloud is in base frame
	scanMoveL(scanner_end_pose, cloud, scan_acceleration_, scan_speed_);

	eigenMat4dToArray6(finish_pose, array6);
	robot_arm_client_->moveHandL(array6, 0.2, 0.5);	//do not wait

	controlChamber(cur_chamber_id_, RESUME_LIGHT);

	if (cloud->size() > 100)
	{
		if (filename != "")
		{
			filename += "_" + getCurrentDateTimeStr() + ".pcd";
			pcl::io::savePCDFileBinary(filename, *cloud);
		}

		// for leaf probing, remove points below pot height, otherwise will probe pot rim
		pass_.setFilterFieldName("z");
		pass_.setInputCloud(cloud);
		pass_.setFilterLimits(shelf_z_ + pot_height_, 2.0f);
		pass_.filter(*tmp_cloud);

		vox_.setInputCloud(tmp_cloud);
		vox_.setLeafSize(voxel_grid_size_laser_, voxel_grid_size_laser_, voxel_grid_size_laser_);
		vox_.filter(*cloud);

		viewer_->removeAllPointClouds();
		viewer_->addPointCloud(cloud, "laser");
		display();
	}

	robot_arm_client_->waitTillHandReachDstPose(array6);

	return true;
}

bool VisionArmCombo::lineScanPot(Eigen::Vector3d & min, Eigen::Vector3d & max, double scan_angle, float pot_diameter, PointCloudT::Ptr cloud, int option, std::string filename, int rover_position)
{
	if (option != LASER_SCANNER && option != HYPERSPECTRAL)
		return false;

	double sensor_2_object_dist = option == LASER_SCANNER ? 0.2 : sensor_2_object_dist = pot_diameter*1473.71 / 506.289;
	double x_range = std::abs(max(0) - min(0));
	double y = 0.5*(min(1) + max(1));
	double x = max(0) + sensor_2_object_dist*std::sin(scan_angle);
	double z = max(2) + sensor_2_object_dist*std::cos(std::abs(scan_angle));

	Eigen::Matrix4d scanner_start_pose, test_hand_pose, final_end_hand_pose;

	Eigen::Matrix4d rot = Eigen::Matrix4d::Identity();

	if (option == LASER_SCANNER)
	{
		robot_arm_client_->laserScannerLightControl(true);
		scanner_start_pose.col(0) << 0., 1., 0., 0.; //x->y_base
		scanner_start_pose.col(1) << 1., 0., 0., 0.;	//y->x_base
		scanner_start_pose.col(2) << 0., 0., -1., 0.;	//z->-z_base
		scanner_start_pose.col(3) << x, y, z, 1.0;

		rot.block<3, 3>(0, 0) = Eigen::AngleAxisd(scan_angle, Eigen::Vector3d::UnitX()).matrix();

		scanner_start_pose = scanner_start_pose*rot;

		scanner_start_pose = scanner_start_pose*hand_to_scanner_.inverse();
	}
	else if(option == HYPERSPECTRAL)
	{
		scanner_start_pose.col(0) << 0., -1., 0., 0.; //x->y_base
		scanner_start_pose.col(1) << -1., 0., 0., 0.;	//y->x_base
		scanner_start_pose.col(2) << 0., 0., -1., 0.;	//z->-z_base
		scanner_start_pose.col(3) << x, y, z, 1.0;

		rot.block<3, 3>(0, 0) = Eigen::AngleAxisd(-scan_angle, Eigen::Vector3d::UnitX()).matrix();

		scanner_start_pose = scanner_start_pose*rot;

		scanner_start_pose = scanner_start_pose*hand_to_hyperspectral_d_.inverse();
	}

	ArmConfig start_config, end_config;
	bool start_pose_reachable = false;
	bool end_pose_reachable = false;

	// limit y to avoid hit wall
	scanner_start_pose(1, 3) = std::max(scanner_start_pose(1, 3), -0.86);

	Eigen::Matrix4d scanner_end_pose = scanner_start_pose;
	scanner_end_pose(0, 3) -= x_range;

	//limit x based on rover position 
	if (rover_position == 0)	//right
	{
		scanner_start_pose(0, 3) = std::max(scanner_start_pose(0, 3), -0.36);
		scanner_end_pose(0, 3) = std::max(scanner_end_pose(0, 3), -0.36);
	}
	else if (rover_position == 2)
	{
		scanner_start_pose(0, 3) = std::min(scanner_start_pose(0, 3), 0.47);
		scanner_end_pose(0, 3) = std::min(scanner_end_pose(0, 3), 0.47);
	}

	//std::cout << "initial scanner start pose\n" << scanner_start_pose << "\nintial scan end pose\n" << scanner_end_pose << std::endl;;

	Eigen::Matrix4d tmp_hand_pose;
	checkHandPoseReachableAlongAxis(scanner_start_pose, -0.01, 0.1, tmp_hand_pose, "x", ONLY_CHECK_DESTINATION_SELFCOLLISION);
	scanner_start_pose = tmp_hand_pose;
	//std::cout << "scanner_start_pose\n" << scanner_start_pose << std::endl;
	start_pose_reachable = checkHandPoseReachable(scanner_start_pose, start_config, ONLY_CHECK_DESTINATION_SELFCOLLISION);

	checkHandPoseReachableAlongAxis(scanner_end_pose, 0.01, 0.1, tmp_hand_pose, "x", ONLY_CHECK_DESTINATION_SELFCOLLISION);
	scanner_end_pose = tmp_hand_pose;
	//std::cout << "scanner_end_pose\n" << scanner_end_pose << std::endl;
	end_pose_reachable = checkHandPoseReachable(scanner_end_pose, end_config, ONLY_CHECK_DESTINATION_SELFCOLLISION);

	std::cout << "start_pose_reachable " << start_pose_reachable << " \nend_pose_reachable " << end_pose_reachable << std::endl;

	if (!start_pose_reachable || !end_pose_reachable)
	{
		robot_arm_client_->laserScannerLightControl(true);
		return false;
	}

	x_range = std::abs(scanner_end_pose(0, 3) - scanner_start_pose(0, 3));

	if (x_range < 0.05)
	{
		std::cout << "scan range too small" << std::endl;
		robot_arm_client_->laserScannerLightControl(false);
		return false;
	}

	//move to a place higher than start config
	Eigen::Matrix4d prepare_pose = scanner_start_pose;
	Eigen::Matrix4d finish_pose = scanner_end_pose;
	ArmConfig prepare_config, finish_config;

	prepare_pose(2, 3) = max_plant_z_ + 0.25;
	finish_pose(2, 3) = max_plant_z_ + 0.25;

	//std::cout << "intial prepare pose\n" << prepare_pose << std::endl;
	checkHandPoseReachableAlongAxis(prepare_pose, 0.05, 0.4, tmp_hand_pose, "z", CHECK_PATH);
	prepare_pose = tmp_hand_pose;
	//std::cout << "then prepare pose\n" << prepare_pose << std::endl;

	if (!checkHandPoseReachable(prepare_pose, prepare_config, CHECK_PATH))
	{
		std::cout << "scan prepare pose NOT reachable" << std::endl;
		robot_arm_client_->laserScannerLightControl(false);
		return false;
	}

	checkHandPoseReachableAlongAxis(finish_pose, 0.05, 0.4, tmp_hand_pose, "z", CHECK_PATH);
	finish_pose = tmp_hand_pose;

	if (!checkHandPoseReachable(finish_pose, finish_config, CHECK_PATH))
	{
		std::cout << "scan finish pose NOT reachable" << std::endl;
		robot_arm_client_->laserScannerLightControl(false);
		return false;
	}
	
	double vec3d[] = { -x_range, 0., 0.};
	moveToConfigGetPointCloud(prepare_config, VIEW_PATH);

	// ignore collision with plants
	double array6[6];
	eigenMat4dToArray6(scanner_start_pose, array6);
	robot_arm_client_->moveHandL(array6, move_arm_acceleration_, move_arm_speed_);
	Sleep(1000);

	if (option == LASER_SCANNER)
	{
		PointCloudT::Ptr tmp_cloud(new PointCloudT);

		scanTranslateOnly(vec3d, tmp_cloud, scan_acceleration_, scan_speed_);

		eigenMat4dToArray6(finish_pose, array6);
		robot_arm_client_->moveHandL(array6, move_arm_acceleration_, move_arm_speed_);
		robot_arm_client_->laserScannerLightControl(false);

		if (tmp_cloud->size() > 100)
		{
			pcl::transformPointCloud(*tmp_cloud, *cloud, (scanner_start_pose*hand_to_scanner_).cast<float>());

			if (filename != "" && cloud->size() > 100)
			{
				filename += "_" + getCurrentDateTimeStr() + ".pcd";
				pcl::io::savePCDFileBinary(filename, *cloud);
			}

			// for leaf probing, remove points below pot height, otherwise will probe pot rim
			pass_.setFilterFieldName("z");
			pass_.setInputCloud(cloud);
			pass_.setFilterLimits(shelf_z_ + pot_height_, 2.0f);
			pass_.filter(*tmp_cloud);

			vox_.setInputCloud(tmp_cloud);
			vox_.setLeafSize(voxel_grid_size_laser_, voxel_grid_size_laser_, voxel_grid_size_laser_);
			vox_.filter(*cloud);

			viewer_->removeAllPointClouds();
			viewer_->addPointCloud(cloud, "laser");
			display();
		}
	}
	else if (option == HYPERSPECTRAL)
	{
		scanTranslateOnlyHyperspectral(vec3d, start_scan_pose_, 0.1, 0.03, CONTINUOUS);

		std::cout << "scan done\n";

		showHyperspectralImage(hypercam_->scanlines_);
	}
	
	return true;
}

bool VisionArmCombo::checkHandPoseReachableAlongAxis(Eigen::Matrix4d & start_hand_pose, double step, double range, Eigen::Matrix4d & result_hand_pose, std::string axis, int option)
{
	// search along axis to find the first reachable scan start pose
	double sign = step > 0. ? 1.0 : -1.0;
	for (float offset = 0; offset <=range; offset += std::abs(step))
	{
		Eigen::Matrix4d test_pose = start_hand_pose;

		if(axis == "x")
			test_pose(0, 3) += offset*sign;
		else if(axis == "y")
			test_pose(1, 3) += offset*sign;
		else
			test_pose(2, 3) += offset*sign;

		ArmConfig arm_config;
		
		if (checkHandPoseReachable(test_pose, arm_config, option))
		{
			result_hand_pose = test_pose;
			return true;
		}
	}

	return false;
}

int VisionArmCombo::checkCurtainOpenOrClosed(int chamber_id, int option, int data_collection_mode)
{
	if (option != OPEN_CURTAIN && option != CLOSE_CURTAIN)
		return WRONG_OPTION;

	if (data_collection_mode != TOP_VIEW && data_collection_mode != SIDE_VIEW)
		return -9;

	if (data_collection_mode == TOP_VIEW)
	{
		if (!moveToConfigGetPointCloud(home_config_, SKIP_PATH_PLAN))
			return -1;
	}
	else if(data_collection_mode == SIDE_VIEW && option == OPEN_CURTAIN)
	{
		ArmConfig sideview_check_curtain_config;
		sideview_check_curtain_config.setJointPos(17.29, -73.56, -113.85, 7.6, 71.8, -180.);
		sideview_check_curtain_config.toRad();

		if (!moveToConfigGetPointCloud(sideview_check_curtain_config, SKIP_PATH_PLAN))
			return -1;
	}

	if (option == OPEN_CURTAIN)
	{
		if (tof_cam_->streaming_.load() == false)
			tof_cam_->start();
	}
	else
	{
		controlChamber(chamber_id, option);
		Sleep(18000);
		return 0;
	}

	cv::Mat depth;

	int time_cnt = 0;

	while (1)
	{
		depth = tof_cam_->getDepth16U();
		
		float screen_dist = cv::mean(depth).val[0];	// unit 0.1mm

		//std::cout << screen_dist << std::endl;

		if (option == OPEN_CURTAIN)
		{
			if ( (data_collection_mode_ == TOP_VIEW && screen_dist > 15000.f) 
				|| (data_collection_mode_ == SIDE_VIEW && screen_dist > 10000.f) )
			{			
				std::cout << "curtain open\n";
				break;
			}
		}

		if (++time_cnt > 30)
		{
			Utilities::to_log_file("curtain is not open");

			if(data_collection_mode_ == TOP_VIEW)
				close();
			else return SUCCESS;
		}

		Sleep(1000);
	}

	return SUCCESS;
}

int VisionArmCombo::getChamberConfig(int chamber_id, ChamberConfig & chamber_config)
{
	chamber_config.reset();

	boost::asio::io_service io_service;
	boost::asio::ip::udp::socket socket(io_service);
	boost::asio::ip::udp::endpoint remote_endpoint;

	socket.open(boost::asio::ip::udp::v4());

	remote_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string("10.25.215.211"), 10000);

	boost::system::error_code err;

	std::string command;

	char recv_buffer[1024];

	command = "get config "+std::to_string(chamber_id) + " rover";
	int send_success = 0;
	int num_bytes;
	std::string pot_sensor_map_str;

	for (int try_id = 0; try_id < 5 && send_success == 0; try_id++)
	{
		socket.send_to(boost::asio::buffer(command, command.size()), remote_endpoint, 0, err);

		for (int i = 0; i < 10; i++)
		{
			num_bytes = socket.available();

			if (num_bytes > 256)
			{
				int num_bytes = socket.receive_from(boost::asio::buffer(recv_buffer, 1024), remote_endpoint, 0, err);	//blocking read
				//std::string result(recv_buffer, recv_buffer + num_bytes);
				//std::cout << result << std::endl;
				send_success = 1;
				break;
			}
			else
			{
				//no response from server
				if (i == 5)
				{
					Utilities::to_log_file(command);
				}
			}

			Sleep(1000);
		}

		Sleep(3000);
	}

	if (send_success == 1)
	{
		std::string config_str(recv_buffer, recv_buffer + num_bytes);

		std::cout << config_str << std::endl;

		std::vector<std::string> str_vec;

		boost::split(str_vec, config_str, boost::is_any_of("\n"), boost::token_compress_on);

		std::cout << str_vec.size() << std::endl;

		for (auto str : str_vec)
		{
			std::vector<std::string> inner_str_vec;

			boost::split(inner_str_vec, str, boost::is_any_of(" "), boost::token_compress_on);

			if (inner_str_vec.size() != 2) continue;

			std::cout << inner_str_vec[0] << "___" << inner_str_vec[1] << std::endl;

			if (inner_str_vec[0] == "current_experiment:")
			{
				chamber_config.experiment_name = inner_str_vec[1];
			}
			else if (inner_str_vec[0] == "cur_chamber_time_local:")
			{
				chamber_config.chamber_time = inner_str_vec[1];
			}
			else if (inner_str_vec[0] == "pot_count:")
			{

			}
			else if (inner_str_vec[0] == "row_count:")
			{
				chamber_config.rows = std::stoi(inner_str_vec[1]);
			}
			else if (inner_str_vec[0] == "column_count:")
			{
				chamber_config.cols = std::stoi(inner_str_vec[1]);
			}
			else if (inner_str_vec[0] == "pot_height:")
			{
				chamber_config.pot_height = std::stof(inner_str_vec[1])*0.01;
			}
			else if (inner_str_vec[0] == "pot_width:")
			{
				chamber_config.pot_width = std::stof(inner_str_vec[1])*0.0254;
			}
			else if (inner_str_vec[0] == "pot_shape:")
			{

			}
			else if (inner_str_vec[0] == "lights_on:")
			{
				chamber_config.light_status = std::stoi(inner_str_vec[1]);
			}
			else if (inner_str_vec[0] == "pot_sensor_map:")
			{
				pot_sensor_map_str = inner_str_vec[1];
			}
			else if (inner_str_vec[0] == "hyperspectral:")
			{
				chamber_config.hyperspectral = std::stoi(inner_str_vec[1]);
			}
			else if (inner_str_vec[0] == "thermal:")
			{
				chamber_config.thermal = std::stoi(inner_str_vec[1]);
			}
			else if (inner_str_vec[0] == "fluorometer:")
			{
				chamber_config.fluorometer = std::stoi(inner_str_vec[1]);
			}
			else if (inner_str_vec[0] == "leaves_to_scan_max:")
			{
				chamber_config.max_num_leaves_to_scan = std::stoi(inner_str_vec[1]);
			}
		}

		chamber_config.pot_sensor_map = cv::Mat::ones(chamber_config.rows, chamber_config.cols, CV_8U);

		str_vec.clear();

		boost::split(str_vec, pot_sensor_map_str, boost::is_any_of(","), boost::token_compress_on);

		std::cout << "str_vec size " << str_vec.size() << "\n";
		for (auto s : str_vec) std::cout << s<<"-";
		std::cout << "\n";

		if (str_vec.size() == chamber_config.rows*chamber_config.cols)
		{
			str_vec.front() = str_vec.front().substr(1, str_vec.front().length() - 1);
			str_vec.back() = str_vec.back().substr(0, str_vec.back().length() - 1);
			for (int i = 0; i < str_vec.size(); i++)
			{
				if (str_vec[i] == "0" )
					chamber_config.pot_sensor_map.data[i] = 0;
			}

			std::cout << "pot sensor map:\n"<<chamber_config.pot_sensor_map<<std::endl;
		}
		else
		{
			Utilities::to_log_file("pot sensor map size not equal rows*cols");
		}

		

	}

	socket.close();

	return SUCCESS;
}

void VisionArmCombo::solveHandEyeCalibration(std::vector<Eigen::Matrix4d*> & camera_pose_vec, std::vector<Eigen::Matrix4d*> & tcp_pose_vec, Eigen::Matrix4d & T)
{
	if (camera_pose_vec.size() != tcp_pose_vec.size())
	{
		std::cerr << "camera_pose_vec.size() != tcp_pose_vec.size()" << std::endl;
		return;
	}

	const int nframes = camera_pose_vec.size();
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

	T = Eigen::Matrix4d::Identity();
	T.block<3, 3>(0, 0) = hand_to_eye_rotation;
	T.col(3).head(3) = hand_to_eye_translation;

	std::cout << "hand to eye T:\n" << T << std::endl;
}

void VisionArmCombo::solveLinearCameraCalibration(std::vector<std::vector<cv::Point2d>> &image_points_vec, std::vector<cv::Point3d> &corners)
{
	const int num_frames = image_points_vec.size();

	const int num_points = corners.size();

	if (num_frames < 3) return;

	std::vector<Eigen::VectorXd> homography_vec;

	std::vector<Eigen::MatrixXd> M_vec;

	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_frames * 2, 3 + num_frames);

	//solve homography
	for (int frame_id = 0; frame_id < num_frames; frame_id++)
	{
		Eigen::MatrixXd lh = Eigen::MatrixXd::Zero(2 * num_points, 12);

		for (int point_id = 0; point_id < num_points; point_id++)
		{
			const double a = corners[point_id].x;
			const double b = corners[point_id].y;
			const double u = image_points_vec[frame_id][point_id].x;
			const double v = image_points_vec[frame_id][point_id].y;
			const double aa = a*a;
			const double bb = b*b;
			const double ab = a*b;
			const double au = a*u;
			const double bu = b*u;
			const double av = a*v;
			const double bv = b*v;

			lh(point_id * 2, 0) = a; lh(point_id * 2, 1) = b; lh(point_id * 2, 2) = 1.0; lh(point_id * 2, 9) = -au; lh(point_id * 2, 10) = -bu; lh(point_id * 2, 11) = -u;
			lh(point_id * 2 + 1, 3) = a; lh(point_id * 2 + 1, 4) = b; lh(point_id * 2 + 1, 5) = 1.0; lh(point_id * 2 + 1, 6) = aa; lh(point_id * 2 + 1, 7) = bb; lh(point_id * 2 + 1, 8) = ab; lh(point_id * 2 + 1, 9) = -av; lh(point_id * 2 + 1, 10) = -bv; lh(point_id * 2 + 1, 11) = -v;

			//std::cout << point_id << "  a=" << a << "  b=" << b << "  u=" << u << "  v=" << v << std::endl;
		}

		Eigen::JacobiSVD<Eigen::MatrixXd> svd(lh.transpose()*lh, Eigen::ComputeThinU | Eigen::ComputeThinV);

		//	cout << "Its singular values are:" << endl << svd.singularValues() << endl;
		//	cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
		//	cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;

		//last column corresponds to the "zero" singular value, is the nontrivial solution to Ax=0
		Eigen::VectorXd h = svd.matrixV().col(11);

		h = h / h(11);	//important

		homography_vec.push_back(h);

		Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
		H(0, 0) = h(0); H(0, 1) = h(1); H(0, 2) = h(2);
		H(1, 0) = h(3); H(1, 1) = h(4); H(1, 2) = h(5); H(1, 3) = h(6); H(1, 4) = h(7); H(1, 5) = h(8);
		H(2, 0) = h(9); H(2, 1) = h(10); H(2, 2) = h(11);

		double homograph_error = 0.;

		for (int point_id = 0; point_id < num_points; point_id++)
		{
			const double a = corners[point_id].x;
			const double b = corners[point_id].y;
			const double u = image_points_vec[frame_id][point_id].x;
			const double v = image_points_vec[frame_id][point_id].y;
			const double aa = a*a;
			const double bb = b*b;
			const double ab = a*b;

			Eigen::VectorXd r(6);
			r << a, b, 1, aa, bb, ab;

			Eigen::Vector3d p = (H*r);// .normalized().head(2);
			Eigen::Vector2d uv; uv << u - p(0) / p(2), v - p(1) / p(2);
			homograph_error += uv(0)*uv(0) + uv(1)*uv(1);

			//std::cout << uv.transpose() << "  " << p.transpose()<<std::endl;
			//std::cout<<u<<" "<<v<<"     "<<(H*r).transpose()<<std::endl;
		}

		homograph_error = std::sqrt(homograph_error / num_points);

		std::cout << "homograph_error "<<homograph_error << std::endl;

		//std::cout << "h31=" << h(9) << "  h32=" << h(10) << std::endl;

		Eigen::MatrixXd M(3, 2);
		M.row(0) << h(0), h(1);
		M.row(1) << h(3) - h(9)*h(5), h(4) - h(10)*h(5);
		//M.row(1) << (h(11)*h(3) - h(9)*h(5))/(h(11)*h(11)), (h(11)*h(4) - h(10)*h(5)) / (h(11)*h(11));
		//M.row(1) << h(6)/h(9), h(7)/h(10);	//2011
		M.row(2) << h(9), h(10);

		M_vec.push_back(M);

		A(frame_id * 2, 0) = M(0, 0)*M(0, 1);
		A(frame_id * 2, 1) = M(0, 0)*M(2, 1) + M(0, 1)*M(2, 0);
		A(frame_id * 2, 2) = M(2, 0)*M(2, 1);
		A(frame_id * 2, 3 + frame_id) = M(1, 0)*M(1, 1);
		A(frame_id * 2 + 1, 0) = M(0, 0)*M(0, 0) - M(0, 1)*M(0, 1);
		A(frame_id * 2 + 1, 1) = 2.*(M(0, 0)*M(2, 0) - M(0, 1)*M(2, 1));
		A(frame_id * 2 + 1, 2) = M(2, 0)*M(2, 0) - M(2, 1)*M(2, 1);
		A(frame_id * 2 + 1, 3 + frame_id) = M(1, 0)*M(1, 0) - M(1, 1)*M(1, 1);

	}

	// solve Ax = 0
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A.transpose()*A, Eigen::ComputeThinU | Eigen::ComputeThinV);

	//cout << "Its singular values are:" << endl << svd.singularValues() << endl;
	//cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
	//cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;

	Eigen::VectorXd x = svd.matrixV().col(2 + num_frames);

	//principal point
	const double u0 = -x(1) / x(0);

	// focal length
	const double f = std::sqrt(x(2) / x(0) - u0*u0);

	// solve 1/s^2 and ti3^2
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(num_frames * 2, 1 + num_frames);
	for (int frame_id = 0; frame_id < num_frames; frame_id++)
	{
		Eigen::MatrixXd M = M_vec[frame_id];
		B(frame_id * 2, 0) = M(1, 0)*M(1, 0);
		B(frame_id * 2 + 1, frame_id + 1) = (M(0, 1)*M(0, 1) - M(2, 1)*M(0, 1)*u0 + M(2, 1)*M(2, 1)*(u0*u0 + f*f) - M(0, 1)*M(2, 1)*u0) / (f*f);
	}

	Eigen::VectorXd b = Eigen::VectorXd::Ones(2 * num_frames);

	Eigen::VectorXd st = B.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	st = st.cwiseSqrt();

	//std::cout << st << std::endl;

	const double s = 1. / st(0);

	// intrinsic matrix
	Eigen::Matrix3d K = Eigen::Matrix3d::Zero();

	K(0, 0) = f;
	K(0, 2) = u0;
	K(1, 1) = s;
	K(2, 2) = 1.;

	Eigen::Matrix3d K_inverse = K.inverse();

	std::cout << "u0: " << u0 << "    f: " << f << "    s: " << s << std::endl;

	// camera poses in calibration pattern coordinate system
	std::vector<Eigen::Matrix4d*> camera_pose_vec;
	for (int frame_id = 0; frame_id < num_frames; frame_id++)
	{
		double t3 = st(frame_id + 1);
		Eigen::VectorXd hi = homography_vec[frame_id] * t3;

		Eigen::Vector3d t;
		t << (hi(2) - u0*t3) / f, hi(5) / (t3*s), t3;

		Eigen::Matrix3d R;
		R(0, 0) = (hi(0) - u0*hi(9)) / f;
		R(0, 1) = (hi(1) - u0*hi(10)) / f;
		R(1, 0) = hi(6) / (hi(9)*s);
		R(1, 1) = hi(7) / (hi(10)*s);
		R(2, 0) = hi(9);
		R(2, 1) = hi(10);

		R.col(2) = R.col(0).cross(R.col(1));


		Eigen::JacobiSVD<Eigen::Matrix3d> svd_R(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Matrix3d U = svd_R.matrixU();
		Eigen::Matrix3d V = svd_R.matrixV().transpose();
		Eigen::Matrix3d tmp = Eigen::Matrix3d::Identity();
		tmp(2, 2) = (U*V).determinant();
		R = U*tmp*V;

		//R = svd.matrixU() * svd.matrixV().transpose();

		Eigen::Matrix4d *T = new Eigen::Matrix4d;
		*T = Eigen::Matrix4d::Identity();
		(*T).block<3, 3>(0, 0) = R;
		(*T).col(3).head(3) = t;

		std::cout << "Frame " << frame_id << std::endl << *T << std::endl;

		camera_pose_vec.push_back(T);
	}
}

int VisionArmCombo::moveArmInOrOutChamber(int option, int data_collection_mode)
{
	if (option != MOVE_ARM_IN_CHAMBER && option != MOVE_ARM_OUT_CHAMBER)
		return -1;

	if (data_collection_mode != TOP_VIEW && data_collection_mode != SIDE_VIEW)
		return -9;

	if (data_collection_mode == SIDE_VIEW)
	{
		robot_arm_client_->moveHandJ(sideview_home_config_.joint_pos_d, move_joint_speed_, move_joint_acceleration_);
		return SUCCESS;
	}

	Eigen::Matrix4d pose;
	getCurHandPoseD(pose);

	if (option == MOVE_ARM_IN_CHAMBER && pose(0, 3) <= -0.3)
		return SUCCESS;

	std::vector<std::vector<double>> in_config_sequence;

	std::vector<double> c0 = { home_config_.joint_pos_d[0], home_config_.joint_pos_d[1], home_config_.joint_pos_d[2], 
								home_config_.joint_pos_d[3], home_config_.joint_pos_d[4], home_config_.joint_pos_d[5] };
	in_config_sequence.push_back(c0);

	//std::vector<double> c = { pcl::deg2rad(-180.), pcl::deg2rad(-51.), pcl::deg2rad(-122.), pcl::deg2rad(-26.), pcl::deg2rad(90.), pcl::deg2rad(-180.) };
	std::vector<double> c = { pcl::deg2rad(-180.), pcl::deg2rad(-75.), pcl::deg2rad(-103.), pcl::deg2rad(-26.), pcl::deg2rad(90.), pcl::deg2rad(-180.) };
	in_config_sequence.push_back(c);

	c[1] = pcl::deg2rad(-87.3); c[2] = pcl::deg2rad(-79.15); c[3] = pcl::deg2rad(-103.4);
	in_config_sequence.push_back(c);

	int begin, end, delta;

	if (option == MOVE_ARM_IN_CHAMBER)
	{
		begin = 0; 
		end = in_config_sequence.size();
		delta = 1;
	}
	else if (option == MOVE_ARM_OUT_CHAMBER)
	{
		begin = in_config_sequence.size()-1;
		end = -1;
		delta = -1;
	}

	for (int i = begin; i != end; i += delta)
		robot_arm_client_->moveHandJ(in_config_sequence[i].data(), 0.7, 1.2);
		
	return SUCCESS;
}

// Input: camera_pose (tilt center pose), Output: start_scan_pose
int VisionArmCombo::tiltLinearScan(Eigen::Matrix4d &camera_pose, cv::Mat& thumbnail, cv::Vec6d & start_scan_pose, double angle, int option)
{
	if (option != ONLY_PERFORM_COLLISION_CHECK && option != DIRECT_IMAGING && option != IMAGING_IGNORE_COLLISION)
		return -1;

	Eigen::Matrix4d start_hand_pose = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d end_hand_pose = Eigen::Matrix4d::Identity();

	double angle_2 = pcl::deg2rad(angle*0.5);

	//Eigen::Matrix4d rot = Eigen::Matrix4d::Identity();

	//rot.block<3, 3>(0, 0) = Eigen::AngleAxisd(angle_2, Eigen::Vector3d::UnitX()).matrix();

	start_hand_pose = camera_pose;

	end_hand_pose = camera_pose;

	start_hand_pose.topLeftCorner<3, 3>() = camera_pose.topLeftCorner<3, 3>() * Eigen::AngleAxisd(angle_2, Eigen::Vector3d::UnitX()).matrix();

	start_hand_pose = start_hand_pose*hand_to_hyperspectral_d_.inverse();

	//rot.block<3, 3>(0, 0) = Eigen::AngleAxisd(-angle_2, Eigen::Vector3d::UnitX()).matrix();

	end_hand_pose.topLeftCorner<3,3>() = camera_pose.topLeftCorner<3, 3>() * Eigen::AngleAxisd(-angle_2, Eigen::Vector3d::UnitX()).matrix();

	end_hand_pose = end_hand_pose*hand_to_hyperspectral_d_.inverse();

	ArmConfig start_config, end_config;

	if (option == ONLY_PERFORM_COLLISION_CHECK)
	{
		bool start_reachable = checkHandPoseReachable(start_hand_pose, start_config, ONLY_CHECK_DESTINATION);

		bool end_reachable = checkHandPoseReachable(end_hand_pose, end_config, ONLY_CHECK_DESTINATION);

		//std::cout << "start_reachable " << start_reachable << "    end_reachable " << end_reachable << std::endl;

		if (start_reachable && end_reachable)
			return REACHABLE;
		else
			return NOT_REACHABLE;
	}
	else if(option == DIRECT_IMAGING)
		checkHandPoseReachable(start_hand_pose, start_config, ONLY_CHECK_DESTINATION_SELFCOLLISION);	//compute inverse kinematics

	double dist = (start_hand_pose.col(3).head(3) - end_hand_pose.col(3).head(3)).norm();
	
	const double pixel_size = 0.016; //mm
	const double focal_length = 15.; //mm
	
	const double required_num_frames = std::tan(angle_2)*focal_length * 2. / pixel_size;

	std::cout << "desired num frames  " << required_num_frames << std::endl;

	double move_speed = hypercam_->dFrameRate_*dist / required_num_frames * 1.3;

	std::cout << "move_speed  " << move_speed << std::endl;

	double start_vec6[6];

	double end_vec6[6];

	double sync_pose[6];

	eigenMat4dToArray6(start_hand_pose, start_vec6);

	eigenMat4dToArray6(end_hand_pose, end_vec6);

	if(option != IMAGING_IGNORE_COLLISION && option != DIRECT_IMAGING) moveToConfigGetPointCloud(start_config, SKIP_PATH_PLAN);

	robot_arm_client_->moveHandL(start_vec6, 0.2, 0.2);

	Sleep(1000);

	hypercam_->start(CONTINUOUS);
	
	while (hypercam_->frame_count_.load() == 0)
		Sleep(2);

	robot_arm_client_->startOrStopRecordPose(true);

	std::cout << "moving...\n";

	robot_arm_client_->moveHandL(end_vec6, 0.1, move_speed, false);

	const double final_speed = robot_arm_client_->waitTillTCPMove(move_speed*0.5);

	unsigned int start_frame_count = hypercam_->frame_count_.load();

	robot_arm_client_->getCartesianInfo(sync_pose);

	int result = robot_arm_client_->waitTillHandReachDstPose(end_vec6);

	unsigned int stop_frame_count = hypercam_->frame_count_.load();

	hypercam_->stop();

	robot_arm_client_->startOrStopRecordPose(false);

	std::cout << "final_speed " << final_speed << std::endl;

	if (result < 0)	Utilities::to_log_file("movel timeout in tiltLinearScan");

	for (int i = 0; i < 6; i++)
		start_scan_pose[i] = sync_pose[i];

	int num_profiles = hypercam_->scanlines_.size();

	std::cout << "num frames: " << num_profiles << std::endl;

	std::cout << "start frame count: " << start_frame_count << " stop frame count: " << stop_frame_count << std::endl;

#if 1
	// trim end
	hypercam_->scanlines_.erase(hypercam_->scanlines_.begin() + stop_frame_count - 1, hypercam_->scanlines_.end());
	hypercam_->timestamps_.erase(hypercam_->timestamps_.begin() + stop_frame_count - 1, hypercam_->timestamps_.end());

	// trim beginning
	hypercam_->scanlines_.erase(hypercam_->scanlines_.begin(), hypercam_->scanlines_.begin() + start_frame_count - 1);
	hypercam_->timestamps_.erase(hypercam_->timestamps_.begin(), hypercam_->timestamps_.begin() + start_frame_count - 1);

	std::cout << "after erase num frames: " << hypercam_->scanlines_.size() << "\n";

	thumbnail = showHyperspectralImage(hypercam_->scanlines_);
#endif

	return SUCCESS;
}


cv::Mat VisionArmCombo::showHyperspectralImage(std::vector<cv::Mat> & scans)
{
	if (scans.size() == 0)
		return cv::Mat();

	cv::Mat image;
	image.create(scans.size(), scans[0].cols, CV_64F);

	for (int i = 0; i < scans.size(); i++)
	{
		cv::Mat scanline;
		cv::reduce(scans.at(i), scanline, 0, CV_REDUCE_AVG, CV_64F);
		scanline.copyTo(image.row(i));
	}

	double min_val, max_val;

	cv::minMaxLoc(image, &min_val, &max_val);

	//std::cout << "min " << min_val << "  max " << max_val << std::endl;
	///(v - min)/(max-min)*255 = v*(255/(max-min)) - 255*min/(max-min)
	//min_val = 170.;

	cv::Mat image_8u;
	image.convertTo(image_8u, CV_8U, 255. / (max_val - min_val), -255 * min_val / (max_val - min_val));

	cv::Mat shrinked;
	cv::resize(image_8u, shrinked, cv::Size(), 0.5, 0.5);

	cv::imshow("hyperspectral", shrinked);
	cv::moveWindow("hyperspectral", 1330, 10);
	cv::waitKey(view_time_);

	return image_8u;
}

int VisionArmCombo::getHyperspectralImageAtNearestReachable(cv::Vec3f & pot_xyz, float pot_diameter, cv::Vec6d &start_scan_pose, cv::Mat & thumbnail)
{
	bool imaging_pose_found = false;

	Eigen::Vector3f camera_moving_dir(chamber_center_x_-pot_xyz[0], -pot_xyz[1], 0.);
	camera_moving_dir.normalize();

	double dist_to_plant = pot_diameter*0.5/std::tan(pcl::deg2rad(20.));// *1473.71 / 506.289;

	dist_to_plant = std::max(dist_to_plant, 0.3);

	Eigen::Matrix4d best_camera_pose;

	// try multiple camera poses. Start from right above, then move away from the walls
	for (float offset = 0.f; offset < pot_diameter && !imaging_pose_found; offset += 0.05f)
	{
		//try rotate around z axis
		float best_angle = std::numeric_limits<float>().max();

		for (float angle = 0. / 180.*M_PI; angle <= 141. / 180.*M_PI; angle += 10.f / 180.f*M_PI)
		{
			//std::cout << "try angle " << angle << std::endl;
			Eigen::Matrix4d rot = Eigen::Matrix4d::Identity();
			rot.block<3, 3>(0, 0) = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).matrix();

			Eigen::Matrix4d camera_pose;
			camera_pose.col(0) << -1., 0., 0., 0.;
			camera_pose.col(1) << 0., 1., 0., 0.;
			camera_pose.col(2) << 0., 0., -1., 0.;
			camera_pose.col(3) << pot_xyz[0] + camera_moving_dir(0)*offset,
									pot_xyz[1] + camera_moving_dir(1)*offset,
									pot_xyz[2] + dist_to_plant,
									1.;

			camera_pose = camera_pose*rot;

			//limit hand z upper limit
			if (camera_pose(2, 3) > 0.8) camera_pose(2, 3) = 0.8;

			cv::Vec6d start_scan_pose;
			
			int result = tiltLinearScan(camera_pose, thumbnail, start_scan_pose, hyperspectral_tilt_angle_, ONLY_PERFORM_COLLISION_CHECK);

			if (result == REACHABLE)
			{
				if (std::abs(angle) < std::abs(best_angle))
				{
					best_angle = angle;

					best_camera_pose = camera_pose;

					imaging_pose_found = true;
				}
			}
		}
	}

	if (imaging_pose_found)
	{
		tiltLinearScan(best_camera_pose, thumbnail, start_scan_pose, hyperspectral_tilt_angle_, DIRECT_IMAGING);
	}
	else
	{
		return -1;
		std::cout << "FAIL TO GET HYPERSPECTRAL IMAGE" << std::endl;
	}

	return SUCCESS;
}

int VisionArmCombo::getReferenceHyperspectralData(std::string filename, int rover_position, int mode)
{
	if (hypercam_ == NULL)
	{
		return -1;
	}
	
	if (data_collection_mode_ == SIDE_VIEW)
	{
		ArmConfig calib_config;
		calib_config.setJointPos(13.52, -90.61, -148.47, -30.94, 90., -166.5);
		calib_config.toRad();
		robot_arm_client_->moveHandJ(calib_config.joint_pos_d, move_joint_speed_, move_joint_acceleration_);
	}
	else 
	{
		double calib_joint_pos[6] = { -3.141592653, 0.0, -1.972222, -2.740167, 1.464331, -3.141592653 };
		robot_arm_client_->moveHandJ(calib_joint_pos, move_joint_speed_, move_joint_acceleration_);
	}


	//collect dark reference
	hypercam_->start(CONTINUOUS, false);

	while (hypercam_->frame_count_ < 30)
		Sleep(10);

	hypercam_->stop();

	std::string time_str = getCurrentDateTimeStr();
	cv::Mat thumbnail;

	std::string name = filename + "hsr_m_" + std::to_string(mode) + "_d_rp_" + std::to_string(rover_position) + "_" + time_str;

	hypercam_->saveData(name);

	robot_arm_client_->saveCurPose(name);

	thumbnail = showHyperspectralImage(hypercam_->scanlines_);

	std::string thumbnail_path = name + ".pgm";

	cv::imwrite(thumbnail_path, thumbnail);

	// collect white reference

	if (mode == PROBING)
		robot_arm_client_->lineLightControl(true);
	else
		robot_arm_client_->lineLightControl(false);

	//robot_arm_client_->probeCylinderControl(true);

	//robot_arm_client_->whiteBoardServoControl(true);

	hypercam_->start(CONTINUOUS, true);

	while (hypercam_->frame_count_ < 30)
		Sleep(10);

	hypercam_->stop();
	
	name = filename + "hsr_m_" + std::to_string(mode) + "_w_rp_"+ std::to_string(rover_position) + "_" + time_str;

	hypercam_->saveData(name);

	robot_arm_client_->saveCurPose(name);

	thumbnail = showHyperspectralImage(hypercam_->scanlines_);

	thumbnail_path = name + ".pgm";

	cv::imwrite(thumbnail_path, thumbnail);

	robot_arm_client_->lineLightControl(false);

	//robot_arm_client_->whiteBoardServoControl(false);
	
	//robot_arm_client_->probeCylinderControl(false);

	//moveArmToHome();

	return SUCCESS;
}

int VisionArmCombo::loadOrUpdateChamberOccupancyData(int rover_pos, int option, PointCloudT::Ptr cur_cloud)
{
	if (option != LOAD_OCCUPANCY && option != UPDATE_OCCUPANCY)
		return -1;

	if (rover_pos < 0 || rover_pos > 2)
		return -2;

	PointCloudT::Ptr tmp_cloud(new PointCloudT);

	if (option == UPDATE_OCCUPANCY && cur_cloud != NULL)
	{
		if (cur_cloud->size() < 100)
		{
			std::cout << "cloud size 0" << std::endl;
			return -3;
		}

		std::string filename = occu_cloud_dir_ + std::to_string(cur_chamber_id_)+".pcd";

		pcl::CropBox<PointT> crop_box;
		crop_box.setInputCloud(cur_cloud);
		crop_box.setMin(Eigen::Vector4f(-1.2f, -1.2f, -0.7, 1.f));
		crop_box.setMax(Eigen::Vector4f(-0.1f, 1.2f, 1.28f, 1.f));
		crop_box.filter(*tmp_cloud);
		
		//shift ocupancy cloud to rover center position
		if (rover_pos != 1)
			for (auto & p : tmp_cloud->points)
				p.y += work_pos_offset_map_.at(rover_pos);

		*tmp_cloud += *chamber_occupancy_cloud_;
	
		pcl::VoxelGrid<PointT> vox;
		vox.setInputCloud(tmp_cloud);
		vox.setLeafSize(0.01, 0.01, 0.01);
		vox.filter(*chamber_occupancy_cloud_);

		pcl::io::savePCDFileBinary(filename, *chamber_occupancy_cloud_);
	}
	
	if (option == LOAD_OCCUPANCY)
	{
		std::string filename = occu_cloud_dir_ + std::to_string(cur_chamber_id_) + ".pcd";
		pcl::io::loadPCDFile(filename, *chamber_occupancy_cloud_);

		if (chamber_occupancy_cloud_->size() > 100)
		{
			*tmp_cloud = *chamber_occupancy_cloud_;

			//shift occupancy cloud to rover position
			if (rover_pos != 1)
				for (auto & p : tmp_cloud->points)
					p.y -= work_pos_offset_map_.at(rover_pos);

			pp_.addPointCloudToOccupancyGrid(tmp_cloud);
		}
		else
			std::cerr << "Fail to load chamber occupancy data load" << std::endl;
	}

	return SUCCESS;
}

void VisionArmCombo::recordTime()
{
	tick_count_ = cv::getTickCount();
}

void VisionArmCombo::printTime(std::string msg)
{
	std::cout<<msg<< " time: "<< (cv::getTickCount() - tick_count_) / cv::getTickFrequency() << std::endl;
}

int VisionArmCombo::manualMapPotPositions(int x_start, int y_start)
{
	if (tof_cam_ == NULL)
	{
		std::cout << "tof not init\n"; std::getchar();
		close();
	}

	if (robot_arm_client_ == NULL)
	{
		std::cout << "arm not init\n"; std::getchar();
		close();
	}

	Eigen::Matrix4d init_hand_pose;

	getCurHandPoseD(init_hand_pose);

	init_hand_pose.col(0) << 0., -1., 0., 0.;
	init_hand_pose.col(1) << -1., 0., 0., 0.;
	init_hand_pose.col(2) << 0., 0., -1., 0.;

	double array6[6];

	eigenMat4dToArray6(init_hand_pose, array6);

	robot_arm_client_->moveHandL(array6, 0.05, 0.1);


	const int rows = pot_position_vec_[cur_chamber_id_ - 1].rows;
	const int cols = pot_position_vec_[cur_chamber_id_ - 1].cols;

	tof_cam_->setPower(1200);

	for (int y = y_start; y < rows; y++)
	{
		for (int x = x_start; x < cols; x++)
		{
			int new_x = y % 2 == 0 ? x : cols - x - 1;

			std::cout << "x = " << new_x << "   y = " << y << "   move hand to place pot in the image center and hit space" << std::endl;

			while (1)
			{
				cv::Mat ir = tof_cam_->getIR();

				cv::Mat ir3;

				cv::cvtColor(ir, ir3, CV_GRAY2BGR);

				cv::circle(ir3, cv::Point(ir.cols/2, ir.rows/2), 100, cv::Scalar(255, 255, 0, 0), 3);

				cv::imshow("ir", ir3);

				int key = cv::waitKey(50);

				if (key == 32)
				{
					Eigen::Matrix4d cam_pose;
					getCurHandPoseD(cam_pose);
					cam_pose = cam_pose*hand_to_depth_;

					pot_position_vec_[cur_chamber_id_ - 1].at<cv::Vec3f>(y, new_x)[0] = cam_pose(0, 3);
					pot_position_vec_[cur_chamber_id_ - 1].at<cv::Vec3f>(y, new_x)[1] = cam_pose(1, 3);
					pot_position_vec_[cur_chamber_id_ - 1].at<cv::Vec3f>(y, new_x)[2] = 0.3;

					readOrUpdateChamberPotConfigurationFile(UPDATE_POT_CONFIG);
					break;
				}
			}
		}
	}

	close();
}

// vertical scan, start_pos->y should equal dst_pos->y, so ignore dst_pos->y
int VisionArmCombo::sideViewVerticalLaserScan(PointCloudT::Ptr cloud, Eigen::Vector3d & start_pos, Eigen::Vector3d & dst_pos, 
											double start_tilt_down_angle, double dst_tilt_down_angle, std::string file_path)
{
	// laser profiler
	const double y = start_pos(1);
	Eigen::Matrix4d init_profiler_pose, start_profiler_pose, dst_profiler_pose;
	init_profiler_pose.col(0) << 0., 0., -1., 0.;
	init_profiler_pose.col(1) << 0., 1., 0., 0.;
	init_profiler_pose.col(2) << 1., 0., 0., 0.;
	init_profiler_pose.topLeftCorner<3, 3>() = init_profiler_pose.topLeftCorner<3, 3>()*Eigen::AngleAxisd(pcl::deg2rad(45.), Eigen::Vector3d::UnitY()).matrix();
	init_profiler_pose.col(3) << 0.7, y, 0.6, 1.;

	// move to preparation pose
	Eigen::Matrix4d hand_pose = init_profiler_pose*hand_to_scanner_.inverse();

	double pose6[6];

	eigenMat4dToArray6(hand_pose, pose6);

	robot_arm_client_->moveHandL(pose6, move_arm_acceleration_, move_arm_speed_);

	Sleep(1000);
	
	start_profiler_pose = Eigen::Matrix4d::Identity();
	start_profiler_pose.col(0) << 0., 1., 0., 0;
	start_profiler_pose.col(1) << 0., 0., 1., 0;
	start_profiler_pose.col(2) << 1., 0., 0., 0;
	start_profiler_pose.col(3).head(3) = start_pos;
	dst_profiler_pose.block<3, 3>(0, 0) = start_profiler_pose.block<3, 3>(0, 0);
	dst_profiler_pose.col(3).head(3) = dst_pos; 
	dst_profiler_pose(1, 3) = y;	//overwrite dst_pos->y with start_pos->y

	start_profiler_pose.block<3, 3>(0, 0) = start_profiler_pose.block<3, 3>(0, 0)*Eigen::AngleAxisd(pcl::deg2rad(start_tilt_down_angle), Eigen::Vector3d::UnitX()).matrix();
	dst_profiler_pose.block<3, 3>(0, 0) = dst_profiler_pose.block<3, 3>(0, 0)*Eigen::AngleAxisd(pcl::deg2rad(dst_tilt_down_angle), Eigen::Vector3d::UnitX()).matrix();

	hand_pose = start_profiler_pose*hand_to_scanner_.inverse();

	eigenMat4dToArray6(hand_pose, pose6);

	robot_arm_client_->moveHandL(pose6, move_arm_acceleration_, move_arm_speed_);

	Sleep(1000);

	PointCloudT::Ptr scan_cloud(new PointCloudT);

	scanMoveL(dst_profiler_pose, scan_cloud, scan_acceleration_, scan_speed_);

	if (scan_cloud->size() > 100)
	{
		file_path += "_" + getCurrentDateTimeStr() + ".pcd";
		pcl::io::savePCDFileBinary(file_path, *scan_cloud);
	}

	vox_.setLeafSize(voxel_grid_size_laser_, voxel_grid_size_laser_, voxel_grid_size_laser_);

	vox_.setInputCloud(scan_cloud);

	vox_.filter(*cloud);

	viewer_->removeAllPointClouds();
	viewer_->addPointCloud(cloud, "vertical_scan");

	display();

	// go back to preparation pose
	hand_pose = init_profiler_pose*hand_to_scanner_.inverse();
	eigenMat4dToArray6(hand_pose, pose6);
	robot_arm_client_->moveHandL(pose6, move_arm_acceleration_, move_arm_speed_);

	return 0;
}

void VisionArmCombo::topViewImagingRoutinePerPlant(int rover_position, int x, int y, int plant_id, cv::Vec3f & pot_xyz, int skip_hyperspectral)
{
	const float pot_diameter = pot_diameter_vec_[cur_chamber_id_ - 1];

	ArmConfig imaging_config;

	double distance_to_pot;

	PointCloudT::Ptr plant_cloud(new PointCloudT);
	Eigen::Vector4f plant_centroid;
	Eigen::Matrix4d target_hand_pose;

	switch (plant_imaging_stage_)
	{
	case 1:
		goto THERMAL_IMAGING;
		break;
	case 2:
		goto RGB_IMAGING;
		break;
	case 3:
		goto HYPERSPECTRAL_IMAGING;
		break;
	}

	// first use depth camera to image pot and refine pot z
	if (tof_cam_ != NULL
		&& computeImageConfigforPot(pot_xyz, pot_diameter, infrared_camera_matrix_cv_, hand_to_depth_, imaging_config, distance_to_pot, target_hand_pose)
		)
	{
		moveToConfigGetPointCloud(imaging_config, GET_POINT_CLOUD | SKIP_PATH_PLAN);

		pcl::CropBox<PointT> crop_box;
		float radius = pot_diameter*0.6;
		crop_box.setInputCloud(tof_cloud_);
		crop_box.setMin(Eigen::Vector4f(pot_xyz[0] - radius, pot_xyz[1] - radius, chamber_floor_z_, 1.f));
		crop_box.setMax(Eigen::Vector4f(pot_xyz[0] + radius, pot_xyz[1] + radius, 2.0f, 1.f));
		crop_box.filter(*plant_cloud);

		//viewer_->removeAllPointClouds(); viewer_->addPointCloud(plant_cloud, "plant"); display();

		//pcl::compute3DCentroid(*plant_cloud, plant_centroid);

		Eigen::Vector4f min, max;
		pcl::getMinMax3D(*plant_cloud, min, max);

		std::cout << "max(2) " << max(2) << " pot_xyz[2]" << pot_xyz[2] << std::endl;

		if (max(2) > pot_xyz[2])
		{
			// update the plant height
			pot_xyz[2] = max(2);
			pot_position_vec_[cur_chamber_id_ - 1].at<cv::Vec3f>(y, x)[2] = max(2);
			readOrUpdateChamberPotConfigurationFile(UPDATE_POT_CONFIG);
			//std::cout << "updated plant centroid z" << pot_xyz[2] << std::endl;
		}

		if (computeImageConfigforPot(pot_xyz, pot_diameter, infrared_camera_matrix_cv_, hand_to_depth_, imaging_config, distance_to_pot, target_hand_pose))
		{
			moveToConfigGetPointCloud(imaging_config, GET_POINT_CLOUD | SKIP_PATH_PLAN);

			loadOrUpdateChamberOccupancyData(rover_position, UPDATE_OCCUPANCY, tof_cloud_);

			Eigen::Matrix4d camera_pose; getCurHandPoseD(camera_pose);

			camera_pose = camera_pose*hand_to_depth_;

			cv::Mat ir_img, ir_img_16u, depth_img_16u;

			ir_img_16u = tof_cam_->getIR16U();

			depth_img_16u = tof_cam_->getDepth16U();

			saveTOFImageData(plant_id, camera_pose, ir_img_16u, depth_img_16u);

			ir_img = tof_cam_->getIR();

			ir_img *= 2;

			cv::imshow("ir", ir_img);
			cv::moveWindow("ir", 10, 10);
			cv::waitKey(view_time_);
		}
	}

	plant_imaging_stage_ = 1; saveTaskProgressToFile();
THERMAL_IMAGING:
	if (thermocam_ != NULL
		&& computeImageConfigforPot(pot_xyz, pot_diameter, flir_thermal_camera_matrix_cv_, hand_to_thermal_d_, imaging_config, distance_to_pot, target_hand_pose)
		)
	{
		moveToConfigGetPointCloud(imaging_config, SKIP_PATH_PLAN);

		cv::Mat color_map, temperature_map;

		if (thermocam_->isConnected)
		{
			Eigen::Matrix4d camera_pose; getCurHandPoseD(camera_pose);

			camera_pose = camera_pose*hand_to_thermal_d_;

			unsigned char focus_dist_cm = (unsigned char)(distance_to_pot*100.);

			int result = thermocam_->snapShot(color_map, temperature_map, focus_dist_cm);

			if (result == 0)
			{
				saveThermalImageData(plant_id, camera_pose, color_map, temperature_map);

				cv::imshow("thermal", color_map);
				cv::moveWindow("thermal", 680, 10);
				cv::waitKey(view_time_);
			}
			else
			{
				Utilities::to_log_file("thermal camera fail once");
				//close();
			}
		}
	}

	plant_imaging_stage_ = 2; saveTaskProgressToFile();
RGB_IMAGING:
	if (rgb_cam_ != NULL && cur_chamber_config_.light_status == 1 
		&& computeImageConfigforPot(pot_xyz, pot_diameter, rgb_camera_matrix_cv_, hand_to_rgb_, imaging_config, distance_to_pot, target_hand_pose)
		)
	{
		moveToConfigGetPointCloud(imaging_config, SKIP_PATH_PLAN);

		Eigen::Matrix4d camera_pose; getCurHandPoseD(camera_pose);

		camera_pose = camera_pose*hand_to_rgb_;

		cv::Mat rgb_img = rgb_cam_->getRGB();

		saveRGBImageData(plant_id, camera_pose, rgb_img);

		cv::Mat tmp;
		cv::resize(rgb_img, tmp, cv::Size(), 0.2, 0.2);

		cv::imshow("rgb", tmp);  cv::moveWindow("rgb", 10, 520); cv::waitKey(view_time_);
	}

	if (skip_hyperspectral == 1)
	{
		plant_imaging_stage_ = 0; saveTaskProgressToFile();
		return;
	}

	plant_imaging_stage_ = 3; saveTaskProgressToFile();
HYPERSPECTRAL_IMAGING:
	if (hypercam_ != NULL && hyperspectral_topview_ 
		&& computeImageConfigforPot(pot_xyz, pot_diameter, rgb_camera_matrix_cv_, hand_to_rgb_, imaging_config, distance_to_pot, target_hand_pose))
	{
		std::cout << "hyperspectral" << std::endl;
		// since hyperspectral imaging is slow, it is separated from the other imaging
		// move to the safe RGB pose first
		moveToConfigGetPointCloud(imaging_config, SKIP_PATH_PLAN);

		cv::Vec6d start_scan_pose;
		cv::Mat thumbnail;

		if (getHyperspectralImageAtNearestReachable(pot_xyz, pot_diameter, start_scan_pose, thumbnail) == SUCCESS)
		{
			std::string file_path(data_saving_folder_.begin(), data_saving_folder_.end());

			file_path += "hs_" + std::to_string(plant_id) + "_f_0_" + getCurrentDateTimeStr();
		
			//quality check, sometimes the camera streams but the data are all zeros
			if (cv::mean(thumbnail).val[0] < 1)
			{
				Utilities::to_log_file("hyperspectral streams zeros");
			}
			else
			{
				hypercam_->saveData(file_path);

				robot_arm_client_->saveTimeStampPoses(file_path);

				std::string thumbnail_path = file_path + ".pgm";
				cv::imwrite(thumbnail_path, thumbnail);
			}
		}
	}
}

void VisionArmCombo::topViewProbingRoutinePerPlant(int rover_position, int plant_id, cv::Vec3f & pot_xyz)
{
	const double pot_diameter = pot_diameter_vec_[cur_chamber_id_ - 1];
	double radius = pot_diameter*0.6f;
	const double height = pot_xyz[2];

	PointCloudT::Ptr cloud(new PointCloudT);

	std::string file_path(data_saving_folder_.begin(), data_saving_folder_.end());

	file_path += "ls_" + std::to_string(plant_id) + "_f_0";

	if (height > chamber_floor_z_ + pot_height_ && height < 0.6f)
	{
		robot_arm_client_->laserScannerLightControl(true);
		rotateLaserScanPot(pot_xyz, pcl::deg2rad(60.), cloud, file_path, rover_position);
		robot_arm_client_->laserScannerLightControl(false);

		std::vector<pcl::PointXYZRGBNormal> probe_pn_vec;

		std::cout << "Extract leaf probing points for plant " << plant_id << "\n";

		std::vector<pcl::PointIndices> leaf_cluster_indices_vec;
		std::vector<std::vector<Eigen::Matrix4d*>> hyperscan_hand_pose_sequence_vec;
		std::vector<std::vector<ArmConfig>> hyperscan_arm_config_sequence_vec;
		std::vector<int> hyperscan_leaf_id_vec;

		if (enable_probing_ || enable_hyperspectral_)
			extractLeafProbingPointsAndHyperspectralScanPoses(cloud,
				leaf_cluster_indices_vec,
				hyperscan_hand_pose_sequence_vec,
				hyperscan_arm_config_sequence_vec,
				hyperscan_leaf_id_vec, plant_id);

		recordTime();
		int cnt = 0;
		if (enable_probing_ == 1)
		{
			// multiple probing points on a leaf
			// go through each leaf
			int num_sampled_leaf = 0;
			
			for (int leaf_idx = 0; leaf_idx < leaf_probing_pn_vector_.size() && num_sampled_leaf < max_samples_per_plant_; leaf_idx++)
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

					if (std::sqrt(pow(p.x - pre_probed_point.x, 2.0f) + pow(p.y - pre_probed_point.y, 2.0f) + pow(p.z - pre_probed_point.z, 2.0f)) < 0.04)
						continue;

					pcl::Normal n;
					n.normal_x = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].normal_x;
					n.normal_y = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].normal_y;
					n.normal_z = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].normal_z;

					cnt++;
	
					if (probeLeaf(p, n, PAM, plant_id))
					{
						//if (raman_ != NULL)	probeLeaf(p, n, RAMAN_1064, plant_id);

						pre_probed_point.x = p.x;
						pre_probed_point.y = p.y;
						pre_probed_point.z = p.z;
						num_successful_probing++;
						num_sampled_leaf++;
					}
				}
			}
		}
		printTime("probing");
		std::cout << "probeLeaf called " << cnt << " times\n";

		if (hypercam_ != NULL && leaf_tracing_hyperspectral_) 
		{
			robot_arm_client_->lineLightControl(true);

			for (int hyperscan_idx = 0; hyperscan_idx < hyperscan_hand_pose_sequence_vec.size() && hyperscan_idx < max_samples_per_plant_; hyperscan_idx++) 
			{
				scanLeafWithHyperspectral(hyperscan_hand_pose_sequence_vec[hyperscan_idx], hyperscan_arm_config_sequence_vec[hyperscan_idx], plant_id);
			}

			robot_arm_client_->lineLightControl(false);
		}
		else 
		{
			std::cout << "Hyperspectral not initialized or leaf tracing hyperspectral imaging disabled.\n";
		}

	}
	else
		std::cout << "plant height out of range" << std::endl;

}

void VisionArmCombo::sideViewImagingRoutinePerPlant(int plant_id, double pot_y_wrt_rover, int rover_pos, int skip_hyperspectral)
{
	// move to initial pose for depth imaging
	Eigen::Matrix4d init_cam_pose, cam_pose, hand_pose;
	init_cam_pose.col(0) << 0., 1., 0., 0.;
	init_cam_pose.col(1) << 0., 0., 1., 0.;
	init_cam_pose.col(2) << 1., 0., 0., 0.;

	init_cam_pose.col(3) << sideview_camera_x_pos_, pot_y_wrt_rover, 0.6, 1.;

	cam_pose = init_cam_pose;
	cam_pose.topLeftCorner<3, 3>() = cam_pose.topLeftCorner<3, 3>() * Eigen::AngleAxisd(pcl::deg2rad(45.), Eigen::Vector3d::UnitX()).matrix();
	collectSideViewImageDataGivenPose(cam_pose, plant_id, 0, rover_pos, skip_hyperspectral);

	cam_pose = init_cam_pose;
	cam_pose.topLeftCorner<3, 3>() = cam_pose.topLeftCorner<3, 3>() * Eigen::AngleAxisd(pcl::deg2rad(20.), Eigen::Vector3d::UnitX()).matrix();
	cam_pose.col(3)(2) = 0.75;
	collectSideViewImageDataGivenPose(cam_pose, plant_id, 1, rover_pos, skip_hyperspectral);

	cam_pose = init_cam_pose;
	cam_pose.col(3)(2) = 0.9;
	collectSideViewImageDataGivenPose(cam_pose, plant_id, 2, rover_pos, skip_hyperspectral);
}

void VisionArmCombo::sideViewProbingRoutinePerPlant(int plant_id, double pot_y_wrt_rover, int rover_pos)
{
	double pose6[6];
	ArmConfig config;

	if (enable_scanner_)
	{
		if (cur_chamber_config_.light_status == 1)
			controlChamber(cur_chamber_id_, PAUSE_LIGHT);

		std::string file_path(data_saving_folder_.begin(), data_saving_folder_.end());

		file_path += "ls_" + std::to_string(plant_id) + "_f_";

		robot_arm_client_->laserScannerLightControl(true);

		PointCloudT::Ptr cloud0(new PointCloudT);
		Eigen::Vector3d start_pos(sideview_camera_x_pos_, pot_y_wrt_rover, 0.8);
		Eigen::Vector3d dst_pos(sideview_camera_x_pos_, pot_y_wrt_rover, 0.2);
		std::string file_prefix = file_path + "0";
		robot_arm_client_->laserScannerLightControl(true);
		sideViewVerticalLaserScan(cloud0, start_pos, dst_pos, 45., 45., file_prefix);

		PointCloudT::Ptr cloud1(new PointCloudT);
		start_pos(0) = 0.8;
		dst_pos(0) = 0.8;
		start_pos(2) = 0.7;
		dst_pos(2) = 0.7;
		file_prefix = file_path + "1";
		sideViewVerticalLaserScan(cloud1, start_pos, dst_pos, 0., 80., file_prefix);
		robot_arm_client_->laserScannerLightControl(false);

		controlChamber(cur_chamber_id_, RESUME_LIGHT);

		*cloud1 += *cloud0;

		viewer_->removeAllPointClouds();
		viewer_->addPointCloud(cloud1, "cloud1");
		display();

		if (enable_probing_)
		{
			std::cout << "Extract leaf probing points for plant " << plant_id << "\n";
			std::vector<pcl::PointXYZRGBNormal> probe_pn_vec;
			std::vector<pcl::PointIndices> leaf_cluster_indices_vec;
			std::vector<std::vector<Eigen::Matrix4d*>> hyperscan_hand_pose_sequence_vec;
			std::vector<std::vector<ArmConfig>> hyperscan_arm_config_sequence_vec;
			std::vector<int> hyperscan_leaf_id_vec;

			extractLeafProbingPointsAndHyperspectralScanPoses(cloud1,
				leaf_cluster_indices_vec,
				hyperscan_hand_pose_sequence_vec,
				hyperscan_arm_config_sequence_vec,
				hyperscan_leaf_id_vec, plant_id);

			std::cout << "leaf tracing sample size: " << hyperscan_hand_pose_sequence_vec.size() << std::endl;

			// multiple probing points on a leaf
			// go through each leaf
			int num_sampled_leaf = 0;
			for (int leaf_idx = 0; leaf_idx < leaf_probing_pn_vector_.size() && num_sampled_leaf < max_samples_per_plant_; leaf_idx++)
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

					if (std::sqrt(pow(p.x - pre_probed_point.x, 2.0f) + pow(p.y - pre_probed_point.y, 2.0f) + pow(p.z - pre_probed_point.z, 2.0f)) < 0.04)
						continue;

					pcl::Normal n;
					n.normal_x = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].normal_x;
					n.normal_y = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].normal_y;
					n.normal_z = leaf_probing_pn_vector_[sorted_leaf_idx][patch_idx].normal_z;

					//	continue;
					Eigen::Matrix4d probe_pose;

					if (computeSideViewProbingPose(p, n, probe_pose, config))
					{
						viewPlannedPath(config.joint_pos_f, config.joint_pos_f, true);

						Eigen::Matrix4d hand_pose = probe_pose*probe_to_hand_;

						eigenMat4dToArray6(hand_pose, pose6);

						robot_arm_client_->moveHandL(pose6, move_arm_acceleration_, move_arm_speed_);

						saveProbingData(p, n, PAM, plant_id);

						pre_probed_point.x = p.x;
						pre_probed_point.y = p.y;
						pre_probed_point.z = p.z;
						num_successful_probing++;
						num_sampled_leaf++;
					}
					//else std::cout << "probing pose not found\n";
				}
			}

			if (enable_hyperspectral_ && leaf_tracing_hyperspectral_)
			{
				robot_arm_client_->lineLightControl(true);

				for (int hyperscan_idx = 0; hyperscan_idx < hyperscan_hand_pose_sequence_vec.size() && hyperscan_idx < max_samples_per_plant_; hyperscan_idx++)
					scanLeafWithHyperspectral(hyperscan_hand_pose_sequence_vec[hyperscan_idx], hyperscan_arm_config_sequence_vec[hyperscan_idx], plant_id);

				robot_arm_client_->lineLightControl(false);
			}
		}

	}
}

void VisionArmCombo::collectSideViewImageDataGivenPose(Eigen::Matrix4d & cam_pose, int plant_id, int image_id, int rover_pos, int skip_hyperspectral)
{
	double array6d[6];

	switch (plant_imaging_stage_)
	{
	case 1:
		goto THERMAL_IMAGING;
		break;
	case 2:
		goto RGB_IMAGING;
		break;
	case 3:
		goto HYPERSPECTRAL_IMAGING;
		break;
	}

	if (enable_tof_)
	{
		tof_cam_->setPower(900);

		Eigen::Matrix4d hand_pose = cam_pose*hand_to_depth_.inverse();

		eigenMat4dToArray6(hand_pose, array6d);

		int result = robot_arm_client_->moveHandL(array6d, move_arm_acceleration_, move_arm_speed_);

		if (result < 0)	Utilities::to_log_file("movel timeout at tof");

		//wait for stop shaking
		Sleep(1000);

		Eigen::Matrix4d camera_pose; getCurHandPoseD(camera_pose);

		camera_pose = camera_pose*hand_to_thermal_d_;

		cv::Mat ir_img, ir_img_16u, depth_img_16u;

		ir_img_16u = tof_cam_->getIR16U();

		depth_img_16u = tof_cam_->getDepth16U();

		saveTOFImageData(plant_id, camera_pose, ir_img_16u, depth_img_16u);

		PointCloudT::Ptr cloud(new PointCloudT);

		tof_cam_->getPointCloud(cloud);

		cv::imshow("ir", ir_img_16u);
		cv::moveWindow("ir", 10, 10);
		cv::waitKey(view_time_);

		Eigen::Vector4f box_min(-1.0f, 0.7f, -0.65f, 1.0f);
		Eigen::Vector4f box_max(1.0f, 1.6f, 1.28f, 1.0f);

		preprocessTOFCloud(cloud, box_min, box_max);
	}

	plant_imaging_stage_ = 1; saveTaskProgressToFile();
	THERMAL_IMAGING:
	if (enable_thermo_)
	{
		Eigen::Matrix4d hand_pose = cam_pose*hand_to_thermal_d_.inverse();

		eigenMat4dToArray6(hand_pose, array6d);

		int result = robot_arm_client_->moveHandL(array6d, move_arm_acceleration_, move_arm_speed_);

		if (result < 0)	Utilities::to_log_file("movel timeout at flir");

		//wait for stop shaking
		Sleep(1000);

		cv::Mat color_map, temperature_map;

		double distance_to_plant = 0.6;

		if (thermocam_->isConnected)
		{
			Eigen::Matrix4d camera_pose; getCurHandPoseD(camera_pose);

			camera_pose = camera_pose*hand_to_thermal_d_;

			unsigned char focus_dist_cm = (unsigned char)(distance_to_plant*100.);

			thermocam_->snapShot(color_map, temperature_map, focus_dist_cm);

			saveThermalImageData(plant_id, camera_pose, color_map, temperature_map, image_id);

			cv::imshow("thermal", color_map); cv::moveWindow("thermal", 680, 10); cv::waitKey(view_time_);
		}
	}

	plant_imaging_stage_ = 2; saveTaskProgressToFile();
	RGB_IMAGING:
	if (enable_rgb_ && cur_chamber_config_.light_status == 1)
	{
		Eigen::Matrix4d hand_pose = cam_pose*hand_to_rgb_.inverse();

		eigenMat4dToArray6(hand_pose, array6d);

		int result = robot_arm_client_->moveHandL(array6d, move_arm_acceleration_, move_arm_speed_);

		if (result < 0)	Utilities::to_log_file("movel timeout at rgb");

		//wait for stop shaking
		Sleep(1000);

		Eigen::Matrix4d camera_pose; getCurHandPoseD(camera_pose);

		camera_pose = camera_pose*hand_to_rgb_;

		cv::Mat rgb_img = rgb_cam_->getRGB();

		saveRGBImageData(plant_id, camera_pose, rgb_img, image_id);

		cv::Mat tmp;
		cv::resize(rgb_img, tmp, cv::Size(), 0.2, 0.2);

		cv::imshow("rgb", tmp);  cv::moveWindow("rgb", 10, 520); cv::waitKey(view_time_);
	}

	if (skip_hyperspectral == 1)
	{
		plant_imaging_stage_ = 0; saveTaskProgressToFile();
		return;
	}

	plant_imaging_stage_ = 3; saveTaskProgressToFile();

	HYPERSPECTRAL_IMAGING:
	if (hyperspectral_topview_ && enable_hyperspectral_ && image_id != 0 && cur_chamber_config_.light_status == 1)
	{
		// move to preparation pose
		Eigen::Matrix4d prepare_cam_pose;
		prepare_cam_pose.col(0) << 0., 0., 1., 0;
		prepare_cam_pose.col(1) << 0., -1., 0., 0;
		prepare_cam_pose.col(2) << 1., 0., 0., 0;
		prepare_cam_pose.topLeftCorner<3, 3>() = prepare_cam_pose.topLeftCorner<3, 3>()*Eigen::AngleAxisd(pcl::deg2rad(-45.), Eigen::Vector3d::UnitY()).matrix();
		prepare_cam_pose.col(3) << 0.7, cam_pose.col(3)(1), 0.6;

		Eigen::Matrix4d hand_pose = prepare_cam_pose*hand_to_hyperspectral_d_.inverse();

		eigenMat4dToArray6(hand_pose, array6d);

		int result = robot_arm_client_->moveHandL(array6d, move_arm_acceleration_, move_arm_speed_);

		if (result < 0)	Utilities::to_log_file("movel timeout at specim");

		Sleep(1000);

		cv::Vec6d start_scan_pose;

		Eigen::Matrix4d tilt_center_cam_pose;
		double scan_angle;
		cv::Mat thumbnail;

		if (image_id == 1)
		{
			tilt_center_cam_pose.col(3) << sideview_camera_x_pos_, cam_pose.col(3)(1), 0.72, 1.;

			scan_angle = 75.;

			tilt_center_cam_pose.col(0) << 0., -1., 0., 0.;
			tilt_center_cam_pose.col(1) << 0., 0., -1., 0.;
			tilt_center_cam_pose.col(2) << 1., 0., 0., 0.;	
		}
		else if(image_id == 2)
		{
			tilt_center_cam_pose.col(3) << sideview_camera_x_pos_, cam_pose.col(3)(1), 0.9, 1.;

			scan_angle = 40.;

			tilt_center_cam_pose.col(0) << 0., 0., 1., 0;
			tilt_center_cam_pose.col(1) << 0., -1., 0., 0;
			tilt_center_cam_pose.col(2) << 1., 0., 0., 0;
		}

		// start_scan_pose is output
		tiltLinearScan(tilt_center_cam_pose, thumbnail, start_scan_pose, scan_angle, DIRECT_IMAGING);

		std::string file_path(data_saving_folder_.begin(), data_saving_folder_.end());

		file_path += "hs_" + std::to_string(plant_id) + "_f_"+std::to_string(image_id)+"_" + getCurrentDateTimeStr();
	
		//quality check, sometimes the camera streams but the data are all zeros
		if (cv::mean(thumbnail).val[0] < 1)
		{
			Utilities::to_log_file("hyperspectral streams zeros");
		}
		else
		{
			hypercam_->saveData(file_path);
			robot_arm_client_->saveTimeStampPoses(file_path);
			std::string thumbnail_path = file_path + ".pgm";
			cv::imwrite(thumbnail_path, thumbnail);
		}

		result = robot_arm_client_->moveHandL(array6d, move_arm_acceleration_, move_arm_speed_);

		if (result < 0)	Utilities::to_log_file("movel timeout at homing");
	}
}

bool VisionArmCombo::computeSideViewProbingPose(PointT & probe_point, pcl::Normal & normal, Eigen::Matrix4d & final_probe_pose, ArmConfig & final_config, int probe_id, int plant_id)
{
	if (probe_id == PAM)
		probe_to_hand_ = probe_to_hand_pam_;
	else if (probe_id == RAMAN_532)
		probe_to_hand_ = probe_to_hand_raman_532_;
	else if (probe_id == RAMAN_1064)
		probe_to_hand_ = probe_to_hand_raman_1064_;

	// make normal point towards plants
	if (normal.normal_x < 0.f)
	{
		normal.normal_x *= -1.f;
		normal.normal_y *= -1.f;
		normal.normal_z *= -1.f;
	}

	Eigen::Matrix4d init_probe_pose = Eigen::Matrix4d::Identity();
	init_probe_pose.col(2).head(3) = normal.getNormalVector3fMap().cast<double>();
	init_probe_pose.col(2).head(3).normalize();
	init_probe_pose.col(3).head(3) = probe_point.getVector3fMap().cast<double>();
	
	init_probe_pose.col(0).head<3>(0) = init_probe_pose.col(3).head<3>().cross(init_probe_pose.col(2).head<3>());

	init_probe_pose.col(0).head(3).normalize();	// remember sin(theta)

	// does not matter which cross which, fix here
	if (init_probe_pose.col(0)(1) < 0.)
		init_probe_pose.col(0).head(3) *= -1.;

	init_probe_pose.col(1).head(3) = init_probe_pose.col(2).head<3>().cross(init_probe_pose.col(0).head<3>());

	std::cout << "init_probe_pose\n" << init_probe_pose << "\n";

	Eigen::Matrix4d probe_pose;

	for (double angle = 0.; angle < 90.; angle += 5.)
	{
		for (int i = 0; i < 2; i++)
		{
			probe_pose = init_probe_pose;

			if(i == 0)
				probe_pose.topLeftCorner<3, 3>() = probe_pose.topLeftCorner<3, 3>() * Eigen::AngleAxisd(pcl::deg2rad(angle), Eigen::Vector3d::UnitZ()).matrix();
			else
				probe_pose.topLeftCorner<3, 3>() = probe_pose.topLeftCorner<3, 3>() * Eigen::AngleAxisd(pcl::deg2rad(-angle), Eigen::Vector3d::UnitZ()).matrix();

			std::vector<int> ik_sols_vec;

			Eigen::Matrix4d hand_pose = probe_pose*probe_to_hand_;

			pp_.inverseKinematics(hand_pose, ik_sols_vec, SIDE_VIEW);

			for (auto idx : ik_sols_vec)
			{
				double* sol_d = pp_.ik_sols_ + idx*num_joints_;

				final_config.setJointPos(sol_d[0], sol_d[1], sol_d[2], sol_d[3], sol_d[4], sol_d[5]);

				if (!pp_.selfCollision(final_config.joint_pos_f))
				{
					final_probe_pose = probe_pose;
					return true;
				}
			}
		}
	}

	return false;
}

int VisionArmCombo::saveTaskProgressToFile()
{
	cv::FileStorage fs("TaskProgress.yml", cv::FileStorage::WRITE);

	if (fs.isOpened())
	{
		fs << "inside_or_outside_chamber_" << inside_or_outside_chamber_;
		fs << "rover_run_index_in_chamber_" << rover_run_index_in_chamber_;
		fs << "rover_pos_in_chamber_" << rover_pos_in_chamber_;
		fs << "cur_chamber_id_" << cur_chamber_id_;
		fs << "data_collection_mode_" << data_collection_mode_;
		fs << "imaging_or_probing_" << imaging_or_probing_;
		fs << "data_collection_done_" << data_collection_done_;
		fs << "plant_imaging_stage_" << plant_imaging_stage_;
		fs << "data_folder_" << data_folder_;
		fs << "experiment_id_" << experiment_id_;
		fs << "pot_processed_map_" << pot_processed_map_;
		fs << "pot_sensor_map_" << pot_sensor_map_;
	}

	fs.release();

	return 0;
}

void VisionArmCombo::moveArmToHome()
{
	if (data_collection_mode_ == TOP_VIEW)
		robot_arm_client_->moveHandJ(home_config_.joint_pos_d, move_joint_speed_, move_joint_acceleration_);
	else if(data_collection_mode_ == SIDE_VIEW)
		robot_arm_client_->moveHandJ(sideview_home_config_.joint_pos_d, move_joint_speed_, move_joint_acceleration_);
	else
	{ 
		Utilities::to_log_file("wrong data_collection_mode_");
		close();
	}
}

void VisionArmCombo::close()
{
	if (hypercam_ != NULL)
		hypercam_->close();

	if (tof_cam_ != NULL)
		tof_cam_->stop();

	if (line_profiler_ != NULL)
		line_profiler_->finalize();

	if (rgb_cam_ != NULL)
		rgb_cam_->finishCamera();

	if(robot_arm_client_ != NULL)
		robot_arm_client_->stopRecvTCP();

	mir_.~MirClient();

	data_server_manager_.~ServerManager();

	exit(0);
}