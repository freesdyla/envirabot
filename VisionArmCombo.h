#ifndef VISION_ARM_COMBO_H_
#define VISION_ARM_COMBO_H_
#define _CRT_SECURE_NO_WARNINGS

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/common/angles.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ml/kmeans.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/search/search.h>
#include <pcl/registration/icp.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Eigenvalues>
#include <vector>
#include <bitset>
#include <thread>
#include <future>
#include <ctime>
#include "RobotArmClient.h" // need to be 1st due to winsock
#include "ServerManager.h"
#include "utilities.h"

#include <ddeml.h>
#include "KeyenceLineProfiler.h"
#include "TOF_Swift.h"
#include "BaslerRGB.h"
#include "PathPlanner.h"

#include "HyperspectralCamera.h"
#include "FlirThermoCamClient.h"
#include "Raman.h"


#include <Windows.h>
#include "ShellAPI.h"
#include <direct.h>

#include "boost/asio.hpp"

#include "time.h"
#include "mir_client.h"


//probes
#define RAMAN_532 0
#define RAMAN_1064 1
#define PAM 2

// compute collision-free pose for different sensor types
#define PROBE 0
#define SCANNER 1
#define THERMAL 2

//chamber interaction
#define OPEN_DOOR 0
#define CLOSE_DOOR 1
#define OPEN_CURTAIN 2
#define CLOSE_CURTAIN 3
#define PAUSE_LIGHT 4
#define RESUME_LIGHT 5

//marker detection camera
#define RGB 0
#define IR 1

#define SUCCESS 0
#define EMPTY_POINT_CLOUD -1
#define STOP_AT_WRONG_DOOR -2
#define DOOR_OPEN_FAIL -3
#define DOOR_CLOSE_FAIL -4
#define FAIL_TO_SEE_DOOR_WHEN_EXITING -5
#define CURTAIN_OPEN_FAIL -6
#define STOP_AT_DOOR_TIMEOUT -7
#define WRONG_OPTION -8

#define ENTER_CHAMBER 1
#define EXIT_CHAMBER 2

//moveToConfig
#define GET_POINT_CLOUD				1 << 0
#define DISABLE_DIRECT_PATH			1 << 1
#define DISABLE_SMOOTH_PATH			1 << 2
#define DISABLE_MOVE				1 << 3
#define SKIP_ADD_OCCUPANCY_GRID		1 << 4
#define VIEW_PATH					1 << 5
#define SKIP_PATH_PLAN				1 << 6

#define UPDATE_POT_CONFIG 0
#define READ_POT_CONFIG 1

#define MOVE_ARM_IN_CHAMBER 0
#define MOVE_ARM_OUT_CHAMBER 1

#define HYPERSPECTRAL 0
#define LASER_SCANNER 1

#define CHECK_PATH 0
#define ONLY_CHECK_DESTINATION 1
#define ONLY_CHECK_DESTINATION_SELFCOLLISION 2

//tilting linear scan
#define DIRECT_IMAGING 0
#define ONLY_PERFORM_COLLISION_CHECK 1
#define IMAGING_IGNORE_COLLISION 2
#define REACHABLE 3
#define NOT_REACHABLE 4

#define LOAD_OCCUPANCY 0
#define UPDATE_OCCUPANCY 1

#define IMAGING 0
#define PROBING 1

#define TOP_VIEW 0
#define SIDE_VIEW 1

struct VisionArmCombo
{
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;
	typedef pcl::PointNormal PointNT;
	typedef pcl::PointCloud<PointNT> PointNCloudT;
	typedef pcl::PointXYZL PointLT;
	typedef pcl::PointCloud<PointLT> PointLCloudT;

	struct ArmConfig
	{
		double joint_pos_d[6];
		float joint_pos_f[6];

		ArmConfig()
		{

		}

		ArmConfig(const ArmConfig & config)
		{
			std::memcpy(joint_pos_d, config.joint_pos_d, sizeof(double) * 6);
			std::memcpy(joint_pos_f, config.joint_pos_f, sizeof(float) * 6);
		}
		
		void setJointPos(double j1, double j2, double j3, double j4, double j5, double j6)
		{
			joint_pos_d[0] = j1; joint_pos_d[1] = j2;
			joint_pos_d[2] = j3; joint_pos_d[3] = j4;
			joint_pos_d[4] = j5; joint_pos_d[5] = j6;
			joint_pos_f[0] = (float)j1; joint_pos_f[1] = (float)j2;
			joint_pos_f[2] = (float)j3; joint_pos_f[3] = (float)j4;
			joint_pos_f[4] = (float)j5; joint_pos_f[5] = (float)j6;
		}

		void toRad() 
		{ 
			for (int i = 0; i < 6; i++) joint_pos_d[i] = joint_pos_d[i] / 180.*M_PI; 
			for (int i = 0; i < 6; i++) joint_pos_f[i] = joint_pos_f[i] / 180.f*M_PI;
		}
	};

	struct ChamberConfig
	{
		std::string experiment_name, chamber_time, pot_shape;
		int pot_count, rows, cols, max_num_leaves_to_scan;
		float pot_height, pot_width;	// meter
		int light_status, hyperspectral, thermal, fluorometer;

		ChamberConfig() { reset(); }

		void reset()
		{
			pot_count = rows = cols = max_num_leaves_to_scan = -1;
			light_status = hyperspectral = thermal = fluorometer = -1;
			pot_height = pot_width = -1.f;
		}

		void print()
		{
			std::cout << "rows: " << rows << "  cols: " << cols << "  light_status: " << light_status << "  pot_height: " << pot_height<<std::endl;
		}
	};

	std::vector<ArmConfig> imaging_config_vec;
	std::vector<std::vector<ArmConfig>> imaging_config_vec_vec_;

	std::vector<ArmConfig> rgb_top_view_imaging_config_vec_;
	std::vector<ArmConfig> thermal_top_view_imaging_config_vec_;

	// UR10 dh parameters
	const double d1 = 0.1273;
	const double a2 = -0.612;
	const double a3 = -0.5723;
	const double d4 = 0.163941;
	const double d5 = 0.1157;
	const double d6 = 0.0922;

	const double ZERO_THRESH = 1e-10;
	int SIGN(double x) { return (x > 0) - (x < 0); }
	const double PI = M_PI;

	const int num_joints_ = 6;

	RobotArmClient* robot_arm_client_ = NULL;

	KeyenceLineProfiler* line_profiler_ = NULL;

	HyperspectralCamera* hypercam_ = NULL;

	FlirThermoCamClient* thermocam_ = NULL;

	Raman* raman_ = NULL;

	TOF_Swift* tof_cam_ = NULL;

	BaslerRGB* rgb_cam_ = NULL;
	
	std::ofstream result_file_;

	std::vector<PointCloudT::Ptr> calibration_point_cloud_vec;

	Eigen::Vector3f pre_point_;

	int pre_point_idx_;

	pcl::Normal pre_normal_;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

	// Distance vector of calibration plane in sensor frame
	std::vector<Eigen::Vector4d*> normalized_plane_coefficients_vec_sensor_frame_;
	std::vector<Eigen::Matrix4d*> hand_pose_vec_;
	std::vector<Eigen::Vector3d*> plane_normal_embed_dist_vec;

	Eigen::Matrix4f guessHandToScanner_;
	Eigen::Matrix4f handToScanner_;
	Eigen::Vector3f tool_center_point_, tool_center_point_pam_, tool_center_point_raman_532_, tool_center_point_raman_1064_;
	Eigen::Matrix4d probe_to_hand_;
	Eigen::Matrix4d scan_start_to_hand_;
	Eigen::Matrix4d scan_end_to_hand_;
	Eigen::Matrix4f hand_to_hyperspectral_;
	Eigen::Matrix4f hand_to_thermal_;

	PointCloudT::Ptr tof_cloud_;

	float vis_z_range_;

	float chamber_floor_z_ = -0.72;

	PointCloudT::Ptr laser_cloud_;

	pcl::VoxelGrid<PointT> vox_;

	pcl::StatisticalOutlierRemoval<PointT> sor_;

	float voxel_grid_size_ = 0.002f;

	float voxel_grid_size_laser_ = 0.001;

	float normal_estimation_radius_ = 0.005;

	pcl::PassThrough<PointT> pass_;

	pcl::ExtractIndices<PointT> extract_indices_;

	pcl::RadiusOutlierRemoval<PointT> ror_;

	PathPlanner pp_;

	Eigen::Affine3f pre_viewer_pose_;

	float scan_acceleration_ = 0.25f;
	
	float scan_speed_ = 0.05f;

	float move_arm_speed_ = 0.05f;

	float move_arm_acceleration_ = 0.1f;

	float move_joint_speed_ = 0.4f;

	float move_joint_acceleration_ = 0.4f;

	const float speed_correction_ = 0;// -0.0085f;

	float laser_scan_period_ = 1.0/500.0;

	int view_time_ = 1000;

	int counter_;

	// kmeans
	cv::Mat object_centers_;

	float scan_radius_;

	cv::Mat rgb_hand_to_eye_cv_, rgb_camera_matrix_cv_, rgb_dist_coeffs_cv_;

	cv::Mat infrared_hand_to_eye_cv_, infrared_camera_matrix_cv_, infrared_dist_coeffs_cv_;

	cv::Mat flir_thermal_hand_to_eye_cv_, flir_thermal_camera_matrix_cv_, flir_thermal_dist_coeffs_cv_;

	cv::Ptr<cv::aruco::Dictionary> marker_dictionary_;

	cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

	float marker_length_;

	Eigen::Matrix4d cur_rgb_to_marker_, hand_to_rgb_, gripper_to_hand_, hand_to_depth_, hand_to_thermal_d_, hand_to_scanner_, hand_to_hyperspectral_d_;
		
	Eigen::Matrix4d probe_to_hand_pam_, probe_to_hand_raman_532_, probe_to_hand_raman_1064_;

	int region_grow_min_cluster_size_ = 1000;
	int region_grow_max_cluster_size_ = 10000;
	float region_grow_residual_threshold_ = 0.005f;
	float region_grow_smoothness_threshold_ = 3.0 / 180.*M_PI;
	float region_grow_curvature_threshold_ = 1.0;
	int region_grow_num_neighbors_ = 30;

	int max_samples_per_leaf_ = 4;
	float probing_patch_rmse_ = 0.001;
	float hyperscan_slice_thickness_ = 0.01;
	int enable_probing_ = 1;
	int enable_path_planning_ = 1;


	// point cloud vector for three stations. 0-left, 1-center, 2-right
	std::vector<PointCloudT::Ptr> growthChamberPointCloudVec_;
	std::vector<bool> pot_process_status_;

	float scan_pot_x_abs_limit_ = 0.5f;

	std::vector<Eigen::Vector4f*> plant_cluster_min_vec_;
	std::vector<Eigen::Vector4f*> plant_cluster_max_vec_;

	std::vector<int> kmeans_label_sorted_;

	float rover_dist_ = 0.5f;

	Eigen::Matrix4f icp_final_transformation_;

	// probe plate center test
	cv::Vec3f plate_center_;
	float plate_radius_;

	float seed_resolution_ = 0.04f;
	float color_importance_ = 0.f;
	float spatial_importance_ = 1.f;
	float normal_importance_ = 1.f;

	float sor_mean_k_ = 50;
	float sor_std_ = 1.0;

	float ror_radius_ = 0.01f;
	int ror_num_ = 4;

	std::vector<std::vector<pcl::PointXYZRGBNormal>> leaf_probing_pn_vector_;

	ArmConfig home_config_, sideview_home_config_;
	ArmConfig check_curtain_config_, map_chamber_front_config_, chamber_safe_birdseye_config_;

	int num_plants_ = 1;

	float hyperspectral_imaging_dist_ = 0.21f;

	const int num_chambers_ = 8;

	// chamber pots configuration
	cv::Vec<float, 8> pot_diameter_vec_;

	std::vector<cv::Mat> pot_position_vec_;

	static bool probing_rank_comparator(pcl::PointXYZRGBNormal & a, pcl::PointXYZRGBNormal & b) 
	{ 
		if (a.r > b.r)
			return true;
		else if (a.r == b.r)
			return std::abs(a.normal_z) > std::abs(b.normal_z);
		else
			return false;
	}

	struct LeafIDnX
	{
		int id;
		float x;
	};

	static bool leaf_x_comparator(LeafIDnX & a, LeafIDnX & b) { return a.x > b.x;}

	static bool pot_center_x_comparator(cv::Vec3f & a, cv::Vec3f & b) {	return a[0] < b[0]; }

	static bool pot_center_y_comparator(cv::Vec3f & a, cv::Vec3f & b) { return a[1] < b[1]; }

	static bool curvature_comparator(pcl::PointXYZRGBNormal & a, pcl::PointXYZRGBNormal & b) { return a.curvature < b.curvature; }

	std::vector<LeafIDnX> leaf_cluster_order_;

	std::vector<PointCloudT::Ptr> plant_laser_pc_vec_;

	float shelf_z_ = -0.6;

	cv::Vec3f plant_center_;
	Eigen::Vector3f min_point_AABB_, max_point_AABB_;

	int only_do_probing_ = 0;

	int cur_chamber_id_ = 1;

	int cur_plant_id_ = 1;

	int cur_pot_coordinate_x_ = 0;

	int cur_pot_coordinate_y_ = 0;

	std::wstring data_saving_folder_;

	ServerManager data_server_manager_;

	bool remap_pot_position_ = true;

	bool multi_work_position_ = true;

	cv::Mat pot_processed_map_;
	int cur_processing_pot_x_;
	int cur_processing_pot_y_;

	float pot_height_ = 0.22f;

	std::map<int, float> work_pos_offset_map_ = { {0, 0.5f}, {2, -0.5f} };

	cv::Vec6d start_scan_pose_;

	const float chamber_center_x_ = -0.7f;

	double hyperspectral_tilt_angle_ = 40.;

	bool enable_thermo_ = true;
	bool enable_tof_ = true;
	bool enable_rgb_ = true;
	bool enable_hyperspectral_ = true;
	bool enable_scanner_ = true;
	bool leaf_tracing_hyperspectral_ = true;
	bool enable_arm_ = true;

	PointCloudT::Ptr chamber_occupancy_cloud_;
	std::string occu_cloud_dir_ = "C:\\Users\\lietang123\\Documents\\RoAdFiles\\LineProfilerRobotArmTest\\LineProfilerRobotArmTest\\chamber_occupancy_data\\";

	double tick_count_;

	PointCloudT::Ptr probe_points_;

	int max_samples_per_plant_ = 2;

	bool hyperspectral_topview_ = true; 

	bool enable_imaging_ = true;

	double max_plant_z_ = 0.6;

	bool upload_data_ = true;

	ChamberConfig cur_chamber_config_;

	std::string start_time_;

	bool in_charging_mode_ = false;

	float probe_patch_max_curvature_ = 0.05;

	MirClient mir_;

	//additional variables for resume work upon program start
	bool entered_mapWorkspace_ = false;
	std::string inside_or_outside_chamber_ = "outside";
	std::string imaging_or_probing_ = "imaging";
	int rover_pos_in_chamber_ = -1;
	int data_collection_done_ = 1;
	int plant_imaging_stage_ = 0;
	bool initial_enter_imagePots = true;
	std::string data_folder_ = "";
	std::string experiment_id_ = "new";
	int data_collection_mode_ = TOP_VIEW;
	double sideview_camera_x_pos_ = 0.8;

	double scheduled_minutes_per_chamber_ = 40.;

	int close_door_when_rover_inside_ = 1;

	VisionArmCombo();

	~VisionArmCombo();

	double magnitudeVec3(double * vec);

	void array6ToEigenMat4d(double * array6, Eigen::Matrix4d & mat4d);

	void eigenMat4dToArray6(Eigen::Matrix4d & mat4d, double * array6);

	void array6ToEigenMat4(double* array6, Eigen::Matrix4f & mat4);

	void initVisionCombo();
	
	void initRobotArmClient();

	void initLineProfiler();

	void initHyperspectralCam();

	void initThermoCam();

	void initRaman();

	float DDERequest(DWORD idInst, HCONV hConv, wchar_t* szItem);

	int MiniPamActPlusYield( float & Fo, float & Fm, float & FvFm, float & qP, float & qL,
		float & qN, float & NPQ, float & YNPQ, float & YNO, float & F, float & FmPrime, float & PAR,
		float & YII, float & ETR, float & FoPrime);

	int MiniPamBasicData(float & PAR, float & YII);

	void calibrateToolCenterPoint(int numPoseNeeded=4, int probe_id = RAMAN_532);

	void calibrateGripperTip(int numPoseNeeded = 4);

	void scanTranslateOnly(double * vec3d, PointCloudT::Ptr cloud, float acceleration, float speed);

	void scanMoveL(Eigen::Matrix4d & dst_scanner_pose, PointCloudT::Ptr cloud, float acceleration, float speed);

	void scanTranslateOnlyHyperspectral(double * vec3d, cv::Vec6d & start_scan_hand_pose, float acceleration, float speed, int option);

	void scanLine(PointCloudT::Ptr & cloud);

	std::string getCurrentDateTimeStr();

	void readCloudAndPoseFromFile();

	void lineScannerHandEyeCalibration(int num_lines_per_plane);

	void testLineScannerProbeCalibrationMatrix();

	void acquireLinesOnPlanes();

	void pp_callback(const pcl::visualization::PointPickingEvent& event, void*);

	int mapWorkspace(int rover_position, int option = IMAGING, int data_collection_mode = TOP_VIEW);

	void addArmModelToViewer(std::vector<PathPlanner::RefPoint> & ref_points);

	void addOBBArmModelToViewer(std::vector<PathPlanner::OBB> & arm_obbs);

	void showOccupancyGrid(bool spin=true);

	void viewPlannedPath(float* start_pose, float* goal_pose, bool only_display_start = false);

	void forward(const double* q, double* T);

	void float2double(float* array6_f, double* array6_d);

	void double2float(double* array6_d, float* array6_f);

	bool moveToConfigGetPointCloud(ArmConfig & config, int options = 0);

	double L2Norm(double* array6_1, double* array6_2);

	bool computeCollisionFreeProbeOrScanPose(PointT & point, pcl::Normal & normal, int sensor_type, std::vector<ArmConfig> & solution_config_vec, 
												Eigen::Matrix4d & scan_start_or_probe_hand_pose, Eigen::Vector3d & hand_translation);

	void getCurHandPose(Eigen::Matrix4f & pose);

	void getCurHandPoseD(Eigen::Matrix4d & pose);

	void smallClusterRemoval(PointCloudT::Ptr cloud_in, double clusterTolerance, int minClusterSize, PointCloudT::Ptr cloud_out);

	void extractLeafProbingPointsAndHyperspectralScanPoses(PointCloudT::Ptr cloud_in, 
															std::vector<pcl::PointIndices> & leaf_cluster_indices_vec,
															std::vector<std::vector<Eigen::Matrix4d*>> & hyperscan_hand_pose_sequence_vec,
															std::vector<std::vector<ArmConfig>> & hyperscan_arm_config_sequence_vec,
															std::vector<int> & hyperscan_leaf_id_vec, int plant_id =-1);

	bool probeLeaf(PointT & probe_point, pcl::Normal & normal, int probe_id=PAM, int plant_id = -1);

	void display();

	void calibrateRGBCamera(int nframes=30);

	void RGBHandEyeCalibration();

	void calibrateIRCamera(int nframes=30);

	void IRHandEyeCalibration();

	void calibrateThermalCamera();

	void ThermalHandEyeCalibration();

	int markerDetection(int rgb_or_ir, float & nearest_marker_dist, int & marker_id, float max_dist_thresh = 10000.f, bool search_zero = false);

	void cvTransformToEigenTransform(cv::Mat & cv_transform, Eigen::Matrix4d & eigen_transform);

	void EigenTransformToCVTransform(Eigen::Matrix4d & eigen_transform, cv::Mat & cv_transform);

	int scanPlantCluster(cv::Vec3f &object_center, float max_z, float radius, int plant_id=0);

	int sendRoverToChamber(int chamber_id);

	void TCPCalibrationErrorAnalysis(int numPoseNeeded = 4);

	void acquireRGBStereoPair();

	void scanLeafWithHyperspectral(std::vector<Eigen::Matrix4d*> & valid_scan_hand_pose_sequence, 
									std::vector<ArmConfig> & valid_arm_config_sequence, int plant_id = -1, int imaging_mode = TOP_VIEW);

	//communications with chamber
	void controlChamber(int chamber_id, int action);

	void hyperspectralCameraCalibration();

	void initTOFCamera();

	void initRGBCamera();

	void readOrUpdateChamberPotConfigurationFile(int operation = UPDATE_POT_CONFIG);

	int imagePots(int rover_position = 1, int option = IMAGING, int data_collection_mode = TOP_VIEW);

	void createBoxCloud(Eigen::Vector3f min, Eigen::Vector3f max, float resolution, PointCloudT::Ptr cloud);

	bool computeImageConfigforPot(cv::Vec3f & pot_xyz, float pot_diameter, cv::Mat & camera_intrinsic, 
									Eigen::Matrix4d & hand_to_camera, ArmConfig & imaging_config, 
									double & dist_to_pot, Eigen::Matrix4d & target_hand_pose, int data_collection_mode = TOP_VIEW);

	int mapPotPosition(PointCloudT::Ptr cloud_in_arm_base);

	int saveThermalImageData(int plant_id, Eigen::Matrix4d & camera_pose, cv::Mat & color_map, cv::Mat & temperature_map, int image_id = 0);

	int saveRGBImageData(int plant_id, Eigen::Matrix4d & camera_pose, cv::Mat & rgb_img, int image_id = 0);

	int saveTOFImageData(int plant_id, Eigen::Matrix4d & camera_pose, cv::Mat & ir_img_16u, cv::Mat & depth_img_16u, int image_id = 0);

	bool checkHandPoseReachable(Eigen::Matrix4d & hand_pose, ArmConfig & target_config, int option = CHECK_PATH);

	bool rotateLaserScanPot(cv::Vec3f& pot_xyz, double angular_fov, PointCloudT::Ptr cloud, std::string filename, int rover_position);

	bool lineScanPot(Eigen::Vector3d & min, Eigen::Vector3d & max, double scan_angle, float pot_diameter, PointCloudT::Ptr cloud, int option =LASER_SCANNER, std::string filename = "", int rover_position = 1);

	bool checkHandPoseReachableAlongAxis(Eigen::Matrix4d & start_hand_pose, double step, double range, Eigen::Matrix4d & result_hand_pose, std::string axis="x", int option = CHECK_PATH);

	int openOrCloseCurtain(int chamber_id, int option, int data_collection_mode = TOP_VIEW);

	int enterOrExitChamber(int chamber_id, int option);

	int getChamberConfig(int chamber_id, ChamberConfig & chamber_config);

	void solveHandEyeCalibration(std::vector<Eigen::Matrix4d*> & camera_pose_vec, std::vector<Eigen::Matrix4d*> & tcp_pose_vec, Eigen::Matrix4d & T);

	void solveLinearCameraCalibration(std::vector<std::vector<cv::Point2d>> &image_points_vec, std::vector<cv::Point3d> &corners);

	int moveArmInOrOutChamber(int option, int data_collection_mode);

	int tiltLinearScan(Eigen::Matrix4d &camera_pose,  cv::Vec6d &start_scan_pose, double angle = 50., int option = DIRECT_IMAGING);

	cv::Mat showHyperspectralImage(std::vector<cv::Mat> &scans);

	int getHyperspectralImageAtNearestReachable(cv::Vec3f & pot_xyz, float pot_diameter, cv::Vec6d &start_scan_pose);

	int getReferenceHyperspectralData(std::string filename, int rover_position, int mode);

	int loadOrUpdateChamberOccupancyData(int rover_pos, int option, PointCloudT::Ptr cur_cloud = NULL);

	void recordTime();

	void printTime(std::string msg);

	int manualMapPotPositions(int x_start = 0 , int y_start = 0);

	void run();

	void sideViewImagingRoutinePerPlant(int plant_id, double pot_y_wrt_rover = 0., int rover_pos = 1);

	void sideViewProbingRoutinePerPlant(int plant_id, double pot_y_wrt_rover, int rover_pos);

	void topViewImagingRoutinePerPlant(int rover_position, int x, int y, int plant_id, cv::Vec3f & pot_xyz);

	void topViewProbingRoutinePerPlant(int rover_position, int plant_id, cv::Vec3f & pot_xyz);

	void collectSideViewImageDataGivenPose(Eigen::Matrix4d & cam_pose, int plant_id, int image_id, int rover_pos);

	void preprocessTOFCloud(PointCloudT::Ptr cloud, Eigen::Vector4f & box_min, Eigen::Vector4f & box_max);

	bool computeSideViewProbingPose(PointT & probe_point, pcl::Normal & normal, Eigen::Matrix4d & final_probe_pose, ArmConfig & final_config, int probe_id = PAM, int plant_id = -1);

	int sideViewVerticalLaserScan(PointCloudT::Ptr cloud, Eigen::Vector3d & start_pos, Eigen::Vector3d & dst_pos, double start_tilt_down_angle, double dst_tilt_down_angle, std::string file_path = "");

	int saveProbingData(PointT & probe_point, pcl::Normal & normal, int probe_id, int plant_id);

	int saveTaskProgressToFile();
};
#endif