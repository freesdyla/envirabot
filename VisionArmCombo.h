#ifndef VISION_ARM_COMBO_H_
#define VISION_ARM_COMBO_H_
#define _CRT_SECURE_NO_WARNINGS
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <ctime>
#include "RobotArmClient.h" // need to be 1st due to winsock
#include "KeyenceLineProfiler.h"
#include "KinectThread.h"
#include "PathPlanner.h"

struct VisionArmCombo
{
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;

	RobotArmClient* robot_arm_client_ = NULL;

	KeyenceLineProfiler* line_profiler_ = NULL;

	KinectThread* kinect_thread_ = NULL;
	
	std::ofstream result_file_;

	std::vector<PointCloudT::Ptr> calibration_point_cloud_vec;

	Eigen::Vector3f pre_point;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

	Eigen::Matrix4f cam2hand_kinect_;



	// Distance vector of calibration plane in sensor frame
	std::vector<Eigen::Vector4d*> normalized_plane_coefficients_vec_sensor_frame_;
	std::vector<Eigen::Matrix4d*> hand_pose_vec_;
	Eigen::Matrix4f guessScannerToHand_;
	Eigen::Matrix4d initScannerToHand_;
	Eigen::Matrix4d curScannerToHand_;

	PointCloudT::Ptr kinect_cloud_;

	pcl::VoxelGrid<PointT> sor_;

	PathPlanner pp_;

	VisionArmCombo();

	double magnitudeVec3(double * vec);

	void array6ToEigenMat4d(double * array6, Eigen::Matrix4d & mat4d);

	void array6ToEigenMat4(double* array6, Eigen::Matrix4f & mat4);
	
	void initRobotArmClient();

	void initLineProfiler();

	void initKinectThread();

	int calibrateToolCenterPoint(Eigen::Vector3d & vec3d, int numPoseNeeded=4);

	void scanTranslateOnly(double * vec3d, PointCloudT::Ptr cloud);

	std::string getCurrentDateTimeStr();

	void readCloudAndPoseFromFile();

	void extractCalibrationPlaneCoefficientsSensorFrame();

	void calibrateScannerPoseSolver(int iterations);

	void acquireCalibrationPointCloud();

	void pp_callback(const pcl::visualization::PointPickingEvent& event, void*);

	void mapWorkspaceUsingKinectArm();

	void addArmModelToViewer(std::vector<PathPlanner::RefPoint> & ref_points, boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);

	void addOBBArmModelToViewer(std::vector<PathPlanner::OBB> & arm_obbs, boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);

	void showOccupancyGrid(PathPlanner & pp, boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);

	void viewPlannedPath(PathPlanner & pp, boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer, float* start_pose, float* goal_pose);

};


#endif
