#ifndef TOF_SWIFT_H_
#define TOF_SWIFT_H_
#define _CRT_SECURE_NO_WARNINGS
#include "swift/SwiftLib.h"
#include <iostream>
#include <fstream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include "utilities.h"

class TOF_Swift {

	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;
	
public:
	odos::Camera cam_;
	int connectedCams = 0;
	int k = 0; // Index for selected camera
	odos::IImage*  image;
	cv::Mat iocv;
	cv::Mat intensity_img_;
	cv::Mat intensity_img_16u_;
	cv::Mat range_filtered_;
	cv::Mat range_raw_16u_;
	const int img_width_ = 640;
	const int img_height_ = 480;
	const int img_size_ = img_width_*img_height_;	
	
	const float *xMultiplier = nullptr;
	const float *yMultiplier = nullptr;
	const float *zMultiplier = nullptr;

	std::string ip_address = "169.254.4.17";
	std::vector<std::thread> thread_vec_;
	std::atomic<bool> streaming_;
	std::atomic<int64> last_frame_tick_count_;

	PointCloudT::Ptr cloud_;

	std::mutex update_mutex_;

	const int wait_image_timeout_sec_ = 10;

	TOF_Swift();
	~TOF_Swift();

	void getAsImage(odos::Camera& camera, const float*& xMultiplier, const float*& yMultiplier, const float*& zMultiplier);

	float* getImage(odos::Camera& camera, odos::ComponentType component);
		 
	void startStreaming();
	void setPower(int power);

	void start();
	int stop();

	void listNodes();
	void setupStreaming();
	int seeNodeChilds(odos::CategoryNode * node, std::string prefix);
	void getPointCloud(PointCloudT::Ptr cloud);
	cv::Mat getIR();

	cv::Mat getIR16U();

	cv::Mat getDepth16U();

	void waitTillLastestFrameArrive();
 };

#endif // !TOF_SWIFT_H_