#ifndef UTILITIES_H_
#define UTILITIES_H_
#include <fstream>
#include <iostream>

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define THERMO_WIDTH 640
#define THERMO_HEIGHT 480
#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480
#define RGB_WIDTH 2464
#define RGB_HEIGHT 2056

struct Utilities
{
//	typedef pcl::PointXYZRGB PointT;
//	typedef pcl::PointCloud<PointT> PointCloudT;

	static void CallBackFunc(int event, int x, int y, int flags, void* userdata)
	{
		if (event == cv::EVENT_LBUTTONDOWN)
		{
			cv::Mat* image = (cv::Mat*) userdata;

			std::cout << "Temp: "<< image->at<double>(y,x)<<"  "<< x << ", " << y << std::endl;
		}
	
	}


	static void to_log_file(const std::string &text)
	{
		std::ofstream log_file ("log_file.txt", std::ios_base::out | std::ios_base::app);
		log_file << text << std::endl;
		log_file.close();
	}

	static int gridID2PotID(int x, int y, int rows) {	return x*rows + rows - y; }

	// return temperature map
	static cv::Mat readThermoImage(std::string path)
	{
		cv::Mat temperature_map;
		temperature_map.create(THERMO_HEIGHT, THERMO_WIDTH, CV_64F);

		std::ifstream in(path, std::ios::in | std::ios::binary);

		if (in.is_open())
		{
			in.read((char*)temperature_map.data, THERMO_WIDTH * THERMO_HEIGHT * sizeof(double));

			in.close();
		}

		cv::Mat normalized;

		cv::normalize(temperature_map, normalized, 1.0, 0, cv::NORM_MINMAX);

		cv::imshow("thermo", normalized);

		cv::setMouseCallback("thermo", CallBackFunc, (void*) &temperature_map);

		cv::waitKey(0);

		return temperature_map;
	}

	static void readTOFImage(std::string ir_path, cv::Mat &ir_img, std::string depth_path, cv::Mat &depth_img)
	{
		ir_img.create(DEPTH_HEIGHT, DEPTH_WIDTH, CV_16U);
		depth_img.create(DEPTH_HEIGHT, DEPTH_WIDTH, CV_16U);

		std::ifstream in(ir_path, std::ios::in | std::ios::binary);

		if (in.is_open())
		{
			in.read((char*)ir_img.data, DEPTH_HEIGHT * DEPTH_WIDTH * sizeof(UINT16));

			in.close();
		}

		in.open(depth_path, std::ios::in | std::ios::binary);
		if (in.is_open())
		{
			in.read((char*)depth_img.data, DEPTH_HEIGHT * DEPTH_WIDTH * sizeof(UINT16));

			in.close();
		}

		cv::Mat ir_8u, depth_8u;
		ir_img.convertTo(ir_8u, CV_8U, 255 / 20000.);

		cv::imshow("ir", ir_8u);

		depth_img.convertTo(depth_8u, CV_8U, 255 / 10000.);

		cv::imshow("depth", depth_8u);

		cv::waitKey(0);
#if 0
		cv::Mat xm, ym, zm;
		xm.create(DEPTH_HEIGHT, DEPTH_WIDTH, CV_32F);
		ym.create(DEPTH_HEIGHT, DEPTH_WIDTH, CV_32F);
		zm.create(DEPTH_HEIGHT, DEPTH_WIDTH, CV_32F);

		in.open("odos_x_multiplier_32f.bin", std::ios::in | std::ios::binary);
		if (in.is_open())
		{
			in.read((char*)xm.data, DEPTH_HEIGHT * DEPTH_WIDTH * sizeof(float));
			in.close();
		}
		
		in.open("odos_y_multiplier_32f.bin", std::ios::in | std::ios::binary);
		if (in.is_open())
		{
			in.read((char*)ym.data, DEPTH_HEIGHT * DEPTH_WIDTH * sizeof(float));
			in.close();
		}
		
		in.open("odos_z_multiplier_32f.bin", std::ios::in | std::ios::binary);
		if (in.is_open())
		{
			in.read((char*)zm.data, DEPTH_HEIGHT * DEPTH_WIDTH * sizeof(float));
			in.close();
		}

		PointCloudT::Ptr cloud(new PointCloudT);
		cv::Mat range_m;
		depth_img.convertTo(range_m, CV_32FC1, 0.0001, 0.);

		float* range_ptr = range_m.ptr<float>(0);
		float* xm_ptr = xm.ptr<float>(0);
		float* ym_ptr = ym.ptr<float>(0);
		float* zm_ptr = zm.ptr<float>(0);

		for (int i = 0; i < DEPTH_HEIGHT * DEPTH_WIDTH; ++i)
		{
			const float range = (*range_ptr);
			const float z = range*(*zm_ptr);

			if (z > 0.01f && z < 3.f) {

				PointT p;
				p.z = z;
				p.x = -(*xm_ptr)*range;
				p.y = -(*ym_ptr)*range;
				p.r = 255;
				p.g = 255;
				p.b = 255;

				cloud->push_back(p);
			}

			++range_ptr;
			++xm_ptr;
			++ym_ptr;
			++zm_ptr;
		}
#endif

	}
};



#endif
