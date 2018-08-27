#ifndef UTILITIES_H_
#define UTILITIES_H_
#define _CRT_SECURE_NO_WARNINGS
#include <fstream>
#include <iostream>
#include <iomanip>

//#define USE_PCL

#ifdef USE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#endif

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/algorithm/string.hpp>

#define THERMO_WIDTH 640
#define THERMO_HEIGHT 480
#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480
#define RGB_WIDTH 2464
#define RGB_HEIGHT 2056

struct Utilities
{
#ifdef USE_PCL
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;
#endif

	static std::string getCurrentDateTimeStr()
	{
		SYSTEMTIME st;

		GetLocalTime(&st);

		//GetSystemTime(&st);

		char currentTime[84] = "";

		sprintf_s(currentTime, "%d_%d_%d_%d_%d_%d_%d", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);

		return std::string(currentTime);
	}

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
		std::string timestamp = getCurrentDateTimeStr();
		log_file << timestamp << ": " << text << std::endl;
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

	/*static Eigen::Matrix4d array6ToEigenMat4d(double* array6)
	{
		Eigen::Matrix4d mat4d = Eigen::Matrix4d::Identity();
		mat4d.col(3).head(3) << array6[3], array6[4], array6[5];
		double angle = mat4d.col(3).head(3).norm();
		Eigen::Vector3d axis(array6[3] / angle, array6[4] / angle, array6[5] / angle);
		Eigen::Matrix3d rotationMatrix;
		rotationMatrix = Eigen::AngleAxisd(angle, axis);
		mat4d.block<3, 3>(0, 0) = rotationMatrix;
		return mat4d;
	}*/

	static int readHyperspectralImage(std::string filename, std::vector<cv::Mat> & frames)
	{
		frames.clear();

		std::vector<std::string> str_vec;

		boost::split(str_vec, filename, boost::is_any_of("\\"));

		std::string tmp = str_vec.back();

		str_vec.clear();

		boost::split(str_vec, tmp, boost::is_any_of("."));

		if (str_vec.size() != 2)
			return -1;

		if (str_vec.back() != "bin")
			return -2;

		tmp = str_vec.front();

		str_vec.clear();

		boost::split(str_vec, tmp, boost::is_any_of("_"));

		int width, height, num_frames;
		
		for (int i = 0; i < str_vec.size(); i++)
		{
			if (str_vec[i] == "f")
				num_frames = std::stoi(str_vec[i + 1]);
			else if (str_vec[i] == "w")
				width = std::stoi(str_vec[i + 1]);
			else if(str_vec[i] == "h")
				height = std::stoi(str_vec[i + 1]);
		}

		const int img_size = width*height;

		std::ifstream in(filename, std::ios::in | std::ios::binary);

		if (!in.is_open())
			return -3;

		for (int i = 0; i < num_frames; i++)
		{
			cv::Mat img; img.create(height, width, CV_16U);

			in.read((char*)img.data, img_size * sizeof(UINT16));

			frames.push_back(img);
		}

		in.close();

		return 0;
	}

	static int readTimestampHandPose(std::string filename, std::vector<cv::Vec6d> & pose_vec, std::vector<std::int64_t> & timestamp_vec)
	{
		std::ifstream in(filename, std::ios::in);

		if (!in.is_open()) return -1;

		std::string line;

		while (std::getline(in, line)) {
			
			std::vector<std::string> str_vec;

			boost::split(str_vec, line, boost::is_any_of(","));

			if (str_vec.size() == 7)
			{
				std::int64_t timestamp = std::stoll(str_vec[0]);

				//std::cout << str_vec[0] << " " << timestamp << std::endl;

				cv::Vec6d pose;

				for (int i = 1; i < 7; i++)
					pose[i-1] = std::stod(str_vec[i]);

				
				//Eigen::Matrix4d eigen_pose = array6ToEigenMat4d(&pose.val[0]);

				pose_vec.push_back(pose);
				timestamp_vec.push_back(timestamp);
			}
		}

		return 0;
	}

	static int readTimestamp(std::string filename, std::vector<std::int64_t> & timestamp_vec)
	{
		std::ifstream in(filename, std::ios::in);

		if (!in.is_open()) return -1;

		std::string line;

		while (std::getline(in, line)) {

			std::int64_t timestamp = std::stoll(line);

			timestamp_vec.push_back(timestamp);
		}

		return 0;
	}

	static int reshapeHyperspectralData(std::vector<cv::Mat> & raw_frames, std::vector<cv::Mat> & spectrum_2d_frames)
	{
		const int num_scanlines = raw_frames.size();

		if (num_scanlines == 0) return -1;

		const int num_spectrums = raw_frames.front().rows;

		const int width = raw_frames.front().cols;

		spectrum_2d_frames.resize(num_spectrums);

		for (int i = 0; i < num_spectrums; i++)
		{
			cv::Mat image; image.create(num_scanlines, width, CV_16U);

			for (int j = 0; j < num_scanlines; j++)
			{
				raw_frames[j].row(i).copyTo(image.row(j));
			}

			double min_val, max_val;

			cv::minMaxLoc(image, &min_val, &max_val);

			//std::cout << "min " << min_val << "  max " << max_val << std::endl;
			///(v - min)/(max-min)*255 = v*(255/(max-min)) - 255*min/(max-min)
			//min_val = 170.;

			cv::Mat image_8u, image_resize;
			image.convertTo(image_8u, CV_8U, 255. / (max_val - min_val), -255 * min_val / (max_val - min_val));

			cv::resize(image_8u, image_resize, cv::Size(), 0.5, 0.5);

			std::string name = std::to_string(i);
			cv::imshow(name, image_resize);
			cv::waitKey(0);
			cv::destroyAllWindows();

			spectrum_2d_frames.push_back(image);
		}



		return 0;
	}
};



#endif
