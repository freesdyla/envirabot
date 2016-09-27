#ifndef KINECT_THREAD_H_
#define KINECT_THREAD_H_

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <Kinect.h>
#include <thread>
#include <vector>
#include <mutex>
#include <time.h>

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

struct KinectThread
{
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;

	static const int cDepthWidth = 512;
	static const int cDepthHeight = 424;

	static const int cColorWidth = 1920;
	static const int cColorHeight = 1080;


	// data buffer for saving
	UINT16 *depthFrameBuffer;
	UINT16 *filteredDepthFrameBuffer;
	BYTE* colorFrameBuffer;

	bool nextFrameAvailable;

	// Current Kinect
	IKinectSensor*          m_pKinectSensor;

	ICoordinateMapper* pCoordinateMapper;
	
	// Depth reader
	IDepthFrameReader*      m_pDepthFrameReader;

	// Color reader
	IColorFrameReader*      m_pColorFrameReader;

	PointCloudT::Ptr m_cloud;

	//pcl::visualization::CloudViewer viewer;

	std::vector<std::thread> m_kinect_stream_handler;

	std::mutex m_updateMutex;

	bool STREAMING;

	bool FILTER_DEPTH;

	clock_t m_tic;

	KinectThread();

	~KinectThread();

	void startStream();

	void updateFrame();

	void getCurPointCloud(PointCloudT::Ptr cloud);
}; 

#endif // !KINECT_THREAD_H_



