#ifndef HYPERSPECTRAL_CAMERA_H
#define HYPERSPECTRAL_CAMERA_H

#include <windows.h>
#include "SI_sensor.h"
#include "SI_errors.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#if 0
#include <PvSampleUtils.h>
#include <PvSystem.h>
#include <PvInterface.h>
#include <PvDevice.h>
#endif

#include <iostream>
#include <vector>
#include <mutex>
#include <atomic>

#define LICENSE_PATH L"C:/Users/Public/Documents/Specim/SpecSensor.lic"

#define CALIBRATION 0
#define DATA_COLLECTION 1

using namespace std;

struct HyperspectralCamera {

	int nError = siNoError;

	int nDeviceIndex_ = 1;

	SI_H g_hDevice_ = 0;

	static std::vector<std::vector<unsigned char>> frames_;

	static std::vector<cv::Mat> scanlines_;

	double dFrameRate_;

	double dExposureTime_;

	double* pWavelengths;

	static int spatial_binning_;

	static int spectral_binning_;

	static int spatial_size_;

	static int spectral_size_;
	
	static int img_size_;

	static cv::Mat img_;

	static cv::Mat img_8u_;

	static cv::Mat img_8uc3_;

	static std::atomic<unsigned int> frame_count_;

	int frame_data_size_;

	static std::mutex update_mutex_;

//	PvResult lResult;

//	PvSystem lSystem;

//	const PvDeviceInfo* lLastDeviceInfo = NULL;

	HyperspectralCamera();
	~HyperspectralCamera();
	
	void init();

	void start(int options = DATA_COLLECTION);

	void stop();

	static int onDataCallback(SI_U8* _pBuffer, SI_64 _nFrameSize, SI_64 _nFrameNumber, void* _pContext);

	static int onDataCallbackAverageSpectral(SI_U8* _pBuffer, SI_64 _nFrameSize, SI_64 _nFrameNumber, void* _pContext);

	cv::Mat getLastestFrame();

};


#endif
