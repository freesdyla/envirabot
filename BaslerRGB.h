#ifndef _BaslerRGB_h
#define _BaslerRGB_h

#include <pylon\PylonGUI.h>
#include <pylon/PylonIncludes.h>
#include <pylon/PylonGUI.h>
#include <iostream>
#include <thread>
#include <String>
#include <mutex>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <pylon/DeviceAccessMode.h>


using namespace Pylon;
using namespace Basler_UsbCameraParams;
using namespace GenApi;
//using namespace GenApi_3_0_Basler_pylon_v5_0;
using namespace Basler_InstantCameraParams;


typedef Pylon::CBaslerUsbInstantCamera Camera_t;

class BaslerRGB {

public:
	CBaslerUsbInstantCamera camera;
	CPylonImage image;
	boolean cameraON = false;
	boolean isPicture = false;
	CIntegerPtr param;
	gcstring param_name;
	CGrabResultPtr ptrGrabResult;
	cv::Mat imageOutMat;
	CImageFormatConverter converter;
	bool streaming_ = false;
	std::vector<std::thread> thread_vec;
	std::mutex myLock;
	const int img_width_ = 2464;
	const int img_height_ = 2056;
	const int img_size_ = img_width_*img_height_;

	BaslerRGB();
	~BaslerRGB();

	void init();
	void setBrightness(float);
	void finishCamera();
	void takePicture();
	void internalStreaming();
	cv::Mat getRGB();

private:


};

#endif