#include "BaslerRGB.h"

BaslerRGB::BaslerRGB()
{
	imageOutMat.create(img_height_, img_width_, CV_8UC3);
	converter.OutputPixelFormat = PixelType_BGR8packed;

	PylonInitialize();

	init();
}

BaslerRGB::~BaslerRGB()
{
	finishCamera();
}

void BaslerRGB::init()
{
	CDeviceInfo info;
	info.SetDeviceClass(Pylon::CBaslerUsbInstantCamera::DeviceClass());
	camera.Attach(CTlFactory::GetInstance().CreateFirstDevice(info));
	camera.RegisterConfiguration(new CAcquireContinuousConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete); // Not sure if necesary.																												 // Print the model name of the camera.
	//std::cout << "Using device " << camera.GetDeviceInfo().GetModelName() << std::endl;

	INodeMap& nodemap = camera.GetNodeMap();

	camera.Open();

	CIntegerPtr offsetX(nodemap.GetNode("OffsetX"));
	CIntegerPtr offsetY(nodemap.GetNode("OffsetY"));

	offsetX->SetValue(0);
	offsetY->SetValue(0);

	CIntegerPtr width(nodemap.GetNode("Width"));
	CIntegerPtr height(nodemap.GetNode("Height"));

	width->SetValue(img_width_);
	height->SetValue(img_height_);

	CBooleanPtr flip_x(nodemap.GetNode("ReverseX"));
	flip_x->SetValue(true);

	CBooleanPtr flip_y(nodemap.GetNode("ReverseY"));
	flip_y->SetValue(true);


	// Access the PixelFormat enumeration type node.
	CEnumerationPtr pixelFormat(nodemap.GetNode("PixelFormat"));

	if (IsAvailable(pixelFormat->GetEntryByName("BGR8")))
		pixelFormat->FromString("BGR8");

	CEnumerationPtr gainAuto(nodemap.GetNode("GainAuto"));

	//	gainAuto->FromString("On");

	thread_vec.push_back(std::thread(&BaslerRGB::internalStreaming, this));

}

void BaslerRGB::finishCamera()
{
	streaming_ = false;
	thread_vec[0].join();
	camera.Open();
	camera.DeviceReset.Execute();
	// Wait a little 
	Sleep(500);
	camera.DetachDevice();
	Sleep(100);
	PylonTerminate();
	// If you dont detach the camera the PylonTerminate cant access the attached camera in the instance and will trow some nasty error
}

void BaslerRGB::setBrightness(float val)
{
	camera.AutoFunctionROISelector.SetValue(AutoFunctionROISelector_ROI1);
	camera.AutoFunctionROIUseBrightness.SetValue(true);
	camera.AutoFunctionROIOffsetX.SetValue(0);
	camera.AutoFunctionROIOffsetY.SetValue(0);
	camera.AutoFunctionROIWidth.SetValue(camera.WidthMax.GetValue());
	camera.AutoFunctionROIHeight.SetValue(camera.HeightMax.GetValue());
	camera.AutoFunctionROIOffsetX.SetValue(0);
	camera.AutoFunctionROIOffsetY.SetValue(0);
	camera.AutoFunctionROIWidth.SetValue(camera.AutoFunctionROIWidth.GetMax());
	camera.AutoFunctionROIHeight.SetValue(camera.AutoFunctionROIHeight.GetMax());
	camera.AutoExposureTimeLowerLimit.SetValue(1000.0);
	camera.AutoExposureTimeUpperLimit.SetValue(50000.0);
	camera.AutoTargetBrightness.SetValue(val);
	camera.ExposureAuto.SetValue(ExposureAuto_Continuous);
}

void BaslerRGB::internalStreaming()
{
	streaming_ = true;
	setBrightness(0.40); //Set brightness to initial desired level
	Sleep(100);
	camera.StartGrabbing(GrabStrategy_LatestImageOnly);

	while (streaming_) 
	{ 
		takePicture();
		Sleep(20);
	} //Refresh imageOut in BGR8-packed format

	camera.StopGrabbing();
}

void BaslerRGB::takePicture()
{
	try {
		if (camera.IsGrabbing()) {
			camera.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_Return);
			if (ptrGrabResult->GrabSucceeded()) {
			
				image.AttachGrabResultBuffer(ptrGrabResult);
				
				//Change image type if not BGR
				myLock.lock();

				std::memcpy((unsigned char*)imageOutMat.ptr<unsigned char>(0), (unsigned char*)image.GetBuffer(), img_size_ * 3);

				myLock.unlock();
			}
		}
	}
	catch (...) { std::cout << "An exception occurred: Probably it was a Timeout Exception, sometimes exposure time will be longer than timeout threshold, consider increase camera aperture or light intensity" << std::endl; }
}

cv::Mat BaslerRGB::getRGB() {

	cv::Mat copy;
	myLock.lock();
	copy = imageOutMat.clone();
	myLock.unlock();

	return copy;
}
