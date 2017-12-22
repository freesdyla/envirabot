#include "HyperspectralCamera.h"

std::vector<std::vector<unsigned char>> HyperspectralCamera::frames_;

cv::Mat HyperspectralCamera::img_;

cv::Mat HyperspectralCamera::img_8u_;

cv::Mat HyperspectralCamera::img_8uc3_;

int HyperspectralCamera::spatial_binning_ = 0;

int HyperspectralCamera::spectral_binning_ = 0;

int HyperspectralCamera::spatial_size_ = 1024 << spatial_binning_;

int HyperspectralCamera::spectral_size_ = 448 << spectral_binning_;

int HyperspectralCamera::img_size_ = spatial_size_*spectral_size_;

unsigned int HyperspectralCamera::frame_count_ = 0;

std::mutex HyperspectralCamera::update_mutex_;

HyperspectralCamera::HyperspectralCamera() {
	img_.create(spectral_size_, spatial_size_, CV_16U);
	img_8u_.create(spectral_size_, spatial_size_, CV_8U);
	img_8uc3_.create(spectral_size_, spatial_size_, CV_8UC3);
	frame_data_size_ = spectral_size_*spatial_size_ * 2;
}

HyperspectralCamera::~HyperspectralCamera() {
	delete[] pWavelengths;
}

void HyperspectralCamera::init() {

	std::cout << "expected bytes: " <<spectral_size_*spatial_size_ * 2 << "\n";

	SI_CHK(SI_Load(LICENSE_PATH));

	// Open the device and set the callbacks
	SI_CHK(SI_Open(nDeviceIndex_, &g_hDevice_));

	// Select a device
	SI_WC sPath[] = L"C:/Users/lietang123/Documents/RoAdFiles/Hyperspectral/Calibrations/3200031_20170809_FX_calpack.scp";

	SI_CHK(SI_SetString(g_hDevice_, L"Camera.CalibrationPack", sPath));


	SI_CHK(SI_Command(g_hDevice_, L"Initialize"));


	bool bIsLoaded = false;
	SI_CHK(SI_GetBool(g_hDevice_, L"Camera.CalibrationPack.IsLoaded", &bIsLoaded));	cout << "is calib loaded: " << bIsLoaded << "\n";
	dFrameRate_ = 10.0;

	SI_SetEnumIndex(g_hDevice_, L"Camera.Binning.Spatial", spatial_binning_); 

	SI_SetEnumIndex(g_hDevice_, L"Camera.Binning.Spectral", spectral_binning_);

	SI_CHK(SI_SetFloat(g_hDevice_, L"Camera.FrameRate", dFrameRate_));

//	SI_CHK(SI_GetFloat(g_hDevice_, L"Camera.FrameRate", &dFrameRate_));
		dExposureTime_ = 10.0;	
	SI_CHK(SI_SetFloat(g_hDevice_, L"Camera.ExposureTime", dExposureTime_));

	//SI_CHK(SI_GetFloat(g_hDevice_, L"Camera.ExposureTime", &dExposureTime_));

	std::cout << "exposure time " << dExposureTime_ << "		frame rate "<<dFrameRate_<<"\n";


	//SI_CHK(SI_RegisterFeatureCallback(g_hDevice, L"Camera.FrameRate", FeatureCallback1, 0));
	//SI_CHK(SI_RegisterFeatureCallback(g_hDevice, L"Camera.ExposureTime", FeatureCallback1, 0));
	//SI_CHK(SI_RegisterFeatureCallback(g_hDevice, L"Camera.ExposureTime", FeatureCallback2, 0));
	SI_CHK(SI_RegisterDataCallback(g_hDevice_, onDataCallback, 0));

#if 0
	int nCount = 0;

	SI_CHK(SI_GetEnumCount(g_hDevice_, L"Camera.WavelengthTable", &nCount));

	std::cout << "wave length count " << nCount << "\n";

	SI_WC wcWavelength[100];
	pWavelengths = new double[nCount];

	for (int n = 0; n < nCount; n++)
	{
		SI_CHK(SI_GetEnumStringByIndex(g_hDevice_, L"Camera.WavelengthTable", n, wcWavelength, 100));
		pWavelengths[n] = wcstod(wcWavelength, 0);
		std::cout << pWavelengths[n] << " ";
	}
	std::cout << "\n"; 
#endif
	return;

Error:

	if (SI_FAILED(nError))
		wprintf(L"An error occurred: %s\n", SI_GetErrorString(nError));

	SI_Close(g_hDevice_);
	SI_Unload();
}

void HyperspectralCamera::start() {

	SI_CHK(SI_Command(g_hDevice_, L"Acquisition.Start"));

	return;

Error:

	if (SI_FAILED(nError))
		wprintf(L"An error occurred: %s\n", SI_GetErrorString(nError));

	SI_Close(g_hDevice_);
	SI_Unload();
}

void HyperspectralCamera::stop() {

	SI_CHK(SI_Command(g_hDevice_, L"Acquisition.Stop"));

	return;

Error:

	if (SI_FAILED(nError))
	{
		wprintf(L"An error occurred: %s\n", SI_GetErrorString(nError));
	}

	SI_Close(g_hDevice_);
	SI_Unload();
}


int HyperspectralCamera::onDataCallback(SI_U8* _pBuffer, SI_64 _nFrameSize, SI_64 _nFrameNumber, void* _pContext) {

	frame_count_ += _nFrameNumber;

//	std::vector<unsigned char> frame(_nFrameSize);

//	std::memcpy(frame.data(), _pBuffer, _nFrameSize);

//	frames_.push_back(frame);

	update_mutex_.lock();

	std::memcpy(img_.ptr<unsigned char>(), _pBuffer, _nFrameSize);

	update_mutex_.unlock();

#if 0
	//std::cout << _nFrameSize << " " << _nFrameNumber <<"\n";
		
	std::memcpy(img_.ptr<unsigned char>(), _pBuffer, _nFrameSize);
	
	double min_val, max_val;

	cv::minMaxLoc(img_, &min_val, &max_val);

	///(v - min)/(max-min)*255 = v*(255/(max-min)) - 255*min/(max-min)

	min_val = 170.;

	img_.convertTo(img_8u_, CV_8U, 255./(max_val-min_val), -255.*min_val/(max_val-min_val));

	//cout << min_val << " " << max_val << "\n";

	cv::imshow("img", img_8u_);

	cv::waitKey(10);
#endif

	return 0;
}

cv::Mat HyperspectralCamera::getLastestFrame() {

	cv::Mat lastest_frame_;

	update_mutex_.lock();

	lastest_frame_ = img_.clone();

	update_mutex_.unlock();

	return lastest_frame_;
}