#include "HyperspectralCamera.h"

std::vector<std::vector<unsigned char>> HyperspectralCamera::frames_;

std::vector<cv::Mat> HyperspectralCamera::scanlines_;

std::vector<LARGE_INTEGER> HyperspectralCamera::timestamps_;

cv::Mat HyperspectralCamera::last_img_;

cv::Mat HyperspectralCamera::img_8u_;

cv::Mat HyperspectralCamera::img_8uc3_;

int HyperspectralCamera::spatial_binning_ = 0;

int HyperspectralCamera::spectral_binning_ = 3;

int HyperspectralCamera::spatial_size_ = 1024 >> spatial_binning_;

int HyperspectralCamera::spectral_size_ = 448 >> spectral_binning_;

int HyperspectralCamera::img_size_ = spatial_size_*spectral_size_;

std::atomic<unsigned int> HyperspectralCamera::frame_count_ = 0;

std::mutex HyperspectralCamera::update_mutex_;

HyperspectralCamera::HyperspectralCamera() 
{
	last_img_.create(spectral_size_, spatial_size_, CV_16U);
	img_8u_.create(spectral_size_, spatial_size_, CV_8U);
	img_8uc3_.create(spectral_size_, spatial_size_, CV_8UC3);
	frame_data_size_ = spectral_size_*spatial_size_ * 2;
}

HyperspectralCamera::~HyperspectralCamera() 
{
	stop();
	SI_Close(g_hDevice_);
	SI_Unload();
	delete[] pWavelengths;
}

void HyperspectralCamera::init() {

	SI_CHK(SI_Load(LICENSE_PATH));

	// Open the device and set the callbacks
	SI_CHK(SI_Open(nDeviceIndex_, &g_hDevice_));

	// Select a device
	SI_WC sPath[] = L"C:/Users/lietang123/Documents/RoAdFiles/Hyperspectral/Calibrations/3200031_20170809_FX_calpack.scp";

	SI_CHK(SI_SetString(g_hDevice_, L"Camera.CalibrationPack", sPath));

	SI_CHK(SI_Command(g_hDevice_, L"Initialize"));

	bool bIsLoaded = false;

	SI_CHK(SI_GetBool(g_hDevice_, L"Camera.CalibrationPack.IsLoaded", &bIsLoaded));	std::cout << "hyperspectral ready: " << bIsLoaded << std::endl;

	SI_CHK(SI_SetEnumIndex(g_hDevice_, L"Camera.Binning.Spatial", spatial_binning_));

	SI_CHK(SI_SetEnumIndex(g_hDevice_, L"Camera.Binning.Spectral", spectral_binning_));

	dFrameRate_ = 30.0;

	SI_CHK(SI_SetFloat(g_hDevice_, L"Camera.FrameRate", dFrameRate_));
		dExposureTime_ = 30.0;	
	SI_CHK(SI_SetFloat(g_hDevice_, L"Camera.ExposureTime", dExposureTime_));

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
		std::cout <<pWavelengths[n] << " ";
	}
	std::cout << "\n"; 
#endif
	return;

Error:

	if (SI_FAILED(nError))
		wprintf(L"An error occurred: %s\n", SI_GetErrorString(nError));

	Utilities::to_log_file("Hyperspectral camera init fail");

	SI_Close(g_hDevice_);
	SI_Unload();

	exit(0);
	return;
}

int HyperspectralCamera::start(int options, bool open_shutter) {

	if (options != CONTINUOUS && options != LAST_FRAME)
		return -2;

	frame_count_ = 0;

	if(open_shutter)
		SI_CHK(SI_Command(g_hDevice_, L"Camera.OpenShutter"));
	else
		SI_CHK(SI_Command(g_hDevice_, L"Camera.CloseShutter"));

	if (options == LAST_FRAME)
	{
		dFrameRate_ = 30.;
		//SI_CHK(SI_SetEnumIndex(g_hDevice_, L"Camera.Binning.Spatial", 0));
		//SI_CHK(SI_SetEnumIndex(g_hDevice_, L"Camera.Binning.Spectral", 0));	// 448
		//SI_CHK(SI_SetFloat(g_hDevice_, L"Camera.ExposureTime", 30.));
		//SI_CHK(SI_SetFloat(g_hDevice_, L"Camera.FrameRate", dFrameRate_));
		SI_CHK(SI_RegisterDataCallback(g_hDevice_, onDataCallbackLastFrame, 0));

		SI_CHK(SI_GetFloat(g_hDevice_, L"Camera.FrameRate", &dFrameRate_));
		SI_CHK(SI_GetFloat(g_hDevice_, L"Camera.ExposureTime", &dExposureTime_));
		std::cout << "exposure time " << dExposureTime_ << "		frame rate " << dFrameRate_ << "\n";
		//std::cout << "ready\n";	std::getchar();
	}
	else if(options == CONTINUOUS)
	{
		scanlines_.clear();
		timestamps_.clear();
		dFrameRate_ = 30.;

		//SI_CHK(SI_SetEnumIndex(g_hDevice_, L"Camera.Binning.Spatial", 0));
		//SI_CHK(SI_SetEnumIndex(g_hDevice_, L"Camera.Binning.Spectral", 3));	// 448/8=56
		//SI_CHK(SI_SetFloat(g_hDevice_, L"Camera.ExposureTime", 30.));
		//SI_CHK(SI_SetFloat(g_hDevice_, L"Camera.FrameRate", dFrameRate_));
		SI_CHK(SI_RegisterDataCallback(g_hDevice_, onDataCallback, 0));
		SI_CHK(SI_GetFloat(g_hDevice_, L"Camera.FrameRate", &dFrameRate_));
		SI_CHK(SI_GetFloat(g_hDevice_, L"Camera.ExposureTime", &dExposureTime_));
		std::cout << "exposure time " << dExposureTime_ << "		frame rate " << dFrameRate_ << "\n";
	}

	SI_CHK(SI_Command(g_hDevice_, L"Acquisition.Start"));

	return 0;

Error:

	if (SI_FAILED(nError))
		wprintf(L"An error occurred in start: %s\n", SI_GetErrorString(nError));

	//SI_Close(g_hDevice_);
	//SI_Unload();

	return -1;
}

void HyperspectralCamera::stop() {

	SI_CHK(SI_Command(g_hDevice_, L"Acquisition.Stop"));

	return;

Error:

	if (SI_FAILED(nError))
	{
		wprintf(L"An error occurred in stop: %s\n", SI_GetErrorString(nError));
	}

//	SI_Close(g_hDevice_);
//	SI_Unload();
}


int HyperspectralCamera::onDataCallback(SI_U8* _pBuffer, SI_64 _nFrameSize, SI_64 _nFrameNumber, void* _pContext) {

	frame_count_.store(frame_count_.load() + _nFrameNumber);

	LARGE_INTEGER timestamp;
	QueryPerformanceCounter(&timestamp);

	for (int i = 0; i < _nFrameNumber; i++)
	{
		cv::Mat tmp; tmp.create(spectral_size_, spatial_size_, CV_16U);
		std::memcpy(tmp.data, _pBuffer + i*_nFrameSize, _nFrameSize);
		scanlines_.push_back(tmp);
		timestamps_.push_back(timestamp);
	}
	
	return 0;
}

int HyperspectralCamera::onDataCallbackLastFrame(SI_U8* _pBuffer, SI_64 _nFrameSize, SI_64 _nFrameNumber, void* _pContext)
{
	frame_count_.store(frame_count_.load() + _nFrameNumber);

	update_mutex_.lock();
	for (int i = 0; i < _nFrameNumber; i++)
	{
		std::memcpy(last_img_.data, _pBuffer + i*_nFrameSize, _nFrameSize);
	}
	update_mutex_.unlock();

	return 0;
}

int HyperspectralCamera::onDataCallbackAverageSpectral(SI_U8* _pBuffer, SI_64 _nFrameSize, SI_64 _nFrameNumber, void* _pContext)
{
	frame_count_.store(frame_count_.load() + _nFrameNumber);

	//std::cout << _nFrameSize << std::endl;
	//if (_nFrameNumber != 1) std::cout << "frame != 1\n";

	//int64 tick = cv::getTickCount();

	for (int i = 0; i < _nFrameNumber; i++)
	{
		cv::Mat tmp(56, 1024, CV_16U, _pBuffer + i*_nFrameSize);
		cv::Mat scan_line;
		cv::reduce(tmp, scan_line, 0, CV_REDUCE_AVG, CV_64F);
		scanlines_.push_back(scan_line);
	}

	//std::cout << (cv::getTickCount() - tick) / cv::getTickFrequency() << "\n";

	return 0;
}

cv::Mat HyperspectralCamera::getLastestFrame() {

	cv::Mat lastest_frame_; lastest_frame_.create(spectral_size_, spatial_size_, CV_16U);

	update_mutex_.lock();

	std::memcpy(lastest_frame_.data, last_img_.data, frame_data_size_);

	update_mutex_.unlock();

	return lastest_frame_;
}

int HyperspectralCamera::saveData(std::string filename)
{
	if (scanlines_.size() != timestamps_.size())
	{
		return -1;
	}

	if (scanlines_.size() == 0)
	{
		return -2;
	}

	const int height = scanlines_.front().rows;

	const int width = scanlines_.front().cols;

	const int img_size = height*width;

	const int num_frames = scanlines_.size();

	std::string timestamp_filename = filename + ".csv";
	std::string image_filename = filename + "_f_"+ std::to_string(num_frames) + "_w_"+std::to_string(width)+"_h_"+std::to_string(height) + ".bin";

	std::ofstream out(image_filename, std::ios::out | std::ios::binary);

	std::ofstream out_ts(timestamp_filename, std::ios::out);

	if (!out_ts.is_open() || !out.is_open())
		return -3;

	for (int i = 0; i < num_frames; i++)
	{
		out.write((char*)scanlines_[i].data, img_size* sizeof(unsigned short));

		out_ts << timestamps_[i].QuadPart;
		
		if(i != num_frames - 1) out_ts << std::endl;
	}

	out.close();
	out_ts.close();
		
	return 0;
}

std::string HyperspectralCamera::getCurrentDateTimeStr()
{
	SYSTEMTIME st;

	GetLocalTime(&st);

	//GetSystemTime(&st);

	char currentTime[84] = "";

	std::sprintf(currentTime, "%d_%d_%d_%d_%d_%d_%d", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);

	return std::string(currentTime);
}

void HyperspectralCamera::dropBufferedFrames()
{
	SI_CHK(SI_Command(g_hDevice_, L"Acquisition.RingBuffer.Sync"));

	return;

Error:

	if (SI_FAILED(nError))
	{
		wprintf(L"An error occurred in dropBufferedFrames: %s\n", SI_GetErrorString(nError));
	}
}
