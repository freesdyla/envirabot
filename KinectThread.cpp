#include "KinectThread.h"

KinectThread::KinectThread() : 
	//viewer("3D Viewer"),
	m_pKinectSensor(NULL),
	m_pDepthFrameReader(NULL),
	nextFrameAvailable(false),
	STREAMING(false),
	FILTER_DEPTH(true)
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);

	if (FAILED(hr))
	{
		std::cout << "can not get default sensor\n";
		return;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get the depth reader
		IDepthFrameSource* pDepthFrameSource = NULL;

		// Initialize the Kinect and get the color reader
		IColorFrameSource* pColorFrameSource = NULL;

		IInfraredFrameSource* pInfraredFrameSource = NULL;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&pCoordinateMapper);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_InfraredFrameSource(&pInfraredFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pInfraredFrameSource->OpenReader(&m_pInfraredFrameReader);
		}

		SafeRelease(pDepthFrameSource);
		SafeRelease(pColorFrameSource);
		SafeRelease(pInfraredFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		std::cout<<"No ready Kinect found!\n";
		return;
	}

	m_cloud.reset(new PointCloudT);
	m_cloud->is_dense = true;

	m_tic = clock();

	stream_infrared_ = true;

	depthFrameBuffer = new UINT16[cDepthSize];

	filteredDepthFrameBuffer = new UINT16[cDepthSize];

	infraredFrameBuffer = new UINT16[cDepthSize];

	colorFrameBuffer = new BYTE[cColorWidth * cColorHeight * 4];

	m_kinect_stream_handler.push_back(std::thread(&KinectThread::startStream, this));

	Sleep(2000);
}

KinectThread::~KinectThread()
{
	// done with depth frame reader
	SafeRelease(m_pDepthFrameReader);

	// close the Kinect Sensor
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);

	delete[] depthFrameBuffer;
	delete[] filteredDepthFrameBuffer;
	delete[] infraredFrameBuffer;
	delete[] colorFrameBuffer;

	m_cloud->clear();
}

void KinectThread::updateFrame()
{
	if (!m_pDepthFrameReader)
	{
		std::cout << "Depth frame reader fail\n";
		return;
	}

	UINT16 *pBuffer = NULL;

	IDepthFrame* pDepthFrame = NULL;

	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

	if (SUCCEEDED(hr))
	{
		hr = pDepthFrame->CopyFrameDataToArray(cDepthSize, depthFrameBuffer);

		// filter flying pixels
		if (FILTER_DEPTH)
		{
			const int win_rad = 2;
			for (int y = win_rad; y < cDepthHeight - win_rad; y++)
			{
				for (int x = win_rad; x < cDepthWidth - win_rad; x++)
				{
					int maxJump = 0;

					int centerIdx = y*cDepthWidth + x;

					int centerD = depthFrameBuffer[centerIdx];

					for (int h = -win_rad; h <= win_rad; h++)
					{
						for (int w = -win_rad; w <= win_rad; w++)
						{
							if (h == 0 && w == 0) continue;

							int neighborD = depthFrameBuffer[centerIdx + h*cDepthWidth + w];

							neighborD = abs(centerD - neighborD);

							maxJump = neighborD > maxJump ? neighborD : maxJump;
						}
					}

					filteredDepthFrameBuffer[centerIdx] = maxJump > 40 ? 0 : centerD;
				}
			}		
		}
	}
	
	SafeRelease(pDepthFrame);

	if (stream_infrared_)
	{
		if (!m_pInfraredFrameReader)
		{
			std::cout << "Infrared frame reader fail\n";
			return;
		}

		IInfraredFrame* pInfraredFrame = NULL;

		hr = m_pInfraredFrameReader->AcquireLatestFrame(&pInfraredFrame);

		if (SUCCEEDED(hr))
		{
			hr = pInfraredFrame->CopyFrameDataToArray(cDepthSize, infraredFrameBuffer);
		}

		SafeRelease(pInfraredFrame);
	}
	
	// get color data
	if (!m_pColorFrameReader) return;

	IColorFrame* pColorFrame = NULL;

	hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

	if (SUCCEEDED(hr))
	{
		hr = pColorFrame->CopyConvertedFrameDataToArray(cColorHeight * cColorWidth * 4, colorFrameBuffer, ColorImageFormat::ColorImageFormat_Bgra);
	}
	
	SafeRelease(pColorFrame);

	if (SUCCEEDED(hr))
	{
		m_updateMutex.lock();

		m_cloud->clear();

		UINT16* finalDepthFrameBuffer = FILTER_DEPTH ? filteredDepthFrameBuffer : depthFrameBuffer;

		for (int y = 0; y < cDepthHeight; y++)
		{
			for (int x = 0; x < cDepthWidth; x++)
			{
				DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
				UINT16 depth = finalDepthFrameBuffer[y * cDepthWidth + x];

				ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
				pCoordinateMapper->MapDepthPointToColorSpace(depthSpacePoint, depth, &colorSpacePoint);
				
				int colorX = static_cast<int>(std::floor(colorSpacePoint.X + 0.5f));
				int colorY = static_cast<int>(std::floor(colorSpacePoint.Y + 0.5f));

				if ((0 <= colorX) && (colorX < cColorWidth) && (0 <= colorY) && (colorY < cColorHeight))
				{
					BYTE* color_p = &colorFrameBuffer[(colorY * cColorWidth + colorX) * 4];

					PointT point;
					CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
					pCoordinateMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);

					if (cameraSpacePoint.Z > 3.f)
						continue;

					// need to negate x and y to have Kinect Fusion world coordinate system, normal camera coordinate system x->left, y->down
					point.x = cameraSpacePoint.X;
					point.y = cameraSpacePoint.Y;
					point.z = cameraSpacePoint.Z;
					point.b = *color_p;
					point.g = *(++color_p);
					point.r = *(++color_p);
					/*point.r = 255;
					point.g = 255;
					point.b = 255;*/
					m_cloud->push_back(point);
				}
			}
		}

		//viewer.showCloud(m_cloud);

		m_updateMutex.unlock();

		/*clock_t toc = clock();
		printf("Elapsed: %f ms\n", (double)(toc - m_tic) / CLOCKS_PER_SEC * 1000.);
		m_tic = clock();*/
	}
}

void KinectThread::startStream()
{
	STREAMING = true;

	while (STREAMING)
	{
		updateFrame();
	}
}

void KinectThread::getCurPointCloud(PointCloudT::Ptr cloud)
{
	cloud->clear();

	m_updateMutex.lock();

	*cloud += *m_cloud;

	m_updateMutex.unlock();
}

// Make sure that the image x axis and image y axis are aligned with robot arm tool frame.
// Otherwise, the hand eye calibration algorithm won't work well. 
// Flip the image accordingly.
cv::Mat KinectThread::getCurRGB()
{
	cv::Mat bgra;
	bgra.create(cColorHeight, cColorWidth, CV_8UC4);

	m_updateMutex.lock();

	std::memcpy(bgra.ptr(0), colorFrameBuffer, cColorHeight*cColorWidth * 4);

	m_updateMutex.unlock();
	
	cv::Mat bgr;

	cv::cvtColor(bgra, bgr, CV_BGRA2BGR);

	cv::Mat output;
	
	//kinect outputs horizontal flipped rgb

	cv::flip(bgr, output, 0);

	//cv::imshow("rgb", output); cv::waitKey(5);
	
	return output;
}


// Make sure that the image x axis and image y axis are aligned with robot arm tool frame.
// Otherwise, the hand eye calibration algorithm won't work well. 
// Flip the image accordingly.
cv::Mat KinectThread::getCurIR()
{
	cv::Mat ir;
	ir.create(cDepthHeight, cDepthWidth, CV_16UC1);

	m_updateMutex.lock();

	std::memcpy(ir.ptr(0), infraredFrameBuffer, cDepthSize*sizeof(UINT16));

	m_updateMutex.unlock();

	cv::Mat output;
	// kinect outputs horizontal flipped IR
	
	cv::flip(ir, output, 0);

	//cv::imshow("ir", output); cv::waitKey(5);
	
	return output;
}