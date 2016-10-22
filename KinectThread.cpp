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

		SafeRelease(pDepthFrameSource);
		SafeRelease(pColorFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		std::cout<<"No ready Kinect found!\n";
		return;
	}

	m_cloud.reset(new PointCloudT);
	m_cloud->is_dense = true;

	m_tic = clock();

	depthFrameBuffer = new UINT16[cDepthWidth * cDepthHeight];

	filteredDepthFrameBuffer = new UINT16[cDepthWidth * cDepthHeight];

	colorFrameBuffer = new BYTE[cColorWidth * cColorHeight * 4];

	m_kinect_stream_handler.push_back(std::thread(&KinectThread::startStream, this));

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
		UINT nBufferSize = cDepthWidth * cDepthHeight;

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->CopyFrameDataToArray(nBufferSize, depthFrameBuffer);

			// filter flying pixels
			if (FILTER_DEPTH)
			{
				for (int y = 1; y < cDepthHeight - 1; y++)
				{
					for (int x = 1; x < cDepthWidth - 1; x++)
					{
						int maxJump = 0;

						int centerIdx = y*cDepthWidth + x;

						int centerD = depthFrameBuffer[centerIdx];

						for (int h = -1; h <= 1; h+=2)
						{
							for (int w = -1; w <= 1; w+=2)
							{
								int neighborD = depthFrameBuffer[centerIdx + h*cDepthWidth + w];

								neighborD = abs(centerD - neighborD);

								maxJump = neighborD > maxJump ? neighborD : maxJump;
							}
						}

						filteredDepthFrameBuffer[centerIdx] = maxJump > 20 ? 0 : centerD;
					}
				}		
			}
		}
	}
	
	SafeRelease(pDepthFrame);

	// get color data
	if (!m_pColorFrameReader)
	{
		return;
	}

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

					if (cameraSpacePoint.Z > 2.f)
						continue;

					// need to negate x and y to have Kinect Fusion world coordinate system, normal camera coordinate system x->left, y->down
					point.x = -cameraSpacePoint.X;
					point.y = -cameraSpacePoint.Y;
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
	// kinect outputs horizontal flipped rgb
	cv::flip(bgr, output, 1);

	//cv::imshow("rgb", output);
	//cv::waitKey(0);
	return output;
}