#include "TOF_Swift.h"

TOF_Swift::TOF_Swift()
{
	streaming_.store(false);

	cloud_.reset(new PointCloudT);
	PointT p; p.x = p.y = p.z = 0;
	cloud_->push_back(p);
	cloud_->is_dense = true;

	range_filtered_.create(img_height_, img_width_, CV_16UC1);
	range_raw_16u_.create(img_height_, img_width_, CV_16UC1);

	intensity_img_16u_.create(img_height_, img_width_, CV_16UC1);
	intensity_img_.create(img_height_, img_width_, CV_8UC1);

#if 0 
	//test if camera can start
	start();
	std::cout << "tof started\n";
	stop();
	std::cout << "tof stopped\n";
#endif
}

TOF_Swift::~TOF_Swift()
{
	stop();
	delete[] xMultiplier;
	delete[] yMultiplier;
	delete[] zMultiplier;
}

void TOF_Swift::start()
{
	if (streaming_.load()) stop();

	// the first connection fails sometimes
	for (int i = 0; i < 10; i++)
	{
		if (odos::Result::Success == cam_.open(ip_address))
			break;

		if (i == 9)
		{
			Utilities::to_log_file("Fail to connect to odos");
			exit(0);
		}

		Sleep(500);
	}

	getAsImage(cam_, xMultiplier, yMultiplier, zMultiplier); //getImage() function does not work after firmware update
	thread_vec_.push_back(std::thread(&TOF_Swift::startStreaming, this));

	// wait for streaming start
	while (!streaming_.load())
		Sleep(1000);
}

void TOF_Swift::getAsImage(odos::Camera& camera, const float*& xMultiplier, const float*& yMultiplier, const float*& zMultiplier)
{
	if (odos::Result::Success != camera.getPointCloudMultipliers(xMultiplier, yMultiplier, zMultiplier))
	{
		Utilities::to_log_file("getPointCloudMultipliers failed");
		exit(0);
	}

	// this method does not work anymore
	/*xMultiplier = getImage(camera, odos::ComponentType::PointCloudMultiplierX);
	yMultiplier = getImage(camera, odos::ComponentType::PointCloudMultiplierY);
	zMultiplier = getImage(camera, odos::ComponentType::PointCloudMultiplierZ);*/
}

float* TOF_Swift::getImage(odos::Camera& camera, odos::ComponentType component)
{
	// Stop any current acquisition
	camera.acquisitionStop();

	odos::EnumNode* componentSelector = camera.getEnumNode("ComponentSelector");
	odos::BooleanNode* componentEnable = camera.getBooleanNode("ComponentEnable");
	if (componentSelector == nullptr || componentEnable == nullptr)
	{
		std::cerr << "could not access required nodes" << std::endl;
		exit(1);
	}
	std::vector<odos::EnumEntry> components;
	if (odos::Result::Success != componentSelector->getEntries(components))
	{
		std::cerr << "could not get components" << std::endl;
		exit(1);
	}

	// Select only one multiplier
	for (auto component : components)
	{
		componentSelector->set(component.enum_name);
		componentEnable->set(false);
	}
	componentSelector->set(componentTypeToText(component));
	componentEnable->set(true);

	// Start the acquisition
	camera.acquisitionStart();

	odos::IImage* image;
	float* buff = nullptr;
	do
	{
		if (odos::Result::Success != camera.waitForImage(&image, std::chrono::seconds(10)))
		{
			std::cerr << "could not get images" << std::endl;
			exit(1);
		}
		if (image->component() == component)
		{
			// Multiplier size is always 640x480 32bit floats
			if (image->pixelBufferSize() < img_size_ * sizeof(float))
			{
				std::cerr << "invalid multiplier image received" << std::endl;
				exit(1);
			}

			buff = new float[img_size_];
			memcpy(buff, image->pixelBuffer(), img_size_ * sizeof(float));
		}
		camera.releaseImage(image);
	} while (buff == nullptr);

	// Stop any current acquisition
	camera.acquisitionStop();

	return buff;
}


void TOF_Swift::startStreaming()
{
	setPower(2600);

	setupStreaming();

	odos::Result res;

	// Multiplier size is always 640x480 32bit floats
	cv::Size size(640, 480);

	// We could have used Camera::readFile to receive directly into the OpenCV matrices.
	// As we will not modify the contents we can use a const cast to allow wrapping the multiplier arrays without needing a copy.
	cv::Mat xa(size, CV_32FC1, const_cast<float*>(xMultiplier));
	cv::Mat ya(size, CV_32FC1, const_cast<float*>(yMultiplier));
	cv::Mat za(size, CV_32FC1, const_cast<float*>(zMultiplier));

#if 0
	std::ofstream out("odos_x_multiplier_32f.bin", std::ios::out | std::ios::binary);

	if (out.is_open())
	{
		out.write((char*)xa.data, DEPTH_HEIGHT * DEPTH_WIDTH * sizeof(float));

		out.close();
	}

	out.open("odos_y_multiplier_32f.bin", std::ios::out | std::ios::binary);

	if (out.is_open())
	{
		out.write((char*)ya.data, DEPTH_HEIGHT * DEPTH_WIDTH * sizeof(float));

		out.close();
	}

	out.open("odos_z_multiplier_32f.bin", std::ios::out | std::ios::binary);

	if (out.is_open())
	{
		out.write((char*)za.data, DEPTH_HEIGHT * DEPTH_WIDTH * sizeof(float));

		out.close();
	}
#endif

	cv::Mat range_m;
	cv::Mat x(size, CV_32FC1);
	cv::Mat y(size, CV_32FC1);
	cv::Mat z(size, CV_32FC1);

	cv::Point center(size.width / 2, size.height / 2);
	
	// 4. Start the acquisition and streaming using the "AcquisitionStart" odos::CommandNode.
	if ((res = cam_.getCommandNode("AcquisitionStart")->execute()) != odos::Result::Success)
	{
		Utilities::to_log_file("AcquisitionStart failed");
		exit(1);
	}

	streaming_ = true;

	last_frame_tick_count_.store(cv::getTickCount());

	while (streaming_.load())
	{
		odos::IImage* odos_image_ = nullptr;

		// Wait for images with odos::Camera::waitForImage, if timeout is 1s, it will time out
		if ((res = cam_.waitForImage(&odos_image_, std::chrono::seconds(wait_image_timeout_sec_))) != odos::Result::Success)
		{
#if 1
			if (res == odos::Result::Timeout)
			{
				Utilities::to_log_file("wait for odos image time out");
				//exit(0);
			}
			else
			{
				Utilities::to_log_file("wait for odos image fail");
				//exit(0);
			}
#endif
		}
		else 
		{
			if (odos_image_->component() == odos::ComponentType::Intensity)
			{
				std::memcpy((UINT8*)intensity_img_16u_.ptr<unsigned short>(0), (UINT8*)odos_image_->pixelBuffer(), img_size_*sizeof(unsigned short));

				cam_.releaseImage(odos_image_);
				update_mutex_.lock();
				intensity_img_16u_.convertTo(intensity_img_, CV_8U, 255. / 20000.);
				update_mutex_.unlock();	
			}
			else if (odos_image_->component() == odos::ComponentType::Range)
			{
				std::memcpy((UINT8*)range_raw_16u_.ptr<unsigned short>(0), (UINT8*)odos_image_->pixelBuffer(), img_size_*sizeof(unsigned short));
				cam_.releaseImage(odos_image_);

#if 0
				// filter flying pixels
				unsigned short *depthFrameBuffer = range_raw_16u_.ptr<unsigned short>(0);
				unsigned short *filteredDepthFrameBuffer = range_filtered_.ptr<unsigned short>(0);

				const int win_rad = 1;
				for (int y = win_rad; y < img_height_ - win_rad; y++)
				{
					for (int x = win_rad; x < img_width_ - win_rad; x++)
					{
						int maxJump = 0;

						int centerIdx = y*img_width_ + x;

						int centerD = depthFrameBuffer[centerIdx];

						for (int h = -win_rad; h <= win_rad; h++)
						{
							for (int w = -win_rad; w <= win_rad; w++)
							{
								if (h == 0 && w == 0) continue;

								int neighborD = depthFrameBuffer[centerIdx + h*img_width_ + w];

								neighborD = std::abs(centerD - neighborD);

								if (neighborD > maxJump)
									maxJump = neighborD;
							}
						}

						filteredDepthFrameBuffer[centerIdx] = maxJump > 400 ? 0 : centerD;
					}
				}
#endif

				// Convert range to range in metres, by default odos range image intensity = 0.1 mm
				range_raw_16u_.convertTo(range_m, CV_32FC1, 0.0001, 0.);

				update_mutex_.lock();

				cloud_->clear();

				float* range_ptr = range_m.ptr<float>(0);
				float* xm_ptr = xa.ptr<float>(0);
				float* ym_ptr = ya.ptr<float>(0);
				float* zm_ptr = za.ptr<float>(0);

				for (int i = 0; i < img_size_; ++i )
				{
					const float range = (*range_ptr);
					const float z = range*(*zm_ptr);

					if (z > 0.005f && z < 4.f) 
					{
						PointT p;
						p.z = z;
						p.x = -(*xm_ptr)*range;
						p.y = -(*ym_ptr)*range;
						p.r = 255;
						p.g = 255;
						p.b = 255;

						cloud_->push_back(p);
					}

					++range_ptr;
					++xm_ptr;
					++ym_ptr;
					++zm_ptr;
				}

				update_mutex_.unlock();
			}

			last_frame_tick_count_.store(cv::getTickCount());
		}
	}

	if (cam_.acquisitionStop() != odos::Result::Success)
	{
		Utilities::to_log_file("stop tof camera fail\n");
		exit(0);
	}
}


int TOF_Swift::stop()
{
	streaming_.store(false);

	if(thread_vec_.size() > 0)
		thread_vec_[0].join();

	thread_vec_.clear();

	cam_.close();

	return 0;
}


void TOF_Swift::setPower(int power)
{
	if (power < 256 || power>3584) { std::cout << "Power out of range: try(256~3584)" << std::endl; return; }

	if (!cam_.isConnected()) { std::cout << "Camera is not connected" << std::endl; return; }

	odos::Node*node = cam_.getNode("IlluminationPower");

	if (odos::Result::Success != node->setValueFromString(std::to_string(power))) 	
		std::cout << "Failed to change intensity"<<std::endl;

	// takes time to take effect, otherwise, the point cloud would lose points
	Sleep(1000);
}

void TOF_Swift::listNodes()
{
	if (!cam_.isConnected()) { std::cout << "Camera is not connected" << std::endl; return; }
	std::cout << "Getting Nodes" << std::endl;
	std::string currName;
	std::string name;
	std::string prefix;

	//Second method
	odos::CategoryNode* node = cam_.getCategoryNode("Root");

	if (node == nullptr)
	{
		std::cerr << "Invalid category" << std::endl;
		return;
	}
	else {
		std::cout << "Accesed Root Category succesfully" << std::endl;
		node->getName(name);
		if (!prefix.empty()) prefix += ".";
		prefix += name;
		std::vector<odos::Node*> features;
		node->getFeatures(features);
		for (auto feature : features)
		{
			feature->getName(name);
			std::cout << prefix << "." << name << std::endl;
			odos::CategoryNode* category = dynamic_cast<odos::CategoryNode*>(feature);
			if (category != nullptr) {
				std::cout << "Tiene hijos" << std::endl << std::endl;
				seeNodeChilds(category, prefix);
			}
		}
	}
}

int TOF_Swift::seeNodeChilds(odos::CategoryNode* node, std::string prefix )
{
	if (node == nullptr)
	{
		std::cerr << "Invalid category" << std::endl;
		return 1;
	}

	std::string name;
	node->getName(name);

	if (!prefix.empty()) prefix += ".";
	prefix += name;

	std::vector<odos::Node*> features;
	node->getFeatures(features);

	for (auto feature : features)
	{
		feature->getName(name);
		std::cout << prefix << "." << name << std::endl;

		// You can use dynamic_cast to check what sub-type an odos::Node is
		odos::CategoryNode* category = dynamic_cast<odos::CategoryNode*>(feature);
		if (category != nullptr)
		{
			seeNodeChilds(category, prefix);
		}
	}
	return 0;
}

void TOF_Swift::setupStreaming()
{
	odos::EnumNode* componentSelector = cam_.getEnumNode("ComponentSelector");
	odos::BooleanNode* componentEnable = cam_.getBooleanNode("ComponentEnable");
	if (componentSelector == nullptr || componentEnable == nullptr)
	{
		std::cerr << "could not access required nodes" << std::endl;
		exit(-1);
	}
	componentSelector->set("Range");
	componentEnable->set(true);
	componentSelector->set("Intensity");
	componentEnable->set(true);
}

void TOF_Swift::getPointCloud(PointCloudT::Ptr cloud)
{
	waitTillLastestFrameArrive();
	update_mutex_.lock();
	*cloud = *cloud_;
	update_mutex_.unlock();
}

cv::Mat TOF_Swift::getIR()
{
	cv::Mat copy;
	waitTillLastestFrameArrive();
	update_mutex_.lock();
	copy = intensity_img_.clone();
	update_mutex_.unlock();
	return copy;
}

cv::Mat TOF_Swift::getIR16U()
{
	cv::Mat copy;
	waitTillLastestFrameArrive();
	update_mutex_.lock();
	copy = intensity_img_16u_.clone();
	update_mutex_.unlock();
	return copy;
}

cv::Mat TOF_Swift::getDepth16U()
{
	cv::Mat copy;
	waitTillLastestFrameArrive();
	update_mutex_.lock();
	copy = range_raw_16u_.clone();
	update_mutex_.unlock();
	return copy;
}

void TOF_Swift::waitTillLastestFrameArrive()
{
	int retry_cnt = 0;

	while (!streaming_.load()) { Sleep(2000); }

	while (1)
	{
		double since_last_frame = ((double)cv::getTickCount() - last_frame_tick_count_.load()) / cv::getTickFrequency();

		if (since_last_frame < (double)wait_image_timeout_sec_)
			return;
		else
		{
			if (++retry_cnt > 5)
			{
				Utilities::to_log_file("tof final restart fail");
				exit(0);
			}

			stop();
			start();
		}
	}
}