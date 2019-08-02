#include "RobotArmClient.h"

RobotArmClient::RobotArmClient()
{
	TCP_ALIVE = true;

	recvbuf = new char[DEFAULT_BUFLEN];

	//initial dst pose, no need for Inf
	for (int i = 0; i < 6; i++)
		dst_cartesian_info_array[i] = 0.0;

	URMsgHandler.push_back(std::thread(&RobotArmClient::startRecvTCP, this));

	Sleep(1000);

	// turn off laser
	laserScannerLightControl(false);
}

RobotArmClient::~RobotArmClient()
{
	stopRecvTCP();

	delete[] recvbuf;
}

void RobotArmClient::startRecvTCP()
{
	result = NULL;

	ptr = NULL;

	TCP_ALIVE = true;

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);

	if (iResult != 0)
	{
		Utilities::to_log_file("WSAStartup failed with error\n");
		TCP_ALIVE = false;
		return;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo("192.168.0.2", DEFAULT_PORT, &hints, &result);
	if (iResult != 0)
	{
		Utilities::to_log_file("getaddrinfo failed with error\n");
		WSACleanup();
		TCP_ALIVE = false;
		return;
	}

	std::cout << "connecting to UR10..." << std::endl;

	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next)
	{
		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);

		if (ConnectSocket == INVALID_SOCKET)
		{
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return;
		}

		// Disable Nagle algorithm for low-latency send
		char value = 1;

		if (setsockopt(ConnectSocket, IPPROTO_TCP, TCP_NODELAY, &value, sizeof(value)) != 0)
			std::cout << "robot arm socket connect fail" << std::endl;

		// Connect to server.
		iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);

		if (iResult == SOCKET_ERROR)
		{
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}
		break;
	}

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET) {
		printf("Unable to connect UR10 to server!\n");
		WSACleanup();
		return;
	}

	unsigned int cartesianStartID = 0x04650000;

	// Receive until the peer closes the connection
	int i = 0;

	lineLightControl(false);

	while (TCP_ALIVE) 
	{
		//clock_t tic = clock();

		//  reset first four bytes
		recvbuf[0] = recvbuf[1] = recvbuf[2] = recvbuf[3] = 0;

		iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);

		if (iResult == 0)
		{
			continue;
		}
		else if (iResult < 0)
		{
			std::cout << "recv error" << std::endl;
			continue;
		}

		timestamp_pose ts_p;
		QueryPerformanceCounter(&ts_p.timestamp);

		std::uint32_t packageLength;
		// Byteswap package length
		char *p = (char *)(&packageLength);

		p[0] = recvbuf[3]; p[1] = recvbuf[2]; p[2] = recvbuf[1]; p[3] = recvbuf[0];

		if ((packageLength == 1116) && (iResult == 1116)) //125Hz	, ur software version > 3.10
		{
			updateMutex.lock();

			//	int package_size = *((int*)recvbuf);

			getTimeFromURPackage();

			getCartesionInfoFromURPackage();	// 125 Hz

			if (record_poses_)
			{
				std::memcpy(ts_p.pose, cur_cartesian_info_array, 48);
				timestamp_pose_vec_.push_back(ts_p);
				ur_timestamp_vec_.push_back(cur_ur_time_);
			}

			getActualJointPosFromURPackage();

			getActualTCPSpeedFromURPackage();

			std::memcpy((char*)&safety_mode_, recvbuf + 812, 8);

			if (safety_mode_ != 0xf03f)
			{
				Utilities::to_log_file("UR streaming fail");
				exit(0);
			}

			tcp_speed_ = std::sqrt(cur_tcp_speed_array[0] * cur_tcp_speed_array[0] + cur_tcp_speed_array[1] * cur_tcp_speed_array[1] + cur_tcp_speed_array[2] * cur_tcp_speed_array[2]);

			rot_speed_ = std::sqrt(cur_tcp_speed_array[3] * cur_tcp_speed_array[3] + cur_tcp_speed_array[4] * cur_tcp_speed_array[4] + cur_tcp_speed_array[5] * cur_tcp_speed_array[5]);

			distanceToDst_ = EuclideanDistance(cur_cartesian_info_array, dst_cartesian_info_array);

			//displacement_ = EuclideanDistance(cur_cartesian_info_array, start_xyz_);

			distanceToDstConfig_ = configLInfNorm(cur_joint_pos_array, dst_joint_pos_array);

			updateMutex.unlock();

			recv_normal_.store(true);

			//std::cout << distanceToDst_ << std::endl;

			//printCartesianInfo(cur_cartesian_info_array);
		}
		else
			recv_normal_.store(false);
			//std::cout << "UR msg size wrong:" << iResult << std::endl;
			//std::cerr << "UR" << iResult << std::endl;

		//clock_t toc = clock();
		//printf("Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

		/*if (i++ == 500)
			break;*/
	}


	// cleanup
	closesocket(ConnectSocket);
	TCP_ALIVE = false;
	WSACleanup();
}


void RobotArmClient::reverse8CharsToDouble(char* end, double* d)
{
	char* tmpCharPtr = (char*)d;

	for (int i = 0; i < 8; i++)
	{
		*tmpCharPtr = *end;
		tmpCharPtr++;
		end--;
	}
}

void RobotArmClient::getCartesionInfoFromURPackage()
{
	char* x_start_ptr = recvbuf + 444;
	for (int i = 0; i < 6; i++)
		reverse8CharsToDouble(x_start_ptr + 7 + i * 8, cur_cartesian_info_array + i);
}

void RobotArmClient::getActualJointPosFromURPackage()
{
	char* start_ptr = recvbuf + 252;

	for (int i = 0; i < 6; i++)
		reverse8CharsToDouble(start_ptr + 7 + i * 8, cur_joint_pos_array + i);
}

void RobotArmClient::getActualTCPSpeedFromURPackage()
{
	char* start_ptr = recvbuf + 492;

	for (int i = 0; i < 6; i++)
		reverse8CharsToDouble(start_ptr + 7 + i * 8, cur_tcp_speed_array + i);
}

void RobotArmClient::getTimeFromURPackage()
{
	char* start_ptr = recvbuf + 4;

	reverse8CharsToDouble(start_ptr + 7, &cur_ur_time_);
}

int RobotArmClient::moveHandL(double* dst_cartesian_info, float acceleration, float speed, bool wait2dst)
{
	char msg[128];

	//updateMutex.lock();
	distanceToDst_.store(1000.);
	//updateMutex.unlock();

	// "p" means pose, without "p" means joint configuration
	sprintf(msg, "movel(p[%.10f,%.10f,%.10f,%.10f,%.10f,%.10f],a=%.10f,v=%.10f)\n", 
		dst_cartesian_info[0], dst_cartesian_info[1], dst_cartesian_info[2], 
		dst_cartesian_info[3], dst_cartesian_info[4], dst_cartesian_info[5], acceleration, speed);

	//std::cout << "len" << strlen(msg)<<std::endl;
	int num_byte = send(ConnectSocket, msg, strlen(msg), 0);

	if (num_byte == SOCKET_ERROR) return SOCKET_ERROR;

	if (wait2dst) waitTillHandReachDstPose(dst_cartesian_info);

	return num_byte;
}

int RobotArmClient::moveHandJ(double* dst_joint_config, float speed, float acceleration, bool wait2dst)
{
	char msg[128];

	distanceToDstConfig_.store(1e7);

	// "p" means pose, without "p" means joint configuration
	sprintf(msg, "movej([%.10f,%.10f,%.10f,%.10f,%.10f,%.10f],a=%.10f,v=%.10f,t=0,r=0)\n",
		dst_joint_config[0], dst_joint_config[1], dst_joint_config[2],
		dst_joint_config[3], dst_joint_config[4], dst_joint_config[5], acceleration, speed);

	//std::cout << msg << std::endl;
	//std::cout << "len" << strlen(msg)<<std::endl;
	
	int num_byte = send(ConnectSocket, msg, strlen(msg), 0);

	if (num_byte == SOCKET_ERROR) return SOCKET_ERROR;

	if (wait2dst) waitTillHandReachDstConfig(dst_joint_config);

	return num_byte;
}

void RobotArmClient::getCartesianInfo(double* array6)
{
	updateMutex.lock();

	std::memcpy(array6, cur_cartesian_info_array, 6 * 8);

	updateMutex.unlock();
}

void RobotArmClient::getTCPSpeed(double* array6)
{
	updateMutex.lock();

	std::memcpy(array6, cur_tcp_speed_array, 6 * 8);

	updateMutex.unlock();
}

void RobotArmClient::getCurJointPose(double* array6)
{
	updateMutex.lock();

	std::memcpy(array6, cur_joint_pos_array, 6 * 8);

	updateMutex.unlock();
}

UINT64 RobotArmClient::getSafetyMode()
{
	return safety_mode_.load();
}

void RobotArmClient::printCartesianInfo(double* array6)
{
	std::cout << "cartesian: ";
	for (int j = 0; j < 6; j++)
		std::cout << array6[j] << " ";
	std::cout << '\n';
}

// only compute distance of the first 3 elements
double RobotArmClient::EuclideanDistance(double* pose6_1, double* pose6_2)
{
	double sum_squares = 0.;

	for (int i = 0; i < 3; i++)
	{
		double diff = pose6_1[i] - pose6_2[i];
		
		sum_squares += diff*diff;
	}

	return sqrt(sum_squares);
}

int RobotArmClient::waitTillHandReachDstPose(double* dst_pose6)
{
	setDstCartesianInfo(dst_pose6);

	int timeout_cnt = 0;
	const int limit = 1000 * 60;

	while (true)
	{
		//std::cout << distanceToDst_.load() << std::endl;
		if (distanceToDst_.load() < 0.004)	break;

		Sleep(1);

		if (++timeout_cnt > limit)
		{
			std::string final_distance = std::to_string(distanceToDst_.load());
			Utilities::to_log_file("waitTillHandReachDstPose timeout: "+final_distance);
			return -1;
		}
	}

	return 0;
}

double RobotArmClient::waitTillRotationSpeedDropDownTo(double target_speed)
{
	double cur_speed = 0.;

	while (true)
	{
		cur_speed = rot_speed_.load();
		Sleep(1);
		if ( cur_speed < target_speed)	break;
	}

	return cur_speed;
}

double RobotArmClient::waitTillTranslationSpeedDropDownTo(double target_speed)
{
	double cur_speed = 0.;

	while (true)
	{
		cur_speed = tcp_speed_.load();
		Sleep(1);
		if (cur_speed < target_speed)	break;
	}

	return cur_speed;
}

int RobotArmClient::waitTillHandReachDstConfig(double* dst_joint_config)
{
	std::thread::id this_id = std::this_thread::get_id();

	setDstConfigInfo(dst_joint_config);

	int timeout_cnt = 0;

	while (true)
	{
		if (distanceToDstConfig_.load() < 0.03) break;

		Sleep(200);
		//std::cout << distanceToDstConfig_.load() << std::endl;
		if (timeout_cnt++ > 2*20)
		{
			std::string txt = "wait till reach config timeout - " + std::to_string(distanceToDstConfig_.load()) + " - " + std::to_string(recv_normal_.load());
			Utilities::to_log_file(txt);
			break;
		}
	}

	Sleep(1000);

	return 0;
}

void RobotArmClient::setDstCartesianInfo(double* array6)
{
	updateMutex.lock();
	std::memcpy(dst_cartesian_info_array, array6, 48);
	distanceToDst_ = 1000.;
	updateMutex.unlock();
}


void RobotArmClient::setDstConfigInfo(double* array6)
{
	updateMutex.lock();
	std::memcpy(dst_joint_pos_array, array6, 48);
	distanceToDstConfig_ = 1000.;
	updateMutex.unlock();
}

void RobotArmClient::getDistanceToDst(double& distance)
{
	//updateMutex.lock();
	distance = distanceToDst_.load();
	//updateMutex.unlock();
}

void RobotArmClient::stopRecvTCP(void)
{
	TCP_ALIVE = false;
	URMsgHandler.back().join();
	URMsgHandler.clear();
}

void RobotArmClient::setStartPoseXYZ()
{
	double pose[6];
	getCartesianInfo(pose);
	std::memcpy(start_xyz_, pose, 3*sizeof(double));
	displacement_.store(0);
}

double RobotArmClient::waitTillTCPMove(double target_speed)
{
	//int counter = 0;
	while (true)
	{
		//counter++;
		if (tcp_speed_.load() >= target_speed) break;
		Sleep(1);
	}

	return tcp_speed_.load();
	//return counter;
}

double RobotArmClient::waitTillRotate(double target_speed)
{
	//int counter = 0;
	while (true)
	{
		//counter++;
		if (rot_speed_.load() >= target_speed) break;
		Sleep(1);
	}

	return rot_speed_.load();
	//return counter;
}

void RobotArmClient::rotateJointRelative(int id, double deg, float acceleration, float speed)
{
	if (id < 0 || id > 5)
	{
		std::cout << "invalid joint id\n";
		return;
	}

	double joints[6];
	getCurJointPose(joints);
	joints[id] += deg*M_PI/180.;
	moveHandJ(joints, speed, acceleration, true);
}

void RobotArmClient::moveHandRelativeTranslate(double x, double y, double z, float acceleration, float speed)
{
	double pose[6];
	getCartesianInfo(pose);
	pose[0] += x; 
	pose[1] += y; 
	pose[2] += z;
	moveHandL(pose, acceleration, speed);
}

double RobotArmClient::configLInfNorm(double* config6_1, double* config6_2)
{
	//L infinite norm
	double abs_max = abs(config6_1[0] - config6_2[0]);
	double tmp;

	for (int i = 1; i < 6; i++) {

		tmp = abs(config6_1[i] - config6_2[i]);

		if (tmp > abs_max) 
			abs_max = tmp;
	}

	return abs_max;
#if 0
	//L2 norm
	double sum_squares = 0.;

	for (int i = 0; i < 6; i++)
	{
		double diff = config6_1[i] - config6_2[i];

		sum_squares += diff*diff;
	}

	return sqrt(sum_squares);
#endif
}

void RobotArmClient::lineLightControl(bool turn_on) {

	if (turn_on) {

		char msg[64];

		sprintf(msg, "set_digital_out(1,True)\n");

		int num_byte = send(ConnectSocket, msg, strlen(msg), 0);

		Sleep(1000);

		sprintf(msg, "set_analog_outputdomain(0, 0)\n");	//port 0, current

		num_byte = send(ConnectSocket, msg, strlen(msg), 0);

		Sleep(100);

		char msg1[64];

		sprintf(msg1, "set_standard_analog_out(0, 0.1)\n");

		num_byte = send(ConnectSocket, msg1, strlen(msg1), 0);
	}
	else {

		char msg[64];

		sprintf(msg, "set_digital_out(1,False)\n");

		int num_byte = send(ConnectSocket, msg, strlen(msg), 0);

		Sleep(200);
	}
}

void RobotArmClient::probeCylinderControl(bool extend) {

	char msg[128];

	if (extend) {

		sprintf(msg, "set_digital_out(0,True)\n");

		int num_byte = send(ConnectSocket, msg, strlen(msg), 0);

		//wait for finish
		Sleep(4000);
	}
	else {

		sprintf(msg, "set_digital_out(0,False)\n");

		int num_byte = send(ConnectSocket, msg, strlen(msg), 0);

		//wait for finish
		Sleep(1000);
	}
}

//analog 1
void RobotArmClient::whiteBoardServoControl(bool extend) {

	if (extend) {

		char msg[128];

		sprintf(msg, "set_analog_outputdomain(1, 1)\n");	//port 1, voltage

		int num_byte = send(ConnectSocket, msg, strlen(msg), 0);

		Sleep(100);

		char msg1[128];

		sprintf(msg1, "set_standard_analog_out(1, 0.5)\n");

		num_byte = send(ConnectSocket, msg1, strlen(msg1), 0);
	}
	else {

		char msg[128];

		sprintf(msg, "set_analog_outputdomain(1, 1)\n");

		int num_byte = send(ConnectSocket, msg, strlen(msg), 0);

		Sleep(100);

		char msg1[128];

		sprintf(msg1, "set_standard_analog_out(1, 0.0)\n");

		num_byte = send(ConnectSocket, msg1, strlen(msg1), 0);
	}

	Sleep(2000);
}

void RobotArmClient::laserScannerLightControl(bool on) {

	if (on) {

		char msg[128];

		sprintf(msg, "set_digital_out(2,False)\n");

		int num_byte = send(ConnectSocket, msg, strlen(msg), 0);

		Sleep(200);
	}
	else {

		char msg[128];

		sprintf(msg, "set_digital_out(2,True)\n");

		int num_byte = send(ConnectSocket, msg, strlen(msg), 0);

		Sleep(200);
	}
}

void RobotArmClient::startOrStopRecordPose(bool start)
{
	if (start)
	{
		record_poses_.store(false);
		updateMutex.lock();
		timestamp_pose_vec_.clear();
		updateMutex.unlock();
		record_poses_.store(true);
	}
	else
		record_poses_.store(false);
}

std::vector<RobotArmClient::timestamp_pose> RobotArmClient::getTimestampPoses()
{
	updateMutex.lock();
	std::vector<RobotArmClient::timestamp_pose> ts_ps(timestamp_pose_vec_);
	updateMutex.unlock();

	return ts_ps;
}

int RobotArmClient::saveTimeStampPoses(std::string filename)
{
	filename += "_timestamp_hand_pose.csv";
	std::ofstream out(filename, std::ios::out);

	if (!out.is_open())
		return -1;

	int cnt = 0;
	for (auto & ts_p : timestamp_pose_vec_)
	{
		out << ts_p.timestamp.QuadPart;
		for (int i = 0; i < 6; i++)
			out << "," << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << ts_p.pose[i] << "," << ur_timestamp_vec_[cnt];
		
		if(cnt++ != timestamp_pose_vec_.size()-1 ) out << std::endl;
	}
	
	out.close();
	return 0;
}

int RobotArmClient::saveCurPose(std::string filename)
{
	filename += "_tcp_pose.csv";
	std::ofstream out(filename, std::ios::out);

	if (!out.is_open())
		return -1;

	double cur_pose[6];

	getCartesianInfo(cur_pose);
	
	for (int i = 0; i < 6; i++)
		out << "," << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << cur_pose[i];

	out.close();
	return 0;
}