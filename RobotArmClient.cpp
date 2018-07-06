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

		iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);

		/*if (iResult > 0)
		printf("Bytes received: %d\n", iResult);
		else if (iResult == 0)
		printf("Connection closed\n");
		else
		printf("recv failed with error: %d\n", WSAGetLastError());*/

		//if (iResult == 635) // 10Hz
		if (iResult == 1108)	//125Hz	, ur software version > 3.5
		{
			updateMutex.lock();

		//	int package_size = *((int*)recvbuf);

			// index 307 is the cartesion info x start
			getCartesionInfoFromURPackage(recvbuf + 444);	// 125 Hz
			//getCartesionInfoFromURPackage(recvbuf + 307);	// 10 Hz

			getActualJointPosFromURPackage();

			getActualTCPSpeedFromURPackage();

			std::memcpy((char*)&safety_mode_, recvbuf + 812, 8);

			if (safety_mode_ != 0xf03f) exit(EXIT_FAILURE);

			tcp_speed_ = std::sqrt(cur_tcp_speed_array[0]* cur_tcp_speed_array[0] + cur_tcp_speed_array[1] * cur_tcp_speed_array[1] + cur_tcp_speed_array[2] * cur_tcp_speed_array[2]);

			distanceToDst_ = EuclideanDistance(cur_cartesian_info_array, dst_cartesian_info_array);

			displacement_ = EuclideanDistance(cur_cartesian_info_array, start_xyz_);

			distanceToDstConfig_ = configLInfNorm(cur_joint_pos_array, dst_joint_pos_array);

			updateMutex.unlock();

			//std::cout << package_size << std::endl;

			//std::cout << distanceToDst << std::endl;

			//printCartesianInfo(cur_cartesian_info_array);


			/*unsigned char* tmp_buffer = (unsigned char*)recvbuf;

			for (int j = 0; j < iResult - 4; j++)
			{
			std::cout << (unsigned int)tmp_buffer[j]<<" ";

			if (std::memcmp(recvbuf + j, &cartesianStartID, 4) == 0)
			{
			std::cout << "found" << j <<" "<< (unsigned int)tmp_buffer[j+4] << std::endl;
			}

			if (tmp_buffer[j] == 4)
			std::cout << std::endl;
			}*/

			//break;
		}
		else
			//std::cout << "UR msg size wrong:" << iResult << std::endl;
			std::cerr << "UR" << iResult << std::endl;

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

void RobotArmClient::getCartesionInfoFromURPackage(char* x_start_ptr)
{
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

int RobotArmClient::moveHandL(double* dst_cartesian_info, float acceleration, float speed, bool wait2dst)
{
	char msg[128];

	//updateMutex.lock();
	distanceToDst_.store(1000.);
	//updateMutex.unlock();

	sprintf(msg, "movel(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f)\n", 
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

	//updateMutex.lock();
	distanceToDstConfig_.store(1e7);
	//updateMutex.unlock();

	sprintf(msg, "movej([%f,%f,%f,%f,%f,%f],a=%f,v=%f)\n",
		dst_joint_config[0], dst_joint_config[1], dst_joint_config[2],
		dst_joint_config[3], dst_joint_config[4], dst_joint_config[5], acceleration, speed);

	//std::cout << "len" << strlen(msg)<<std::endl;
	
	int num_byte = send(ConnectSocket, msg, strlen(msg), 0);

	if (num_byte == SOCKET_ERROR) return SOCKET_ERROR;

	if (wait2dst) {

		int result = waitTillHandReachDstConfig(dst_joint_config);

		if (result == -1) {
			exit(EXIT_FAILURE);
			return -1;
		}
	}

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
//	updateMutex.lock();

	//UINT64 safety_mode = safety_mode_;

	//updateMutex.unlock();

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

	while (true)
	{
		if (distanceToDst_.load() < 0.0005)	break;

		Sleep(1);
		//std::cout << distanceToDst_.load() << std::endl;
	}

	return 0;
}

int RobotArmClient::waitTillHandReachDstConfig(double* dst_joint_config)
{
	setDstConfigInfo(dst_joint_config);

	int timeout_cnt = 0;

	while (true)
	{
		if (distanceToDstConfig_.load() < 0.03) break;

		if (getSafetyMode() != 0xf03f) {
			std::cerr << "UR10 stopped\n";
			return -1;
		}

		Sleep(10);
		//std::cout << distanceToDstConfig_.load() << std::endl;
		if (timeout_cnt++ > 100*30)
		{
			std::cerr << "wait till reach config timeout - "<< distanceToDstConfig_.load() << std::endl;
			stopRecvTCP();
			URMsgHandler.push_back(std::thread(&RobotArmClient::startRecvTCP, this));
			Sleep(1000);
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
		//if (displacement_.load() >= 0.0001)	break;
		Sleep(1);
	}

	return tcp_speed_.load();
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

		char msg[128];

		sprintf(msg, "set_analog_outputdomain(1, 1)\n");

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
		Sleep(4000);
	}
}

//analog 1
void RobotArmClient::whiteBoardServoControl(bool extend) {

	if (extend) {

		char msg[128];

		sprintf(msg, "set_analog_outputdomain(1, 1)\n");

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
}

void RobotArmClient::laserScannerLightControl(bool on) {

	if (on) {

		char msg[128];

		sprintf(msg, "set_digital_out(2,False)\n");

		int num_byte = send(ConnectSocket, msg, strlen(msg), 0);

		Sleep(200);

		char msg1[128];

		sprintf(msg1, "set_digital_out(2,False)\n");

		num_byte = send(ConnectSocket, msg1, strlen(msg1), 0);
	}
	else {

		char msg[128];

		sprintf(msg, "set_digital_out(2,True)\n");

		int num_byte = send(ConnectSocket, msg, strlen(msg), 0);

		Sleep(200);

		char msg1[128];

		sprintf(msg1, "set_digital_out(2,True)\n");

		num_byte = send(ConnectSocket, msg1, strlen(msg1), 0);
	}
}