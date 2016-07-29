#include "RobotArmClient.h"

RobotArmClient::RobotArmClient()
{
	TCP_ALIVE = true;

	//initial dst pose, no need for Inf
	for (int i = 0; i < 6; i++)
		dst_cartesian_info_array[i] = 0.0;

	result = NULL;

	ptr = NULL;

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	
	if (iResult != 0) 
		printf("WSAStartup failed with error: %d\n", iResult);

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo("192.168.0.2", DEFAULT_PORT, &hints, &result);
	if (iResult != 0) 
	{
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
	}

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
		std::cout<<"set sock opt: "<<setsockopt(ConnectSocket, IPPROTO_TCP, TCP_NODELAY, &value, sizeof(value))<<std::endl;

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
		printf("Unable to connect to server!\n");
		WSACleanup();
		return;
	}

	URMsgHandler.push_back(std::thread(&RobotArmClient::startRecvTCP, this));

	Sleep(1000);
}

void RobotArmClient::startRecvTCP()
{
	unsigned int cartesianStartID = 0x04650000;

	// Receive until the peer closes the connection
	int i = 0;

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
		if (iResult == 1060)	//125Hz
		{
			updateMutex.lock();
			// index 307 is the cartesion info x start
			getCartesionInfoFromURPackage(recvbuf + 444);	// 125 Hz
			//getCartesionInfoFromURPackage(recvbuf + 307);	// 10 Hz

			updateMutex.unlock();

			distanceToDst = EuclideanDistance(cur_cartesian_info_array, dst_cartesian_info_array);

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
			std::cout << "UR msg size wrong:" << iResult << std::endl;

		//clock_t toc = clock();
		//printf("Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

		/*if (i++ == 500)
			break;*/
	}


	// cleanup
	closesocket(ConnectSocket);

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
	{
		reverse8CharsToDouble(x_start_ptr + 7 + i * 8, cur_cartesian_info_array + i);
	}
}

int RobotArmClient::moveHand(double* dst_cartesian_info, float speed)
{
	char msg[128];

	sprintf(msg, "movel(p[%f,%f,%f,%f,%f,%f],a=1.2,v=%f)  \n", 
		dst_cartesian_info[0], dst_cartesian_info[1], dst_cartesian_info[2], 
		dst_cartesian_info[3], dst_cartesian_info[4], dst_cartesian_info[5], speed); 

	//std::cout << "len" << strlen(msg)<<std::endl;
	
	return send(ConnectSocket, msg, strlen(msg), 0);
}

void RobotArmClient::getCartesianInfo(double* array6)
{
	updateMutex.lock();

	std::memcpy(array6, cur_cartesian_info_array, 6 * 8);

	updateMutex.unlock();
}

void RobotArmClient::printCartesianInfo(double* array6)
{
	std::cout << "cartesian: ";
	for (int j = 0; j < 6; j++)
		std::cout << array6[j] << " ";
	std::cout << '\n';
}

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
		if (distanceToDst < 0.0001)
		break;

		Sleep(4);
		//std::cout << distanceToDst << std::endl;
	}

	return 0;
}

void RobotArmClient::setDstCartesianInfo(double* array6)
{
	updateMutex.lock();

	std::memcpy(dst_cartesian_info_array, array6, 48);

	updateMutex.unlock();
}

void RobotArmClient::getDistanceToDst(double& distance)
{
	updateMutex.lock();

	distance = distanceToDst;

	updateMutex.unlock();
}

void RobotArmClient::stopRecvTCP(void)
{
	TCP_ALIVE = false;
	Sleep(1000);
}