#include "FlirThermoCamClient.h"

FlirThermoCamClient::FlirThermoCamClient()
{
	if (connectToServer() == 0)
	{
		std::cout << "Connected to flir thermo server\n";
	}
	else
		std::cout << "Not connected to flir thermo server\n";
}

int FlirThermoCamClient::connectToServer()
{
	// Try to open a named pipe; wait for it, if necessary. 
	std::cout << "connect to flir thermo\n";
	while(1)
	{
		Sleep(2000);

		hPipe = CreateFile(
			lpszPipename,   // pipe name 
			GENERIC_READ |  // read and write access 
			GENERIC_WRITE,
			0,              // no sharing 
			NULL,           // default security attributes
			OPEN_EXISTING,  // opens existing pipe 
			0,              // default attributes 
			NULL);          // no template file 

							// Break if the pipe handle is valid. 

		if (hPipe != INVALID_HANDLE_VALUE)
		{
			isConnected = true;
			break;
		}
	}

	//isConnected = true;

	if (isConnected == false)
	{
		Utilities::to_log_file("Failed to connect to FLIR\n");
		exit(-1);
	}
	return 0;
}

int FlirThermoCamClient::snapShot(cv::Mat & color_map, cv::Mat & temperature_map, unsigned char focus_dist_cm)
{
	if (!isConnected)
	{
		std::cout << "Not connected to server\n";
		return -1;
	}

	// Send a message to the pipe server. 
	//std::cout << "sending cmd 1\n";

	unsigned char cmd = focus_dist_cm;

	fSuccess = WriteFile(
		hPipe,                  // pipe handle 
		&cmd,             // message 
		1,              // message length 
		&cbWritten,             // bytes written 
		NULL);                  // not overlapped 

	if (!fSuccess)
	{
		//_tprintf(TEXT("WriteFile to pipe failed. GLE=%d\n"), GetLastError());
		std::cout << "WriteFile to pipe failed\n";
		isConnected = false;
		return -1;
	}

	unsigned char recvByte = 0;

	fSuccess = ReadFile(
		hPipe,    // pipe handle 
		&recvByte,    // buffer to receive reply 
		1,  // size of buffer 
		&cbRead,  // number of bytes read 
		NULL);    // not overlapped 

	if (!fSuccess)
	{
		std::cout << "Failed to recevie confirmation\n";
		isConnected = false;
		return -1;
	}
	else if (recvByte == 1) {
		//	std::cout << "Receive confirmation\n";

#if 0
		bool cSuccess = CopyFileW(_T("c:/users/lietang123/documents/roadfiles/flirthermocamserver/flirthermocamserver/bin/release/thermo.jpg"),
			_T("c:/users/lietang123/documents/roadfiles/lineprofilerrobotarmtest/thermo.jpg"),
			false);	// false means overwrite

		cSuccess = CopyFileW(_T("c:/users/lietang123/documents/roadfiles/flirthermocamserver/flirthermocamserver/bin/release/temperature.bin"),
			_T("c:/users/lietang123/documents/roadfiles/lineprofilerrobotarmtest/temperature.bin"),
			false);	// false means overwrite
#endif

		//if (cSuccess)
		{
			//std::cout << "copy image success!\n";
			color_map = cv::imread("c:/users/lietang123/documents/roadfiles/flirthermocamserver/flirthermocamserver/bin/release/thermo.jpg", CV_LOAD_IMAGE_COLOR);
			temperature_map.create(480, 640, CV_64F);

			std::streampos size;
			char * memblock;

			std::ifstream file("c:/users/lietang123/documents/roadfiles/flirthermocamserver/flirthermocamserver/bin/release/temperature.bin", std::ios::in | std::ios::binary | std::ios::ate);

			if (file.is_open())
			{
				size = file.tellg();
				memblock = new char[size];
				file.seekg(0, std::ios::beg);
				file.read(memblock, size);
				file.close();

				//		std::cout << "the entire temperature file content is in memory";

				std::memcpy(temperature_map.data, (unsigned char*)memblock, size);

				delete[] memblock;
			}
			else std::cout << "Unable to open temperature file";
		}
	/*	else
		{
			_tprintf(TEXT("copy image fail! GLE=%d\n"), GetLastError());
			return -1;
		}*/
	}
	
	return 1;
}