#ifndef FLIR_THERMO_CAM_CLIENT_H
#define FLIR_THERMO_CAM_CLIENT_H

#include <windows.h>
#include "Shlwapi.h"
#include <iostream>
#include <stdio.h>
#include <conio.h>
#include <tchar.h>
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define BUFSIZE 512

class FlirThermoCamClient
{
public:
	HANDLE hPipe;
	LPTSTR lpvMessage = TEXT("Default message from client.");
	TCHAR  chBuf[BUFSIZE];
	BOOL   fSuccess = FALSE;
	DWORD  cbRead, cbToWrite, cbWritten, dwMode;
	LPTSTR lpszPipename = TEXT("\\\\.\\pipe\\thermo-pipe");

	bool isConnected = false;

	FlirThermoCamClient();

	int connectToServer();

	int snapShot(cv::Mat & color_map, cv::Mat & temperature_map);

};

#endif
