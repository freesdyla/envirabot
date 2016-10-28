#ifndef KEYENCE_LINE_PROFILER_H
#define KEYENCE_LINE_PROFILER_H
#include <iostream>
#include <Windows.h>
#include "LJV7_IF.h"
#include "LJV7_ErrorCode.h"
#include <vector>
#include <mutex>
#include <fstream>
#include <string>
#include <time.h>

struct PROFILE_DATA
{
	LJV7IF_PROFILE_INFO m_profileInfo;

	LJV7IF_PROFILE_HEADER m_profileHeader;
	int* m_pnProfileData;
	LJV7IF_PROFILE_FOOTER m_profileFooter;

	PROFILE_DATA();
	PROFILE_DATA(const LJV7IF_PROFILE_INFO &profileInfo, const LJV7IF_PROFILE_HEADER *header, const int *data, const LJV7IF_PROFILE_FOOTER *footer);

	PROFILE_DATA(const PROFILE_DATA& obj);
	PROFILE_DATA& operator = (const PROFILE_DATA& obj);
	~PROFILE_DATA();
};
/*
struct ThreadSafeBuffer
{
	static ThreadSafeBuffer* m_threadSafeBuffer;

	std::mutex mtx;
	int m_anBatchNo;							// Batch number
	DWORD m_adwCount;							// Profile data count
	DWORD m_adwNotify;							// Callback function notification parameter
	std::vector<PROFILE_DATA> m_vecProfileData;		// Profile data

	ThreadSafeBuffer(void);

	static ThreadSafeBuffer* getInstance(void);
	void Add(DWORD dwIndex, std::vector<PROFILE_DATA> &vecProfileData, DWORD dwNotify);
	void AddCount(DWORD deIndex, DWORD dwCount, DWORD dwNotify);
	void Get(DWORD dwIndex, DWORD* pdwNotify, int* pnBatchNo, std::vector<PROFILE_DATA> &vecProfileData);
	void ClearBuffer(int nCurrentDeviceID);
	DWORD GetCount(DWORD dwIndex, DWORD* pdwNotify, int* pnBatchNo);
};*/

struct KeyenceLineProfiler
{
	int DEVICE_ID = 0;

	bool device_initialized = false;

	static LJV7IF_PROFILE_INFO m_profileInfo;

	static std::vector<PROFILE_DATA> m_vecProfileData;

	static clock_t tic;

	LJV7IF_HIGH_SPEED_PRE_START_REQ req;

	DWORD dwThreadId;

	int nRc;

	void init();

	void finalize();

	void start(DWORD dwProfileCnt);

	void stop();

	void saveToTxt(std::string path);

	static void ReceiveHighSpeedData(BYTE* pBuffer, DWORD dwSize, DWORD dwCount, DWORD dwNotify, DWORD dwUser);	
};

#endif