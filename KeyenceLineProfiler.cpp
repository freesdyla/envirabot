#include "KeyenceLineProfiler.h"

LJV7IF_PROFILE_INFO KeyenceLineProfiler::m_profileInfo;

std::vector<PROFILE_DATA> KeyenceLineProfiler::m_vecProfileData;

clock_t KeyenceLineProfiler::tic;

void KeyenceLineProfiler::init()
{
	req.bySendPos = 2;	// send next available profile after start

	dwThreadId = DEVICE_ID;

	int nRc = LJV7IF_RC_OK;
	// Initialize the Dll
	nRc = LJV7IF_Initialize();

	if (nRc == LJV7IF_RC_OK)
	{
		//std::cout << "init dll success" << std::endl;
		device_initialized = true;
	}
	else
	{
		std::cout << "init dll fail" << std::endl;
		device_initialized = false;
	}

	nRc = LJV7IF_UsbOpen(DEVICE_ID);

	if (nRc == LJV7IF_RC_OK)
	{
		//std::cout << "usb open success" << std::endl;
		device_initialized = true;
	}
	else
	{
		std::cout << "usb open fail" << std::endl;
		device_initialized = false;
	}
}

void KeyenceLineProfiler::finalize()
{
	// clear data
	m_vecProfileData.clear();

	int nRc = LJV7IF_RC_OK;

	// Close the communication
	nRc = LJV7IF_CommClose(DEVICE_ID);

	if (nRc == LJV7IF_RC_OK)
		std::cout << "close comm success" << std::endl;
	else
		std::cout << "close comm fail" << std::endl;

	// Finalize the DLL
	nRc = LJV7IF_Finalize();

	if (nRc == LJV7IF_RC_OK)
		std::cout << "finalize success" << std::endl;
	else
		std::cout << "finalize fail" << std::endl;

	device_initialized = false;
}

void KeyenceLineProfiler::start(DWORD dwProfileCnt)
{
	//m_vecProfileData.clear();

	//nRc = LJV7IF_RC_OK;

	// start high-speed data communication
	//nRc = LJV7IF_StopHighSpeedDataCommunication(DEVICE_ID);

	/*if (nRc != LJV7IF_RC_OK)
		std::cout << "stop before start fail" << std::endl;*/

	//nRc = LJV7IF_HighSpeedDataCommunicationFinalize(DEVICE_ID);

	/*if (nRc != LJV7IF_RC_OK)
		std::cout << "finalize before start fail" << std::endl;*/

	// Open the communication path
	//DWORD dwProfileCnt = 1000;

	//nRc = LJV7IF_HighSpeedDataUsbCommunicationInitalize(DEVICE_ID, ReceiveHighSpeedData, dwProfileCnt, dwThreadId);

	LJV7IF_HighSpeedDataUsbCommunicationInitalize(DEVICE_ID, ReceiveHighSpeedData, dwProfileCnt, dwThreadId);

	/*if (nRc != LJV7IF_RC_OK)
	{
		std::cout << "high speed usb start start fail" << std::endl;
		return;
	}*/

	// High-speed data communication start preparations
	//nRc = LJV7IF_PreStartHighSpeedDataCommunication(DEVICE_ID, &req, &m_profileInfo);
	LJV7IF_PreStartHighSpeedDataCommunication(DEVICE_ID, &req, &m_profileInfo);

	/*if (nRc != LJV7IF_RC_OK)
	{
		std::cout << "pre start high speed fail" << std::endl;
		return;
	}*/

	// Start high-speed data communication.
	//nRc = LJV7IF_StartHighSpeedDataCommunication(DEVICE_ID);
	LJV7IF_StartHighSpeedDataCommunication(DEVICE_ID);

	//tic = clock();

	//if (nRc != LJV7IF_RC_OK)
	/*{
		std::cout << "start high speed fail" << std::endl;
		return;
	}*/
}

/*
 callack function(receive profile data)
 @param Pointer for profile data
 @param One profile data size
 @param Profile count
 @param notify
 @param UserID
*/
void KeyenceLineProfiler::ReceiveHighSpeedData(BYTE* pBuffer, DWORD dwSize, DWORD dwCount, DWORD dwNotify, DWORD dwUser)
{
	/*clock_t toc = clock();
	printf("Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);
	tic = toc;*/

	int nProfDataCnt = (dwSize - sizeof(LJV7IF_PROFILE_HEADER)-sizeof(LJV7IF_PROFILE_FOOTER)) / sizeof(DWORD);

	for (DWORD i = 0; i < dwCount; i++)
	{
		BYTE *pbyBlock = pBuffer + dwSize * i;

		LJV7IF_PROFILE_HEADER* pHeader = (LJV7IF_PROFILE_HEADER*)pbyBlock;
		int* pnProfileData = (int*)(pbyBlock + sizeof(LJV7IF_PROFILE_HEADER));
		LJV7IF_PROFILE_FOOTER* pFooter = (LJV7IF_PROFILE_FOOTER*)(pbyBlock + dwSize - sizeof(LJV7IF_PROFILE_FOOTER));

		m_vecProfileData.push_back(PROFILE_DATA(m_profileInfo, pHeader, pnProfileData, pFooter));
	}

	//std::cout << "dwCount: " << dwCount << " ";// << " size: " << m_vecProfileData.size() << std::endl;
}

void KeyenceLineProfiler::stop()
{
	int nRc = LJV7IF_RC_OK;

	nRc = LJV7IF_StopHighSpeedDataCommunication(DEVICE_ID);

	if (nRc != LJV7IF_RC_OK)
		std::cout << "stop after start fail" << std::endl;

	nRc = LJV7IF_HighSpeedDataCommunicationFinalize(DEVICE_ID);

	if (nRc != LJV7IF_RC_OK)
		std::cout << "finalize after start fail" << std::endl;	
}

void KeyenceLineProfiler::saveToTxt(std::string path)
{
	std::ofstream file;
	file.open("result.csv");
	
	if (m_vecProfileData.size()>0)
	for (int i = 0; i < 800; i++)
	{
		file << m_vecProfileData[0].m_pnProfileData[i] << ",";
	}

	file << std::endl;

	file.close();
}


/*
Constructor for PROFILE_DATA
*/
PROFILE_DATA::PROFILE_DATA(const LJV7IF_PROFILE_INFO &profileInfo, const LJV7IF_PROFILE_HEADER *header, const int *data, const LJV7IF_PROFILE_FOOTER *footer)
{
	m_profileInfo = profileInfo;

	m_profileHeader = *header;

	int nReceiveDataSize = profileInfo.wProfDataCnt * profileInfo.byProfileCnt * (profileInfo.byEnvelope + 1);
	m_pnProfileData = new int[nReceiveDataSize];
	memcpy_s(m_pnProfileData, sizeof(int)* nReceiveDataSize, data, sizeof(int)* nReceiveDataSize);

	m_profileFooter = *footer;
}

/*
Copy constructor for PROFILE_DATA
*/
PROFILE_DATA::PROFILE_DATA(const PROFILE_DATA& obj)
{
	m_profileInfo = obj.m_profileInfo;
	m_profileHeader = obj.m_profileHeader;
	m_profileFooter = obj.m_profileFooter;

	int nReceiveDataSize = obj.m_profileInfo.wProfDataCnt * obj.m_profileInfo.byProfileCnt * (obj.m_profileInfo.byEnvelope + 1);
	m_pnProfileData = new int[nReceiveDataSize];
	for (int i = 0; i < nReceiveDataSize; i++)
	{
		m_pnProfileData[i] = obj.m_pnProfileData[i];
	}
}

/*
Assignment operator for PROFILE_DATA
*/
PROFILE_DATA& PROFILE_DATA::operator =(const PROFILE_DATA &obj)
{
	m_profileInfo = obj.m_profileInfo;
	m_profileHeader = obj.m_profileHeader;
	m_profileFooter = obj.m_profileFooter;

	int nReceiveDataSize = obj.m_profileInfo.wProfDataCnt * obj.m_profileInfo.byProfileCnt * (obj.m_profileInfo.byEnvelope + 1);
	m_pnProfileData = new int[nReceiveDataSize];
	for (int i = 0; i < nReceiveDataSize; i++)
	{
		m_pnProfileData[i] = obj.m_pnProfileData[i];
	}

	return *this;
}

/*
Destructor for PROFILE_DATA
*/
PROFILE_DATA::~PROFILE_DATA()
{
	delete[] m_pnProfileData;
}

/*a
ThreadSafeBuffer* ThreadSafeBuffer::m_threadSafeBuffer = 0;

ThreadSafeBuffer::ThreadSafeBuffer(void)aB
{
	m_adwCount = 0; 
	m_adwNotify = 0; 
	m_anBatchNo = 0;
}

ThreadSafeBuffer* ThreadSafeBuffer::getInstance(void)
{
	if (m_threadSafeBuffer == 0)
	{
		m_threadSafeBuffer = new ThreadSafeBuffer();
	}
	return m_threadSafeBuffer;
}
*/