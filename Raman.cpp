#include "Raman.h"

Raman::Raman()
{
	m_pdblRamanShift = new double[2048];
	m_pdblRamanShift_bk = new double[2048];
	m_pdblBackground = new double[2048];
	m_pdblPower = new double[2048];

	initRaman();
}

Raman::~Raman()
{
	DLL_Close_System();
	delete[] m_pdblRamanShift;
	delete[] m_pdblRamanShift_bk;
	delete[] m_pdblBackground;
	delete[] m_pdblPower;
}


int Raman::initRaman()
{
	memset(m_pdblRamanShift, 0, 8 * 2048);
	memset(m_pdblRamanShift_bk, 0, 8 * 2048);
	memset(m_pdblPower, 0, 8 * 2048);
	memset(m_pdblBackground, 0, 8 * 2048);

	int		nErrorCode;
	char	szWorkDir[512];
	DWORD	dwDllVersion;

	GetCurrentDirectoryA(512, szWorkDir);

	std::cout << szWorkDir << "\n";

	dwDllVersion = DLL_Get_Version();

	std::cout << "dwDllVersion " << dwDllVersion << " init..." << "\n";

	nErrorCode = DLL_Init_System(szWorkDir);

	if (nErrorCode != 0)
	{
		std::cout << "Raman init fail\n";
		return -1;
	}

	// set external optical fiber probe sample port
	nErrorCode = DLL_Set_Sample_Port(1);
	if (nErrorCode != 0)
	{
		std::cout << "set external optical fiber probe port fail\n";
		return -1;
	}
	else
		std::cout << "port set\n";

	BYTE byteControl;

	while (true) {

		// try turn on laser
		//nErrorCode = DLL_Laser_TurnOn(450);

		nErrorCode = DLL_Get_Control_Status(&byteControl);

		if ((byteControl & 0x01) == 0)
		{
			std::cout << "Laser is not armed!\n";
		}
		else {

			std::cout << "laser is armed\n";
			break;
		}

		std::cout << "Press arm laser button and hit enter\n";
		std::getchar();
		Sleep(3000);
	}

	return 0;
}

// wavelength: 0=532nm, 1=1064nm
int Raman::getSpectrum(WORD wavelength, DWORD integration_time)
{
	if (!(wavelength == 0 || wavelength == 1) ) {
		std::cout << "wavelength parameter wrong\n";
		return -1;
	}

	if (integration_time > 8000) {

		std::cout << "integration time > 8000 ms\n";
		return -1;
	}

	int		nErrorCode, nLaserOn;
	BYTE	byteControl;

	nErrorCode = DLL_Get_Control_Status(&byteControl);

	if (nErrorCode != 0)
	{
		std::cout<<"Get control panel status failed!\n";
		return -1;
	}

	if ((byteControl & 0x01) == 0)
	{
		std::cout<<"Laser is not armed!\n";
		return -1;
	}

	if ((byteControl & 0x80) != 0)
	{
		std::cout<<"Cover of sample chamber opened!\n";
		return -1;
	}

	// set the longer wavelength band 1064nm
	nErrorCode = DLL_Set_Wavelength_Band(wavelength);
	if (nErrorCode != 0)
	{
		std::cout << "set wavelength band fail\n";
		return -1;
	}
	else std::cout << "wavelength set to "<<wavelength<<"\n";

	// set integration time
	nErrorCode = DLL_Set_Integration_Time(integration_time);
	if (nErrorCode != 0)
	{
		std::cout << "set integration time fail\n";
		return -1;
	}
	else std::cout << "integration time set "<< integration_time<<" ms\n";

	//WORD laser_power = wavelength == 0 ? 100 : 450;
	WORD laser_power = 100;

	nErrorCode = DLL_Laser_TurnOn(laser_power);
	if (nErrorCode != 0)
	{
		std::cout << "turn on laser fail\n";
		return -1;
	}

	nLaserOn = DLL_Get_Laser_Status();
	if (nLaserOn != 1)
	{
		std::cout<<"Laser is OFF\n";
		return -1;
	}

	//turn on laser and wait 3 s
	Sleep(3000);

	memset(m_pdblRamanShift, 0, 8 * 2048);
	memset(m_pdblPower, 0, 8 * 2048);

	nErrorCode = DLL_Acquire_Spectrum(1, &m_dataCount, m_pdblRamanShift, m_pdblPower);

	if (nErrorCode != 0)
	{
		std::cout<<"Get spectrum failed! Error code: "<<nErrorCode<<"\n";
		return -1;
	}

	nErrorCode = DLL_Laser_TurnOff();
	if (nErrorCode != 0)
	{
		std::cout << "turn off laser fail\n";
		return -1;
	}

	std::cout << "Getting background data...\n";

	memset(m_pdblRamanShift_bk, 0, 8 * 2048);
	memset(m_pdblBackground, 0, 8 * 2048);

	nErrorCode = DLL_Acquire_Spectrum(1, &m_dataCount, m_pdblRamanShift_bk, m_pdblBackground);

	if (nErrorCode != 0)
	{
		std::cout << "Get spectrum failed! Error code: " << nErrorCode << "\n";
		return -1;
	}

	//subtract background
	for (WORD i = 0; i < m_dataCount; i++)
		m_pdblPower[i] -= m_pdblBackground[i];

	nErrorCode = DLL_Remove_Baseline(m_dataCount, m_pdblPower);

	if (nErrorCode != 0)
	{
		std::cout << "Remove baseline failed!\n";
		return -1;
	}

#if 0
	std::cout << "Index, Raman Shift (cm^-1), Power (Count)\n";

	for (WORD i = 0; i < m_dataCount; i++)
		std::cout << i << ", " << m_pdblRamanShift[i] << ", " << m_pdblPower[i] << "\n";// << ", " << m_pdblRamanShift_bk[i] << ", " << m_pdblBackground[i] << "\n";
#endif

	return 0;
}

int Raman::getBackgroundSpectrum()
{
	int		nErrorCode, nLaserOn;
	BYTE	byteControl;

	nErrorCode = DLL_Get_Control_Status(&byteControl);

	if (nErrorCode != 0)
	{
		std::cout<<"Get control panel status failed!\n";
		return -1;
	}

	if ((byteControl & 0x80) != 0)
	{
		std::cout<<"Cover of sample chamber opened!\n";
		return -1;
	}

	nLaserOn = DLL_Get_Laser_Status();

	if (nLaserOn == 0)
	{
		std::cout << "Laser on for background\n";
		return -1;
	}

	memset(m_pdblRamanShift, 0, 8 * 2048);
	memset(m_pdblBackground, 0, 8 * 2048);

	nErrorCode = DLL_Acquire_Spectrum(1, &m_dataCount, m_pdblRamanShift, m_pdblBackground);

	if (nErrorCode != 0)
	{
		std::cout<<"Get spectrum failed! Error code: "<<nErrorCode<<"\n";
		
		return -1;
	}

	std::cout << "Index, Raman Shift (cm^-1), Background (Count)\n";
	for (WORD i = 0; i < m_dataCount; i++)
		std::cout << i << ", " << m_pdblRamanShift[i] << ", " << m_pdblBackground[i]<<"\n";
}

void Raman::resetData()
{
	memset(m_pdblRamanShift, 0, 8 * 2048);
	memset(m_pdblRamanShift_bk, 0, 8 * 2048);
	memset(m_pdblBackground, 0, 8 * 2048);
	memset(m_pdblPower, 0, 8 * 2048);
}

void Raman::saveLastSpectrum(std::string path)
{
	std::ofstream file;
	file.open(path);

	if (file.is_open())
	{
		file << "Index, Raman Shift (cm^-1), Power (Count)\n";
		for (int i = 0; i < m_dataCount; i++)
		{
			file<< i << ", " << m_pdblRamanShift[i] << ", " << m_pdblPower[i] << "\n";
		}
	}
}

int Raman::turnOnLaser(int power=490)
{
	int result = DLL_Laser_TurnOn(power);
	if (result != 0)
	{
		std::cout << "laser turn on fail\n";
		return -1;
	}

	Sleep(4000);

	return 0;
}

void Raman::turnOffLaser()
{
	int result = DLL_Laser_TurnOff();
}