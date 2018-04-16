#ifndef __RAMAN_H_
#define __RAMAN_H_
#include <Windows.h>
#include "AgilityDll.h"
#include <iostream>
#include <fstream>


class Raman
{
private:

public:

	double*		m_pdblRamanShift;
	double*		m_pdblRamanShift_bk;
	double*		m_pdblBackground;
	double*		m_pdblPower;
	UINT		m_wLaserPower;
	DWORD		m_dwIntegrationTime;
	double		m_dblThreshold;
	WORD		m_dataCount;

	Raman();
	~Raman();

	int initRaman();

	int getSpectrum(WORD wavelength, DWORD integration_time);

	int getBackgroundSpectrum();

	void resetData();

	void saveLastSpectrum(std::string path);

	int turnOnLaser(int power);
	
	void turnOffLaser();

};



#endif // !__RAMAN_H_
