#ifndef _AGILIYDLL_
#define _AGILIYDLL_


//difine the export type
#define DLL_PORT_TYPE __declspec(dllexport)
#define DLL_PORT_FUNCTION_TYPE __stdcall	//this can be used in visual basic program

typedef enum ERROR_CODE 
{
	ERROR_NONE				= 0,			//Operation successfally	
	ERROR_CONFIG_FILE		= 1,			//Open system configuration faile failed 
	ERROR_CONFIG_PARAM		= 2,			//Passed invalid parameters
	ERROR_MAIN_CTRL			= 3,			//Control main control board failed
	ERROR_PANEL_CTRL		= 4,			//Control front panel board failed
	ERROR_DAQ_CTRL			= 5,			//Control data acquisition board failed
	ERROR_LS_CTRL			= 6,			//Control laser board failed
	ERROR_LICENSE_EXPIRE	= 7,			//System license expired
	ERROR_SYS_CLOSE			= 8,			//System not initialized or closed
	ERROR_MEM				= 9				//Allocated memory failed
};


//----------------------------------Export function declaration----------------------------------
/*function
********************************************************************************
<PRE>
Name:			DLL_Get_DLL_Version

Operation:		Get version of Agility DLL

Parameter:		None

Return value:	DWORD version of Agility DLL
-------------------------------------------------------------------------------- 
</PRE>
*******************************************************************************/ 
DLL_PORT_TYPE	DWORD  DLL_PORT_FUNCTION_TYPE DLL_Get_Version();


/*function
********************************************************************************
<PRE>
name:			DLL_Init_System

operation:		Initialize the system 

parameter:		szWorkDir: Current work directory in which system configuration file and license file are located
				

return value:	0	  - Operation successfully
				Other - Operation failed

Note:			For dual band and dual port system, after initialization, the default setting will be short 
				wavelength and internal sample port 
--------------------------------------------------------------------------------
</PRE>
*******************************************************************************/ 
DLL_PORT_TYPE	int  DLL_PORT_FUNCTION_TYPE DLL_Init_System(char* szWorkDir);


/*function
********************************************************************************
<PRE>
name:			DLL_Close_System()

operation:		Close the system

parameter:		None

return value:	None 
--------------------------------------------------------------------------------
</PRE>
*******************************************************************************/ 
DLL_PORT_TYPE	void DLL_PORT_FUNCTION_TYPE DLL_Close_System();

/*function
********************************************************************************
<PRE>
name:			DLL_Get_SN

operation:		Get system's serial number 

parameter:		Char pointer of system serial number buffer. Buffer size >= 9 byte

return value:	0	  - Operation successfully
				Other - Operation failed
-------------------------------------------------------------------------------- 
</PRE>
*******************************************************************************/ 
DLL_PORT_TYPE	int  DLL_PORT_FUNCTION_TYPE DLL_Get_System_SN(LPSTR lpszSN);

/*function
********************************************************************************
<PRE>
name:			DLL_Get_Battery_Status

operation:		Get battery status 

parameter:		pbyteBattery: Byte pointer of the battery status
							  0x01 - The battery is charging
							  0x02 - The battery level is below 5%
							  0x03 - The battery level is above 25%
							  0x04 - The battery level is above 50%
							  0x05 - The battery level is above 75%
							  0x06 - The battery level is 100%

return value:	0	  - Operation successfully
				Other - Operation failed
-------------------------------------------------------------------------------- 
</PRE>
*******************************************************************************/ 
DLL_PORT_TYPE	int  DLL_PORT_FUNCTION_TYPE DLL_Get_Battery_Status(BYTE* pbyteBattery);

/*function
********************************************************************************
<PRE>
name:			DLL_Get_Control_Status

operation:      Get the status of push buttons (Arm laser, Snapshot, Run and Setting) on the front 
				control panel or cover of sample chamber

parameter:		byteControl: Byte pointer of the status of push buttons or cover of sample chamber
							 0x01 - Laser armed
							 0x02 - Systen setting button pushed down
							 0x04 - Snapshot button pushed down
							 0x08 - Run button pushed down
							 0x80 - Cover of sample chamber opened				
							
return value:	0	  - Operation successfully
				Other - Operation failed

Note:			1. The Laser Arm buuton is latching button. In order to turn on the laser, the laser has to be armed first
				2. The push buttons of System Setting, Snapshot and Run are non-latching button. In order to montinor their status, 
				   user apllication program needs to call this function constantly.
			    3. If the cover of sample chamber is opened, the laser will be disarmed. In order to turn on the laser, the cover 
				   of sample chamber has to be closed and the laser has to be armed again
				 
--------------------------------------------------------------------------------
</PRE>
*******************************************************************************/ 
DLL_PORT_TYPE int	DLL_PORT_FUNCTION_TYPE DLL_Get_Control_Status(BYTE* pbyteControl); 

/*function
********************************************************************************
<PRE>
name:			DLL_Set_Wavelength_Band

operation:		Set wavelength band if the system has dual bands  

parameter:		wBandIndex: Wavelength band index
							0 - The shorter wavelength band
							1 - The longer wavelength band

return value:	0	  - Operation successfully
				Other - Operation failed		
-------------------------------------------------------------------------------- 
</PRE>
*******************************************************************************/ 
DLL_PORT_TYPE	int  DLL_PORT_FUNCTION_TYPE DLL_Set_Wavelength_Band(WORD wBandIndex);

/*function
********************************************************************************
<PRE>
name:			DLL_Get_Wavelength_Band

operation:		Get current selected wavelength band index 

parameter:		pwBandIndex: WORD pointer of the wavelength band index
							 0 - The shorter wavelength band
							 1 - The longer wavelength band

return value:	0	  - Operation successfully
				Other - Operation failed					
-------------------------------------------------------------------------------- 
</PRE>
*******************************************************************************/ 
DLL_PORT_TYPE	int  DLL_PORT_FUNCTION_TYPE DLL_Get_Wavelength_Band(WORD* pwBandIndex);

/*function
********************************************************************************
<PRE>
name:			DLL_Set_Sample_Port

operation:		Set sample port if the system has dual ports  

parameter:		wPortIndex: Sample port index
							0 - The internal free space sample port
							1 - The external optical fiber sample port

return value:	0	  - Operation successfully
				Other - Operation failed				
-------------------------------------------------------------------------------- 
</PRE>
*******************************************************************************/ 
DLL_PORT_TYPE	int  DLL_PORT_FUNCTION_TYPE DLL_Set_Sample_Port(WORD wPortIndex);

/*function
********************************************************************************
<PRE>
name:			DLL_Get_Sample_Port

operation:		Get current selected sample port index

parameter:		pwPortIndex: WORD pointer of the sample port index
							 0 - The internal free space sample port
							 1 - The external optical fiber sample port

return value:	0	  - Operation successfully
				Other - Operation failed 				
-------------------------------------------------------------------------------- 
</PRE>
*******************************************************************************/ 
DLL_PORT_TYPE	int  DLL_PORT_FUNCTION_TYPE DLL_Get_Sample_Port(WORD* pwPortIndex);

/*function
********************************************************************************
<PRE>
name:			DLL_Get_HW_FW_Rev

operation:		Get hardware and firmware revisions of the current selected data acquisition board (wavelength band)

parameter:		pdwHW: DWORD pointer of hardware revision
				pdwFW: DWORD pointer of firmware revision
				
return value:   0	  - Operation successfully
				Other - Operation failed
--------------------------------------------------------------------------------
</PRE>
*******************************************************************************/								
DLL_PORT_TYPE	int DLL_PORT_FUNCTION_TYPE DLL_Get_HW_FW_Rev(DWORD* pdwHW, DWORD* pdwFW);

/*function
********************************************************************************
<PRE>
name:			DLL_Get_Pixel_Count

operation:		Get the pixel count of the current selected data acquisition board (wavelength band)  

parameter:		pwPixelCount: WORD pointer of pixel count

return value:	0	  - Operation successfully
				Other - Operation failed
				
-------------------------------------------------------------------------------- 
</PRE>
*******************************************************************************/ 
DLL_PORT_TYPE	int  DLL_PORT_FUNCTION_TYPE DLL_Get_Pixel_Count(WORD* pwPixelCount);

/*function
********************************************************************************
<PRE>
name:			DLL_Set_Integration_Time

operation:		Set integration time of the current selected data acquisition board (wavelength band)

parameter:		dwIntegrationTime: 1 to 3600000, Unit: ms

return value:   0	  - Operation successfully
				Other - Operation failed
--------------------------------------------------------------------------------
</PRE>
*******************************************************************************/								
DLL_PORT_TYPE	int DLL_PORT_FUNCTION_TYPE DLL_Set_Integration_Time(DWORD dwIntegrationTime); 

/*function
********************************************************************************
<PRE>
name:			DLL_Get_Integration_Time

operation:      Get integration time of the current selected data acquisition board (wavelength band)

parameter:		pdwIntegrationTime: DWORD pionter of integration time; Unit: ms

return value:	0	  - Operation successfully
				Other - Operation failed
--------------------------------------------------------------------------------
</PRE>
*******************************************************************************/ 
DLL_PORT_TYPE	int DLL_PORT_FUNCTION_TYPE   DLL_Get_Integration_Time(DWORD* pdwIntegrationTime); 

/*function
********************************************************************************
<PRE>
name:			DLL_Laser_TurnOn

operation:		Turn on laser with setting output power on the current selected sample port

parameter:		wLaserPower: laser output power, Unit: mW

return value:   0	  - Operation successfully
				Other - Operation failed

Note:			After laser turn on, this function will wait 1 seconds to stablize laser power
--------------------------------------------------------------------------------
</PRE>
*******************************************************************************/
DLL_PORT_TYPE int DLL_PORT_FUNCTION_TYPE DLL_Laser_TurnOn(WORD wLaserPower);

/*function
********************************************************************************
<PRE>
name:			DLL_Laser_TurnOff

operation:		Turn off laser on the current selected sample port

parameter:		None

return value:   0	  - Operation successfully
				Other - Operation failed
--------------------------------------------------------------------------------
</PRE>
*******************************************************************************/
DLL_PORT_TYPE int DLL_PORT_FUNCTION_TYPE DLL_Laser_TurnOff();

/*function
********************************************************************************
<PRE>
name:			DLL_Get_Laser_Status

operation:		Get laser status

parameter:		None

return value:   -1	- THe system is not initialized
				0	- Laser is off
				1	- Laser is on
--------------------------------------------------------------------------------
</PRE>
*******************************************************************************/
DLL_PORT_TYPE int DLL_PORT_FUNCTION_TYPE DLL_Get_Laser_Status();

/*function
********************************************************************************
<PRE>
name:			DLL_Acquire_Spectrum

operation:		Acquire one Raman spectrum from the current selected data acquisition board (wavelength band) and sample port

parameter:		wAverageNumber:			Spectrum average number
				pwDataCount:			WORD pionter of data count
				pdblRamanShift:			Double pointer of Raman shift array; Unit: cm^-1; Arrary size >= 2048
				pdblPower:				Double pointer of Raman signal power array; Unit: Count; Arrary size >= 2048

return value:	0	  - Operation successfully
				Other - Operation failed
--------------------------------------------------------------------------------
</PRE>
*******************************************************************************/ 
DLL_PORT_TYPE	int  DLL_PORT_FUNCTION_TYPE DLL_Acquire_Spectrum(WORD	 wAverageNumber, 															
																 WORD*	 pwDataCount, 
																 double* pdblRamanShift, 
																 double* pdblPower);

/*function
********************************************************************************
<PRE>
name:			DLL_Search_Peak
operation:		Search all peaks in the spectrum
parameter:		wDataCount:			Total data count
				dblThreshold:		The peak search threshold. The powers of all peaks > dblThreshold
				pdblRamanShift:		Pointer of double Raman shift array. Array size >= wDataCount
				pdblPower:			Pointer of double Raman signal power array. Array size >= wDataCount									
				pwPeakCount:		Pointer of WORD searched peak count
				pdblPeakRamanShift:	Pointer of double searched peak Raman shift array. Array size >= wPixelCount
				pdblPeakPower:		Pointer of double searched peak Raman signal power array. Array size >= wPixelCount

return value:   0	  - Operation successfully
				Other - Operation failed

Note:			The background spectrum is recommanded to subtract from the Raman spectrum before the peak searching
--------------------------------------------------------------------------------
</PRE>
*******************************************************************************/								
DLL_PORT_TYPE int DLL_PORT_FUNCTION_TYPE DLL_Search_Peaks(WORD		wDataCount,
														  double	dblThreshold,
														  double*	pdblRamanShift,
														  double*	pdblPower,														   
														  WORD*		pwPeakCount,
														  double*	pdblPeakRamanShift,
														  double*	pdblPeakPower); 

/*function
********************************************************************************
<PRE>
name:			DLL_Remove_Baseline

operation:		Remove baseline

parameter:		wDataCount:			total data count
				pdblPower:			Double pointer of Raman signal power array; Unit: Count; Arrary size >= 2048

return value:	0	  - Operation successfully
				Other - Operation failed
--------------------------------------------------------------------------------
</PRE>
*******************************************************************************/ 
DLL_PORT_TYPE	int  DLL_PORT_FUNCTION_TYPE DLL_Remove_Baseline(WORD	wDataCount, 															 
																double* pdblPower);


#endif //_AGILIYDLL_