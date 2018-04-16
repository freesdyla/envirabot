#ifndef MINIPAM_H_
#define MINIPAM_H_

#include <Windows.h>
#include <ddeml.h>
#include <stdio.h>


HDDEDATA CALLBACK DdeCallback(
	UINT uType,     // Transaction type.
	UINT uFmt,      // Clipboard data format.
	HCONV hconv,    // Handle to the conversation.
	HSZ hsz1,       // Handle to a string.
	HSZ hsz2,       // Handle to a string.
	HDDEDATA hdata, // Handle to a global memory object.
	DWORD dwData1,  // Transaction-specific data.
	DWORD dwData2)  // Transaction-specific data.
{
	return 0;
}

#if 0
void DDEExecute(DWORD idInst, HCONV hConv, wchar_t* szCommand)
{
	HDDEDATA hData = DdeCreateDataHandle(idInst, (LPBYTE)szCommand,
		lstrlen(szCommand) + 1, 0, NULL, CF_TEXT, 0);
	if (hData == NULL)   {
		printf("Command failed: %s\n", szCommand);
	}
	else    {
		DdeClientTransaction((LPBYTE)hData, 0xFFFFFFFF, hConv, 0L, 0,
			XTYP_EXECUTE, TIMEOUT_ASYNC, NULL);
	}
}
#endif

void DDERequest(DWORD idInst, HCONV hConv, wchar_t* szItem)
{
	HSZ hszItem = DdeCreateStringHandle(idInst, szItem, 0);
	HDDEDATA hData = DdeClientTransaction(NULL, 0, hConv, hszItem, CF_TEXT,
		XTYP_REQUEST, 5000, NULL);
	if (hData == NULL)
	{
		printf("Request failed: %s\n", szItem);
	}
	else
	{
		char szResult[255];
		DdeGetData(hData, (unsigned char *)szResult, 255, 0);
		printf("%s\n", szResult);
	}
}

int minpam_get()
{
	//DDE Initialization
	DWORD idInst = 0;
	UINT iReturn;
	iReturn = DdeInitialize(&idInst, (PFNCALLBACK)DdeCallback,
		APPCLASS_STANDARD | APPCMD_CLIENTONLY, 0);

	if (iReturn != DMLERR_NO_ERROR)
	{
		printf("DDE Initialization Failed: 0x%04x\n", iReturn);
		Sleep(1500);
		return -1;
	}

	//DDE Connect to Server using given AppName and topic.
	HSZ hszApp, hszTopic;
	HCONV hConv;
	hszApp = DdeCreateStringHandle(idInst, L"WinControl", 0);
	hszTopic = DdeCreateStringHandle(idInst, L"WinControl", 0);
	hConv = DdeConnect(idInst, hszApp, hszTopic, NULL);
	DdeFreeStringHandle(idInst, hszApp);						
	DdeFreeStringHandle(idInst, hszTopic);
	if (hConv == NULL)
	{
		printf("DDE Connection Failed.\n");
		Sleep(1500); DdeUninitialize(idInst);
		return -1;
	}

	//Execute commands/requests specific to the DDE Server.
	//DDEExecute(idInst, hConv, "S=1");
	DDERequest(idInst, hConv, L"cFt");
	//DDERequest(idInst, hConv, szItem2, szDesc2);
	//DDEPoke(idInst, hConv, szItem3, szData3);
	//DDEExecute(idInst, hConv, szCmd2);

	//DDE Disconnect and Uninitialize.
	DdeDisconnect(hConv);
	DdeUninitialize(idInst);

	Sleep(3000);
	return 1;
	
}

#endif