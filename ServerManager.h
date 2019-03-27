// need to add "ws2_32.lib" to additional dependencies 

#ifndef SERVER_MANAGER_H_
#define SERVER_MANAGER_H_
#define _CRT_SECURE_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include <iostream>
#include "libssh2_config.h"
#include <libssh2.h>
#include <libssh2_sftp.h>
#include <winsock2.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <ctype.h>
#include <thread>
#include <future>
#include "utilities.h"

#define UPLOAD_BUFFER_SIZE 1 << 15

struct ServerManager 
{

	ServerManager();
	~ServerManager();

	int init();

	int makeServerDirectory(std::string path);

	int uploadDirectory(std::string folder_name, std::string experiment_name="experiment");

	int createNewThreadToUploadDirectory(std::string folder_name, std::string chamber_name, std::string experiment_name);

	int deleteOutdatedData();

	int uploadOneDataFolderFromTheSavedFile();


	unsigned long hostaddr;
	int sock, i, auth_pw = 1;
	struct sockaddr_in sin;
	const char *fingerprint;
	LIBSSH2_SESSION *session = NULL;
	const char *username = "rover";
	const char *password = "R$DR0v$r";

	char mem[UPLOAD_BUFFER_SIZE];

	std::string data_root_dir_ = "C:\\Users\\lietang123\\Documents\\RoAdFiles\\LineProfilerRobotArmTest\\LineProfilerRobotArmTest\\Enviratron_Data\\";

	WSADATA wsadata;
	
	int rc;

	LIBSSH2_SFTP *sftp_session = NULL;
};


#endif 

