#include "ServerManager.h"

ServerManager::ServerManager()
{
	int result = init();

	if (result != 0)
	{
		Utilities::to_log_file("Fail to connect to storage server\n");
	}
}

ServerManager::~ServerManager()
{
	if (sftp_session != NULL && session != NULL)
	{

		libssh2_sftp_shutdown(sftp_session);

		libssh2_session_disconnect(session, "Shut down ssh session");

		libssh2_session_free(session);

		closesocket(sock);

		libssh2_exit();
	}
}

int ServerManager::init() {
	
	// win socket stuff
	int err = WSAStartup(MAKEWORD(2, 0), &wsadata);

	if (err != 0) {

		std::cout<<"WSAStartup failed with error: "<<err<<"\n";

		return 1;
	}

	err = libssh2_init(0);

	if (err != 0) {

		std::cout << "libssh2 init failed with error: " << err << "\n";

		return 2;
	}

	sock = socket(AF_INET, SOCK_STREAM, 0);

	sin.sin_family = AF_INET;
	sin.sin_port = htons(22234);
	sin.sin_addr.s_addr = inet_addr("10.25.215.209");

	if (connect(sock, (struct sockaddr*)(&sin),	sizeof(struct sockaddr_in)) != 0) {
		
		Utilities::to_log_file("failed to connect to storage server!");

		exit(-1);

		return 3;
	}

	/* Create a session instance*/
	session = libssh2_session_init();

	if (!session) 
		return 4;

	/* Since we have not set non-blocking, tell libssh2 we are blocking */
	libssh2_session_set_blocking(session, 1);

	/* ... start it up. This will trade welcome banners, exchange keys,
	* and setup crypto, compression, and MAC layers
	*/
	err = libssh2_session_handshake(session, sock);

	if (err) {

		std::cout<< "Failure establishing SSH session: "<<rc<<"\n";
		return 5;
	}

	if (libssh2_userauth_password(session, username, password)) 
	{

		std::cout<<"Authentication by password failed.\n";

		return 6 ;
	}

	sftp_session = libssh2_sftp_init(session);

	if (!sftp_session) {

		std::cout<<"Unable to init SFTP session\n";
		return 7;
	}

	return 0;
}

int ServerManager::makeServerDirectory(std::string path)
{
	int rc = libssh2_sftp_mkdir(sftp_session, path.c_str(),
		LIBSSH2_SFTP_S_IRWXU |
		LIBSSH2_SFTP_S_IRGRP | LIBSSH2_SFTP_S_IXGRP |
		LIBSSH2_SFTP_S_IROTH | LIBSSH2_SFTP_S_IXOTH);

	if (rc) {
		std::cout << "libssh2_sftp_mkdir failed: " << rc << "\n";
		return -1;
	}

	return 0;
}

int ServerManager::uploadDirectory(std::string folder_name)
{
	std::vector<std::string> str_vec;
	boost::split(str_vec, folder_name, boost::is_any_of("-"));

	std::string chamber_name;

	if (str_vec.size() == 8)
	{
		chamber_name = "Chamber_" + str_vec.front().substr(1, 1);
	}
	else
		return -1;

	std::string absolute_path = data_root_dir_ + chamber_name + "\\" + folder_name;

	std::string linux_server_dir = chamber_name + "/" + folder_name;

	makeServerDirectory(linux_server_dir);

	for (auto & p : boost::filesystem::recursive_directory_iterator(absolute_path)) {

		std::vector<std::string> strs;

		boost::split(strs, p.path().string(), boost::is_any_of("\\"));

		if (boost::filesystem::is_directory(p)) {

			//std::cout << "folder: ";

			if (strs.size() != 0) {

				//std::cout << strs.back() << std::endl;

				std::string destination_dir = folder_name + "/" + strs.back();

				if (makeServerDirectory(destination_dir) == 0) {


				}
			}
	
		}
		else {	//this is a file

			std::string dest = "/home/rover/" + chamber_name + "/"+ folder_name + "/"+ strs.back();

			FILE *file;

			file = fopen(p.path().string().c_str(), "rb");

			if (file != NULL) {

				LIBSSH2_SFTP_HANDLE *sftp_handle = libssh2_sftp_open(sftp_session, dest.c_str(),
					LIBSSH2_FXF_WRITE | LIBSSH2_FXF_CREAT | LIBSSH2_FXF_TRUNC,
					LIBSSH2_SFTP_S_IRUSR | LIBSSH2_SFTP_S_IWUSR |
					LIBSSH2_SFTP_S_IRGRP | LIBSSH2_SFTP_S_IROTH);

				if (sftp_handle) {

					size_t nread;
					char* ptr;
					int rc;

					do {

						nread = fread(mem, 1, sizeof(mem), file);

						if (nread <= 0)	break;

						ptr = mem;

						do {
							rc = libssh2_sftp_write(sftp_handle, ptr, nread);
							if (rc < 0) break;

							ptr += rc;
							nread -= rc;

						} while (nread);
					} while (rc>0);		
				}
				else {

					std::cout << "sftp failed to open destination path\n";
				}
			
				libssh2_sftp_close(sftp_handle);

				fclose(file);
			}
		}

		std::cout << p << std::endl;
	}
	
	return 0;
}

int ServerManager::createNewThreadToUploadDirectory(std::string folder_name, std::string chamber_name)
{
	auto upload_thread = std::async(&ServerManager::uploadDirectory, this, folder_name);

	return upload_thread.get();
}

int ServerManager::deleteOutdatedData() {

	
	for (auto & p : boost::filesystem::recursive_directory_iterator(data_root_dir_)) {

		if (boost::filesystem::is_regular_file(p)) {

			WIN32_FILE_ATTRIBUTE_DATA file_info;

			SYSTEMTIME st;
			FILETIME ft;

			GetSystemTime(&st);

			SystemTimeToFileTime(&st, &ft);

			GetFileAttributesEx(p.path().wstring().c_str(), GetFileExInfoStandard, &file_info);

			ULARGE_INTEGER file_creation_time_utc;
			file_creation_time_utc.LowPart = file_info.ftCreationTime.dwLowDateTime;
			file_creation_time_utc.HighPart = file_info.ftCreationTime.dwHighDateTime;

			ULARGE_INTEGER cur_time_utc;
			cur_time_utc.LowPart = ft.dwLowDateTime;
			cur_time_utc.HighPart = ft.dwHighDateTime;

			LONGLONG time_difference_s = (reinterpret_cast<LARGE_INTEGER*>(&cur_time_utc)->QuadPart
										- reinterpret_cast<LARGE_INTEGER*>(&file_creation_time_utc)->QuadPart)/10000000;

			FileTimeToSystemTime(&file_info.ftCreationTime, &st);

			std::cout << time_difference_s <<" - " << time_difference_s /60.<<" - "<<p.path().string()<<"\n";

			if (time_difference_s > 60) {

				DeleteFile(p.path().wstring().c_str());

			}
		}
	}

	return 0;
}