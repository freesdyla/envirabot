#ifndef RoboteqDevice_H_
#define RoboteqDevice_H_

using namespace std;

#define STOP_ON_MAIN			1 << 0
#define ON_MAIN					1 << 1	//moving on the main straight tape
#define FORWARD_ON_BRANCH		1 << 2		//moving from main straight to branch
#define REVERSE_ON_BRANCH		1 << 3	//moving from branch back to main straight tape
#define STOP_AT_DOOR			1 << 4
#define STOP_IN_CHAMBER			1 << 5
#define MOVING_ON_CHAMBER_TRACK 1 << 6

string ReplaceString(string source, string find, string replacement);
void sleepms(int milliseconds);

class RoboteqDevice
{
private:
	int device_fd;
	int fd0;
	int handle;

protected:
	void InitPort();

	int Write(string str);
	int ReadAll(string &str);

	int IssueCommand(string commandType, string command, string args, int waitms, string &response, bool isplusminus = false);
	int IssueCommand(string commandType, string command, int waitms, string &response, bool isplusminus = false);

public:
	bool IsConnected();
	int Connect(string port);
	void Disconnect();

	int SetConfig(int configItem, int index, int value);
	int SetConfig(int configItem, int value);

	int SetCommand(int commandItem, int index, int value);
	int SetCommand(int commandItem, int value);
	int SetCommand(int commandItem);

	int GetConfig(int configItem, int index, int &result);
	int GetConfig(int configItem, int &result);

	int GetValue(int operatingItem, int index, int &result);
	int GetValue(int operatingItem, int &result);

	RoboteqDevice();
	~RoboteqDevice();
};

#endif
