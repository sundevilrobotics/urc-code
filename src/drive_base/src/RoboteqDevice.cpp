#include <iostream>
#include <stdio.h>
#include <string>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sstream>

#ifdef _WIN32
#include <Windows.h>
#endif

#ifdef linux
#include <termios.h>
#include <unistd.h>
#endif

#include "RoboteqDevice.h"
#include "ErrorCodes.h"

using namespace std;

#if _MSC_VER >= 1700 //Visual Studio 2012 or later
#define sprintf sprintf_s
#endif

#define BUFFER_SIZE 1024
#define MISSING_VALUE -1024

RoboteqDevice::RoboteqDevice()
{
	handle = RQ_INVALID_HANDLE;
}
RoboteqDevice::~RoboteqDevice()
{
	Disconnect();
}

bool RoboteqDevice::IsConnected()
{
	return handle != RQ_INVALID_HANDLE;
}

int RoboteqDevice::IssueCommand(string commandType, string command, string args, int waitms, string &response, bool isplusminus)
{
	int status;
	string read;
	response = "";

	if(args == "")
		status = Write(commandType + command + "\r");
	else
		status = Write(commandType + command + " " + args + "\r");

	if(status != RQ_SUCCESS)
		return status;

	sleepms(waitms);

	status = ReadAll(read);
	if(status != RQ_SUCCESS)
		return status;

	if(isplusminus)
	{
		if(read.length() < 2)
			return RQ_INVALID_RESPONSE;

		response = read.substr(read.length() - 2, 1);
		return RQ_SUCCESS;
	}

	string::size_type pos = read.rfind(command + "=");
	if(pos == string::npos)
		return RQ_INVALID_RESPONSE;

	pos += command.length() + 1;

	string::size_type carriage = read.find("\r", pos);
	if(carriage == string::npos)
		return RQ_INVALID_RESPONSE;

	response = read.substr(pos, carriage - pos);

	return RQ_SUCCESS;
}
int RoboteqDevice::IssueCommand(string commandType, string command, int waitms, string &response, bool isplusminus)
{
	return IssueCommand(commandType, command, "", waitms, response, isplusminus);
}

int RoboteqDevice::SetConfig(int configItem, int index, int value)
{
	string response;
	char command[10];
	char args[50];

	if(configItem < 0 || configItem > 255)
		return RQ_INVALID_CONFIG_ITEM;

	sprintf(command, "$%02X", configItem);
	sprintf(args, "%i %i", index, value);
	if(index == MISSING_VALUE)
	{
		sprintf(args, "%i", value);
		index = 0;
	}

	if(index < 0)
		return RQ_INDEX_OUT_RANGE;

	int status = IssueCommand("^", command, args, 10, response, true);
	if(status != RQ_SUCCESS)
		return status;
	if(response != "+")
		return RQ_SET_CONFIG_FAILED;

	return RQ_SUCCESS;
}
int RoboteqDevice::SetConfig(int configItem, int value)
{
	return SetConfig(configItem, MISSING_VALUE, value);
}

int RoboteqDevice::SetCommand(int commandItem, int index, int value)
{
	string response;
	char command[10];
	char args[50];

	if(commandItem < 0 || commandItem > 255)
		return RQ_INVALID_COMMAND_ITEM;

	sprintf(command, "$%02X", commandItem);
	sprintf(args, "%i %i", index, value);
	if(index == MISSING_VALUE)
	{
		if(value != MISSING_VALUE)
			sprintf(args, "%i", value);
		index = 0;
	}

	if(index < 0)
		return RQ_INDEX_OUT_RANGE;

	int status = IssueCommand("!", command, args, 10, response, true);
	if(status != RQ_SUCCESS)
		return status;
	if(response != "+")
		return RQ_SET_COMMAND_FAILED;

	return RQ_SUCCESS;
}
int RoboteqDevice::SetCommand(int commandItem, int value)
{
	return SetCommand(commandItem, MISSING_VALUE, value);
}
int RoboteqDevice::SetCommand(int commandItem)
{
	return SetCommand(commandItem, MISSING_VALUE, MISSING_VALUE);
}

int RoboteqDevice::GetConfig(int configItem, int index, int &result)
{
	string response;
	char command[10];
	char args[50];

	if(configItem < 0 || configItem > 255)
		return RQ_INVALID_CONFIG_ITEM;

	if(index < 0)
		return RQ_INDEX_OUT_RANGE;

	sprintf(command, "$%02X", configItem);
	sprintf(args, "%i", index);

	int status = IssueCommand("~", command, args, 10, response);
	if(status != RQ_SUCCESS)
		return status;

	istringstream iss(response);
	iss>>result;

	if(iss.fail())
		return RQ_GET_CONFIG_FAILED;

	return RQ_SUCCESS;
}
int RoboteqDevice::GetConfig(int configItem, int &result)
{
	return GetConfig(configItem, 0, result);
}

int RoboteqDevice::GetValue(int operatingItem, int index, int &result)
{
	string response;
	char command[10];
	char args[50];

	if(operatingItem < 0 || operatingItem > 255)
		return RQ_INVALID_OPER_ITEM;

	if(index < 0)
		return RQ_INDEX_OUT_RANGE;

	sprintf(command, "$%02X", operatingItem);
	sprintf(args, "%i", index);

	int status = IssueCommand("?", command, args, 10, response);
	if(status != RQ_SUCCESS)
		return status;

	istringstream iss(response);
	iss>>result;

	if(iss.fail())
		return RQ_GET_VALUE_FAILED;

	return RQ_SUCCESS;
}
int RoboteqDevice::GetValue(int operatingItem, int &result)
{
	return GetValue(operatingItem, 0, result);
}

string ReplaceString(string source, string find, string replacement)
{
	string::size_type pos = 0;
    while((pos = source.find(find, pos)) != string::npos)
	{
        source.replace(pos, find.size(), replacement);
        pos++;
    }

	return source;
}

#ifdef _WIN32
int RoboteqDevice::Connect(string port)
{
	if (IsConnected())
	{
		cout << "Device is connected, attempting to disconnect." << endl;
		Disconnect();
	}

	//Open port.
	cout << "Opening port: '" << port << "'...";
	HANDLE h = CreateFileA(port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	handle = (int)h;
	if (h == INVALID_HANDLE_VALUE)
	{
		handle = RQ_INVALID_HANDLE;
		cout << "failed." << endl;
		return RQ_ERR_OPEN_PORT;
	}

	cout << "succeeded." << endl;

	cout << "Initializing port...";
	InitPort();
	cout << "...done." << endl;

	int status;
	string response;
	cout << "Detecting device version...";
	status = IssueCommand("?", "$1E", 10, response);
	//status = IssueCommand("?", "FID", 10, response);
	if (status != RQ_SUCCESS)
	{
		cout << "failed (issue ?$1E response: " << status << ")." << endl;
		Disconnect();
		return RQ_UNRECOGNIZED_DEVICE;
	}

	if (response.length() < 12)
	{
		cout << "failed (unrecognized version)." << endl;
		Disconnect();
		return RQ_UNRECOGNIZED_VERSION;
	}

	cout << response.substr(8, 4) << "." << endl;
	return RQ_SUCCESS;
}
void RoboteqDevice::Disconnect()
{
	if (IsConnected())
		CloseHandle((HANDLE)handle);

	handle = RQ_INVALID_HANDLE;
}
void RoboteqDevice::InitPort()
{
	if (!IsConnected())
		return;

	DCB          comSettings;
	COMMTIMEOUTS CommTimeouts;

	// Set timeouts in milliseconds
	CommTimeouts.ReadIntervalTimeout = 0;
	CommTimeouts.ReadTotalTimeoutMultiplier = 0;
	CommTimeouts.ReadTotalTimeoutConstant = 100;
	CommTimeouts.WriteTotalTimeoutMultiplier = 0;
	CommTimeouts.WriteTotalTimeoutConstant = 100;
	SetCommTimeouts((HANDLE)handle, &CommTimeouts);

	// Set Port parameters.
	// Make a call to GetCommState() first in order to fill
	// the comSettings structure with all the necessary values.
	// Then change the ones you want and call SetCommState().
	GetCommState((HANDLE)handle, &comSettings);
	comSettings.BaudRate = 115200;
	comSettings.StopBits = ONESTOPBIT;
	comSettings.ByteSize = 8;
	comSettings.Parity = NOPARITY;
	comSettings.fParity = FALSE;
	SetCommState((HANDLE)handle, &comSettings);
}
int RoboteqDevice::Write(string str)
{
	if (!IsConnected())
		return RQ_ERR_NOT_CONNECTED;

	//cout<<"Writing: "<<ReplaceString(str, "\r", "\r\n");
	DWORD written = 0;
	int bStatus = WriteFile((HANDLE)handle, str.c_str(), str.length(), &written, NULL);
	if (bStatus == 0)
		return RQ_ERR_TRANSMIT_FAILED;

	return RQ_SUCCESS;
}
int RoboteqDevice::ReadAll(string &str)
{
	DWORD countRcv;
	if (!IsConnected())
		return RQ_ERR_NOT_CONNECTED;

	char buf[BUFFER_SIZE + 1] = "";

	str = "";
	int i = 0;

	while (ReadFile((HANDLE)handle, buf, BUFFER_SIZE, &countRcv, NULL) != 0)
	{
		str.append(buf, countRcv);

		//No further data.
		if (countRcv < BUFFER_SIZE)
			break;
	}

	//if(countRcv > 0)
	//{
	//	str.append(buf, countRcv);
	//}

	return RQ_SUCCESS;
}
void sleepms(int milliseconds)
{
	Sleep(milliseconds);
}
#endif

#ifdef linux
int RoboteqDevice::Connect(string port)
{
	if (IsConnected())
	{
		cout << "Device is connected, attempting to disconnect." << endl;
		Disconnect();
	}

	//Open port.
	cout << "Opening port: '" << port << "'...";
	handle = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (handle == RQ_INVALID_HANDLE)
	{
		cout << "failed." << endl;
		return RQ_ERR_OPEN_PORT;
	}

	cout << "succeeded." << endl;
	fcntl(handle, F_SETFL, O_APPEND | O_NONBLOCK & ~FNDELAY);

	cout << "Initializing port...";
	InitPort();
	cout << "...done." << endl;

	int status, i;
	string response;
	cout << "Detecting device version...";

	for (i = 0; i < 5; i++)
	{
		status = IssueCommand("?", "$1E", 50, response);
		if (status == RQ_SUCCESS)
			break;
	}

	if (status != RQ_SUCCESS)
	{
		cout << "failed (issue ?$1E response: " << status << ")." << endl;
		Disconnect();
		return RQ_UNRECOGNIZED_DEVICE;
	}

	if (response.length() < 12)
	{
		cout << "failed (unrecognized version)." << endl;
		Disconnect();
		return RQ_UNRECOGNIZED_VERSION;
	}

	cout << response.substr(8, 4) << "." << endl;
	return RQ_SUCCESS;
}
void RoboteqDevice::Disconnect()
{
	if (IsConnected())
		close(handle);

	handle = RQ_INVALID_HANDLE;
}
void RoboteqDevice::InitPort()
{
	if (!IsConnected())
		return;

	//Get the existing Comm Port Attributes in cwrget
	int BAUDRATE = B115200;
	struct termios newtio;
	tcgetattr(handle, &newtio);

	//Set the Tx and Rx Baud Rate to 115200
	cfsetospeed(&newtio, (speed_t)BAUDRATE);
	cfsetispeed(&newtio, (speed_t)BAUDRATE);

	//Enable the Receiver and  Set local Mode
	newtio.c_iflag = IGNBRK;		/* Ignore Break Condition & no processing under input options*/
	newtio.c_lflag = 0;			/* Select the RAW Input Mode through Local options*/
	newtio.c_oflag = 0;			/* Select the RAW Output Mode through Local options*/
	newtio.c_cflag |= (CLOCAL | CREAD);	/* Select the Local Mode & Enable Receiver through Control options*/

										//Make RAW Mode more explicit by turning Canonical Mode off, Echo off, Echo Erase off and Signals off*/
	newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	//Disable Software Flow Control
	newtio.c_iflag &= ~(IXON | IXOFF | IXANY);

	//Set Data format to 8N1
	newtio.c_cflag &= ~CSIZE;		/* Mask the Character Size Bits through Control options*/
	newtio.c_cflag |= CS8;			/* Select Character Size to 8-Bits through Control options*/
	newtio.c_cflag &= ~PARENB;		/* Select Parity Disable through Control options*/
	newtio.c_cflag &= ~PARODD;		/* Select the Even Parity (Disabled) through Control options*/
	newtio.c_cflag &= ~CSTOPB;		/*Set number of Stop Bits to 1*/

									//Timout Parameters. Set to 0 characters (VMIN) and 10 second (VTIME) timeout. This was done to prevent the read call from blocking indefinitely.*/
	newtio.c_cc[VMIN] = 0;
	newtio.c_cc[VTIME] = 100;

	/* Flush the Input buffer and set the attribute NOW without waiting for Data to Complete*/
	tcflush(handle, TCIFLUSH);
	tcsetattr(handle, TCSANOW, &newtio);
}
int RoboteqDevice::Write(string str)
{
	if (!IsConnected())
		return RQ_ERR_NOT_CONNECTED;

	//cout<<"Writing: "<<ReplaceString(str, "\r", "\r\n");
	int countSent = write(handle, str.c_str(), str.length());

	//Verify weather the Transmitting Data on UART was Successful or Not
	if (countSent < 0)
		return RQ_ERR_TRANSMIT_FAILED;

	return RQ_SUCCESS;
}
int RoboteqDevice::ReadAll(string &str)
{
	int countRcv;
	if (!IsConnected())
		return RQ_ERR_NOT_CONNECTED;

	char buf[BUFFER_SIZE + 1] = "";

	str = "";
	int i = 0;
	while ((countRcv = read(handle, buf, BUFFER_SIZE)) > 0)
	{
		str.append(buf, countRcv);

		//No further data.
		if (countRcv < BUFFER_SIZE)
			break;
	}

	if (countRcv < 0)
	{
		if (errno == EAGAIN)
			return RQ_ERR_SERIAL_IO;
		else
			return RQ_ERR_SERIAL_RECEIVE;
	}

	return RQ_SUCCESS;
}
void sleepms(int milliseconds)
{
	usleep(milliseconds / 1000);
}
#endif
