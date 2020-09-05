// Copyright 2020 Comm5 Tecnologia
//
// This file is part of the OpenPLC Software Stack.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http ://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissionsand
// limitations under the License.

// This file is the hardware layer for the OpenPLC. If you change the platform
// where it is running, you may only need to change this file. All the I/O
// related stuff is here. Basically it provides functions to read and write
// to the OpenPLC internal buffers in order to update I/O state.
// Thiago Alves, Dec 2015

#include <mutex>
#include <string>
#include <atomic>
#include <memory>
#include <thread>

//#include <wiringSerial.h>
#include <Windows.h>

#include <spdlog/spdlog.h>

static void serialClose(uintptr_t fd) {
	HANDLE io_handler_ = (HANDLE)fd;
	CloseHandle(io_handler_);
}

static uintptr_t serialOpen(const char* device, const int baud)
{
	std::string deviceName = "\\\\.\\";
	deviceName.append(device);

	auto io_handler_ = CreateFileA(static_cast<LPCSTR>(deviceName.c_str()),
							GENERIC_READ | GENERIC_WRITE,
							0,
							NULL,
							OPEN_EXISTING,
							FILE_ATTRIBUTE_NORMAL,
							NULL);

	if (io_handler_ == INVALID_HANDLE_VALUE) {

		if (GetLastError() == ERROR_FILE_NOT_FOUND)
			printf("Warning: Handle was not attached. Reason: %s not available\n", device);
		return -1;
	}
	else {

		DCB dcbSerialParams = { 0 };

		if (!GetCommState(io_handler_, &dcbSerialParams)) {

			printf("Warning: Failed to get current serial params");
		}

		else {
			dcbSerialParams.BaudRate = baud;
			dcbSerialParams.ByteSize = 8;
			dcbSerialParams.StopBits = ONESTOPBIT;
			dcbSerialParams.Parity = NOPARITY;
			dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;

			if (!SetCommState(io_handler_, &dcbSerialParams))
				printf("Warning: could not set serial port params\n");
			else {
				PurgeComm(io_handler_, PURGE_RXCLEAR | PURGE_TXCLEAR);				
			}
		}
	}
    return (uintptr_t)io_handler_;
}

static void serialPutchar(const uintptr_t fd, const unsigned char c) {
	HANDLE io_handler_ = (HANDLE)fd;
	DWORD bytes_sent;
	WriteFile(io_handler_, (void*)&c, 1, &bytes_sent, NULL);
}

static int serialDataAvail(const uintptr_t fd)
{
	HANDLE io_handler_ = (HANDLE)fd;
	COMSTAT status_;
	DWORD errors_;

	ClearCommError(io_handler_, &errors_, &status_);

    return status_.cbInQue;
}

static int serialGetchar(const uintptr_t fd)
{
	HANDLE io_handler_ = (HANDLE)fd;
	char inc_msg[1];
	DWORD bytes_read;
	ReadFile(io_handler_, inc_msg, 1, &bytes_read, NULL);
    return inc_msg[0];
}

#include "ladder.h"
#include "custom_layer.h"

#if !defined(ARRAY_SIZE)
    #define ARRAY_SIZE(x) (sizeof((x)) / sizeof((x)[0]))
#endif

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

/** @addtogroup comm5ma4200  MA-4200 I/O Module from Comm5
  * \brief MA-4200 is a 8 Digital Inputs and 8 Relay outputs device from Comm5.
  * \ingroup hardware_layers
  *  @{ */

int serialFd; //serial file descriptor
volatile bool keepRunning; // Flag to signal working thread to quit

std::string frame;
size_t frameSize;
uint16_t frameCRC;

bool initSensorData;
std::atomic<std::uint32_t> lastKnownSensorState(0);
std::atomic<std::uint32_t> relayStatus;

std::shared_ptr<std::thread> m_thread;

enum {
	STSTART_OCTET1,
	STSTART_OCTET2,
	STFRAME_SIZE,
	STFRAME_CONTROL,
	STFRAME_DATA,
	STFRAME_CRC1,
	STFRAME_CRC2
} currentState;

enum RequestState {
	Idle,
	QueryRelayState,
	QuerySensorState
} reqState;

// Forward declarations
static void parseFrame(std::string & frame);

static void write(std::string& data)
{
	for( int i = 0; i < data.size(); ++i ) {
		serialPutchar(serialFd, (unsigned char)data.at(i));
		//printf("Write byte %x\n", ((int)data.at(i)) & 0xFF);
	}
}

static uint16_t crc16_update(uint16_t crc, uint8_t data)
{
#define POLYNOME 0x13D65
	int i;

	crc = crc ^ ((uint16_t)data << 8);
	for (i = 0; i<8; i++)
	{
		if (crc & 0x8000)
			crc = (crc << 1) ^ POLYNOME;
		else
			crc <<= 1;
	}

	return crc;
#undef POLYNOME
}

static void writeFrame(const std::string & data)
{
	char length = (uint8_t)data.size();
	std::string frame("\x05\x64", 2);
	frame.push_back(length);
	frame.push_back(0x00);
	frame.append(data);
	uint16_t crc = 0;
	for (size_t i = 0; i < frame.size(); ++i)
		crc = crc16_update(crc, frame.at(i));
	crc = crc16_update(crc, 0);
	crc = crc16_update(crc, 0);

	frame.append(1, ( crc >> 8 ) & 0xFF);
	frame.append(1, ( crc      ) & 0xFF);
	write(frame);
}

static void querySensorState()
{
	//                  id    mask
	std::string data("\x01\xFF", 2);
	writeFrame(data);
}

void enableNotifications()
{
	//                 id  op  on/off  mask
	std::string data("\x04\x02\x01\xFF", 4);
	writeFrame(data);
}

static void requestProtocolVersion() 
{
	//                  id
	std::string data("\x00", 1);
	writeFrame(data);
}

static void ParseData(const unsigned char* data, const size_t len)
{

	uint16_t readCRC;
	for (size_t i = 0; i < len; ++i) {
		switch ( currentState ) {
		case STSTART_OCTET1:
			frame.clear();
			if (data[i] == 0x05) {
				frame.push_back(data[i]);
				frameCRC = crc16_update(0, data[i]);
				currentState = STSTART_OCTET2;

			}
			else {
				spdlog::error("Comm5 MA-4200: Framing error\n");
				currentState = STSTART_OCTET1;
			}
			break;

		case STSTART_OCTET2:
			if (data[i] == 0x64) {
				frame.push_back(data[i]);
				frameCRC = crc16_update(frameCRC, data[i]);
				currentState = STFRAME_SIZE;
			}
			else {
				spdlog::error("Comm5 MA-4200: Framing error\n");
				currentState = STSTART_OCTET1;
			}
			break;

		case STFRAME_SIZE:
			frameSize = data[i] + 2 + 1 + 1; // FrameSize + Start Tokens + Control Byte + Frame Size field
			frame.push_back(data[i]);
			frameCRC = crc16_update(frameCRC, data[i]);
			currentState = STFRAME_CONTROL;
			break;

		case STFRAME_CONTROL:
			frame.push_back(data[i]);
			frameCRC = crc16_update(frameCRC, data[i]);
			currentState = STFRAME_DATA;
			break;

		case STFRAME_DATA:
			frame.push_back(data[i]);
			frameCRC = crc16_update(frameCRC, data[i]);
			
			if ( frame.size() >= frameSize )
				currentState = STFRAME_CRC1;
			break;
		
		case STFRAME_CRC1:
			frame.push_back(data[i]);
			frameCRC = crc16_update(frameCRC, 0);
			currentState = STFRAME_CRC2;
			break;

		case STFRAME_CRC2:
			frame.push_back(data[i]);
			frameCRC = crc16_update(frameCRC, 0);
			readCRC = frame.at(frame.size() - 2);
			readCRC <<= 8;
			readCRC |= data[i];
			if (frameCRC == readCRC)
				parseFrame(frame);
			else
				spdlog::error("Comm5 MA-4200: CRC Framing error read {} expected {}\n", readCRC, frameCRC);
			currentState = STSTART_OCTET1;
			frame.clear();
			break;
		}

	}
}

void requestDigitalOutputResponseHandler(const std::string & frame)
{
	//  0     1     2       3       4        5        6
	// 0x05 0x64 <size> <control> <id> <operation> <value>

	relayStatus = frame[6];
	for (int i = 0; i < 8; ++i) {
		bool on = (relayStatus & (1 << i)) != 0 ? true : false;
		//SendSwitch(i + 1, 1, 255, on, 0, "Relay " + boost::lexical_cast<std::string>(i + 1));
	}
	printf("requestDigitalOutputResponseHandler: %x", (int)relayStatus);
}

void enableNotificationResponseHandler(const std::string & frame)
{
	uint8_t operation = frame[5];
	uint8_t operationType = frame[6];
	uint8_t mask = frame[7];

	printf("NotificationResponseHandler: %x", (int)mask);
}

void requestDigitalInputResponseHandler(const std::string & frame)
{
	uint8_t mask = frame[5];
	uint8_t sensorStatus = frame[6];

	printf("requestDigitalInputResponseHandler: %x\n", (int)sensorStatus);
	printf("requestDigitalInputResponseHandler: %x mask\n", (int)mask);
	lastKnownSensorState.store(sensorStatus);
	initSensorData = false;
	reqState = Idle;
}

void requestProtocolVersionResponseHandler(const std::string & frame)
{
	printf("requestProtocolVersionResponseHandler\n");
}

static void parseFrame(std::string & frame)
{
	printf("Received frame: %x\n", (int)frame.at(4));
	switch (frame.at(4)) {
	case 0x00:
		requestProtocolVersionResponseHandler(frame);
		break;
	case 0x01:
		requestDigitalInputResponseHandler(frame);
		break;
	case 0x02:
		requestDigitalOutputResponseHandler(frame);
		break;
	case 0x04:
		enableNotificationResponseHandler(frame);
		break;
	}
}

void handleSerialRead() {
	keepRunning = true;

	requestProtocolVersion();
	querySensorState();
	enableNotifications();
	while(keepRunning) {
		if ( serialDataAvail(serialFd) > 0 ) {
			unsigned char data = serialGetchar(serialFd);
			printf("Received %x\n", (int)data);
			ParseData(&data, 1);
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		//querySensorState();
	}

}

////////////////////////////////////////////////////////////////////////////////
/// \brief Initialization procedures
///
/// This function is called by the main OpenPLC routine when it is initializing.
/// Hardware initialization procedures should be here.
////////////////////////////////////////////////////////////////////////////////
void initializeHardware()
{
	serialFd = serialOpen("COM4", 115200);
	if (serialFd < 0)
	{
		printf("Error trying to open serial port\n");
		exit(1);
	}

	printf("Inicializando hardware layer Comm5 MA-4200\n");
	
	currentState = STSTART_OCTET1;
	reqState = Idle;
	lastKnownSensorState = 0;
	frameSize = 0;
	frameCRC = 0;

	m_thread = std::make_shared<std::thread>(&handleSerialRead);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Resource clearing
///
/// This function is called by the main OpenPLC routine when it is finalizing.
/// Resource clearing procedures should be here.
////////////////////////////////////////////////////////////////////////////////
void finalizeHardware()
{
	keepRunning = false;
	m_thread->join();
	m_thread.reset();

	serialClose(serialFd);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Update internal buffers
///
/// This function is called by the OpenPLC in a loop. Here the internal buffers
/// must be updated to reflect the actual Input state. The mutex bufferLock
/// must be used to protect access to the buffers on a threaded environment.
////////////////////////////////////////////////////////////////////////////////
void updateBuffersIn()
{
	std::lock_guard<std::mutex> lock(bufferLock); //lock mutex

	auto sensorData = lastKnownSensorState.load();
	//printf("updateBuffersIn: %x\n", sensorData);
	for (int i = 0; i < 8; i++)
	{
		if (pinNotPresent(ignored_bool_inputs, ARRAY_SIZE(ignored_bool_inputs), i))
			if (bool_input[0][i] != NULL) * bool_input[0][i] = (sensorData >> i) & 0x01;
	}
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Update output buffers
///
/// This function is called by the OpenPLC in a loop. Here the internal buffers
/// must be updated to reflect the actual Output state. The mutex bufferLock
/// must be used to protect access to the buffers on a threaded environment.
////////////////////////////////////////////////////////////////////////////////
void updateBuffersOut()
{
	std::lock_guard<std::mutex> lock(bufferLock); //lock mutex

	unsigned char bitMask = 0;

	for (int i = 0; i < 8; i++)
	{
		if (pinNotPresent(ignored_bool_outputs, ARRAY_SIZE(ignored_bool_outputs), i))
			if (bool_output[i/8][i%8] != NULL) bitWrite(bitMask, i, *bool_output[i/8][i%8]);
	}

	static unsigned char lastOutputStatus = 0;
	if ( lastOutputStatus != bitMask ) {
		std::string data("\x02\x02", 2);
		data.push_back(bitMask);
		writeFrame(data);
		lastOutputStatus = bitMask;
		printf("Wrote to relays: %x\n", (int)bitMask);
	}
}

/** @} */