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

#include <spdlog/spdlog.h>

#include "ladder.h"
#include "custom_layer.h"
#include "serialport.h"

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

SerialPort serialPort; // serial port instance

std::string frame;
size_t frameSize;
uint16_t frameCRC;
int inputCount = 8;
int outputCount = 8;

bool initSensorData;
std::atomic<std::uint32_t> lastKnownSensorState(0);
std::atomic<std::uint32_t> relayStatus;

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

typedef struct
{
    std::string serialPort;
} configuration;

// Forward declarations
static void parseFrame(std::string & frame);

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
	serialPort.writeString(frame);
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
	inputCount  = frame[4];
	outputCount = frame[5];
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

static int config_handler(void* user, const char* section, const char* name,
                   const char* value)
{
    configuration* pconfig = (configuration*)user;

    if (oplc::ini_matches("comm5", "port", section, name))
        pconfig->serialPort = value;
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Initialization procedures
///
/// This function is called by the main OpenPLC routine when it is initializing.
/// Hardware initialization procedures should be here.
////////////////////////////////////////////////////////////////////////////////
void initializeHardware()
{
	configuration config;

	const char* config_path = oplc::get_config_path();
    if (ini_parse(config_path, config_handler, &config) < 0)
    {
        spdlog::info("Config file {} could not be read, MA-4200 support disabled", config_path);
		return;
    }

	if (!serialPort.open(config.serialPort, 115200))
	{
		spdlog::error("Comm5 MA-4200: Error trying to open serial port\n");
		return;
	}

	spdlog::info("Initializing hardware layer Comm5 MA-4200\n");
	
	currentState = STSTART_OCTET1;
	reqState = Idle;
	lastKnownSensorState = 0;
	frameSize = 0;
	frameCRC = 0;

	serialPort.setReadCallback(ParseData);

	requestProtocolVersion();
	querySensorState();
	enableNotifications();
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Resource clearing
///
/// This function is called by the main OpenPLC routine when it is finalizing.
/// Resource clearing procedures should be here.
////////////////////////////////////////////////////////////////////////////////
void finalizeHardware()
{
	serialPort.close();
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

	for (int i = 0; i < inputCount; i++)
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

	uint32_t bitMask = 0;

	for (int i = 0; i < outputCount; i++)
	{
		if (pinNotPresent(ignored_bool_outputs, ARRAY_SIZE(ignored_bool_outputs), i))
			if (bool_output[i/8][i%8] != NULL) bitWrite(bitMask, i, *bool_output[i/8][i%8]);
	}

	static uint32_t lastOutputStatus = 0;
	if ( lastOutputStatus != bitMask ) {
		std::string data("\x02\x02", 2);
		data.push_back(bitMask);
		writeFrame(data);
		lastOutputStatus = bitMask;
		printf("Wrote to relays: %x\n", (int)bitMask);
	}
}

/** @} */