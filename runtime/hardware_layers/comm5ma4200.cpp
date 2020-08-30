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

#include <wiringSerial.h>

#include "ladder.h"
#include "custom_layer.h"

/** @addtogroup comm5ma4200  MA-4200 I/O Module from Comm5
  * \brief A template with placeholder functions
  * \ingroup hardware_layers
  *  @{ */

int serialFd; //serial file descriptor

enum {
	STSTART_OCTET1,
	STSTART_OCTET2,
	STFRAME_SIZE,
	STFRAME_CONTROL,
	STFRAME_DATA,
	STFRAME_CRC1,
	STFRAME_CRC2
} currentState;

static void write(std::string& data)
{
	for( int i = 0; i < data.size(); ++i ) {
		serialPutchar(serialFd, data.at(i));
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

	frame.append(1, crc >> 8);
	frame.append(1, crc & 0xFF);
	write(frame);
	return true;
}

static void querySensorState()
{
	//                  id    mask
	std::string data("\x01\xFF", 2);
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
				_log.Log(LOG_ERROR, "Comm5 MA-4200: Framing error");
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
				_log.Log(LOG_ERROR, "Comm5 MA-4200: Framing error");
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
			readCRC =  (uint16_t)(frame.at(frame.size() - 2) << 8) | frame.at(frame.size() - 1);
			if (frameCRC == readCRC)
				parseFrame(frame);
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
		SendSwitch(i + 1, 1, 255, on, 0, "Relay " + boost::lexical_cast<std::string>(i + 1));
	}
}

void requestDigitalInputResponseHandler(const std::string & frame)
{
	uint8_t mask = frame[5];
	uint8_t sensorStatus = frame[6];

	for (int i = 0; i < 8; ++i) {
		bool on = (sensorStatus & (1 << i)) != 0 ? true : false;
		if ((lastKnownSensorState & (1 << i) ^ (sensorStatus & (1 << i))) || initSensorData) {
			SendSwitch((i + 1) << 8, 1, 255, on, 0, "Sensor " + boost::lexical_cast<std::string>(i + 1));
		}
	}
	lastKnownSensorState = sensorStatus;
	initSensorData = false;
	reqState = Idle;
}

static void parseFrame(std::string & frame)
{
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

////////////////////////////////////////////////////////////////////////////////
/// \brief Initialization procedures
///
/// This function is called by the main OpenPLC routine when it is initializing.
/// Hardware initialization procedures should be here.
////////////////////////////////////////////////////////////////////////////////
void initializeHardware()
{
	serialFd = serialOpen("/dev/ttyUSB0", 9600);
	if (serialFd < 0)
	{
		printf("Error trying to open serial port\n");
		exit(1);
	}
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Resource clearing
///
/// This function is called by the main OpenPLC routine when it is finalizing.
/// Resource clearing procedures should be here.
////////////////////////////////////////////////////////////////////////////////
void finalizeHardware()
{
	// serialClose(serialFd) // no serialClose for now. Eventually
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

	/*********READING AND WRITING TO I/O**************

	*bool_input[0][0] = read_digital_input(0);
	write_digital_output(0, *bool_output[0][0]);

	*int_input[0] = read_analog_input(0);
	write_analog_output(0, *int_output[0]);

	**************************************************/
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

	/*********READING AND WRITING TO I/O**************

	*bool_input[0][0] = read_digital_input(0);
	write_digital_output(0, *bool_output[0][0]);

	*int_input[0] = read_analog_input(0);
	write_analog_output(0, *int_output[0]);

	**************************************************/
}

/** @} */