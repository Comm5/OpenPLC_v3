// Copyright 2020 Comm5 Tecnologia
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

#include "serialport.h"

#include <string>
#include <algorithm>
#include <thread>
#include <mutex>

#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#elif defined(__APPLE__) || defined(__linux__)
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

using namespace std;


class SerialPortImpl
{
public:
    SerialPortImpl(): backgroundThread(), open(false),
            error(false) {}

    std::thread backgroundThread;
    bool open;
    bool error;
    std::string errorMessage;
    mutable std::mutex errorMutex;

#if defined(_WIN32) || defined(__CYGWIN__)
    HANDLE fd; ///< File descriptor for serial port
#elif defined(__APPLE__) || defined(__linux__)
    int fd;
#endif

    unsigned char readBuffer[SerialPort::readBufferSize]; ///< data being read

    /// Read complete callback
    std::function<void (const unsigned char*, size_t)> callback;
};

SerialPort::SerialPort(): pimpl(new SerialPortImpl)
{

}

SerialPort::SerialPort(const std::string& devname, unsigned int baud_rate)
        : pimpl(new SerialPortImpl)
{
    open(devname,baud_rate);
}

bool SerialPort::open(const std::string& devname, unsigned int baud_rate)
{
    if(isOpen()) 
        close();

#if defined(_WIN32) || defined(__CYGWIN__)
	std::string deviceName = "\\\\.\\";
	deviceName.append(devname);

	auto io_handler_ = ::CreateFileA(static_cast<LPCSTR>(deviceName.c_str()),
							GENERIC_READ | GENERIC_WRITE,
							0,
							NULL,
							OPEN_EXISTING,
							FILE_ATTRIBUTE_NORMAL,
							NULL);

	if (io_handler_ == INVALID_HANDLE_VALUE) 
    {
		if (::GetLastError() == ERROR_FILE_NOT_FOUND)
			setErrorMessage("Failed to open port.");
		return false;
	}
	else 
    {
        pimpl->fd = io_handler_;

		DCB dcbSerialParams = { 0 };

		if (!::GetCommState(pimpl->fd, &dcbSerialParams)) 
        {
			setErrorMessage("Failed to get current serial params");
            return false;
		}
		else 
        {
			dcbSerialParams.BaudRate = baud_rate;
			dcbSerialParams.ByteSize = 8;
			dcbSerialParams.StopBits = ONESTOPBIT;
			dcbSerialParams.Parity = NOPARITY;
			dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;

			if (!::SetCommState(pimpl->fd, &dcbSerialParams)) 
            {
				setErrorMessage("Could not set serial port params\n");
                return false;
			} 
            else 
            {
				::PurgeComm(pimpl->fd, PURGE_RXCLEAR | PURGE_TXCLEAR);				
			}
		}
	}

#elif defined(__APPLE__) || defined(__linux__)

    speed_t speed;
    
    // Open port
    auto io_handler_ = ::open(devname.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (io_handler_ < 0) {
        setErrorMessage("Failed to open port");
    }
    else 
    {
        pimpl->fd = io_handler_;
        // Set Port parameters.
        struct termios new_attributes;
        int status = tcgetattr(pimpl->fd, &new_attributes);
        if(status < 0 || !isatty(pimpl->fd))
        {
            ::close(pimpl->fd);
            setErrorMessage("Device is not a tty");
            return false;
        }
        new_attributes.c_iflag = IGNBRK;
        new_attributes.c_oflag = 0;
        new_attributes.c_lflag = 0;
        new_attributes.c_cflag = (CS8 | CREAD | CLOCAL);//8 data bit,Enable receiver,Ignore modem
        /* In non canonical mode (Ctrl-C and other disabled, no echo,...) VMIN and VTIME work this way:
        if the function read() has'nt read at least VMIN chars it waits until has read at least VMIN
        chars (even if VTIME timeout expires); once it has read at least vmin chars, if subsequent
        chars do not arrive before VTIME expires, it returns error; if a char arrives, it resets the
        timeout, so the internal timer will again start from zero (for the nex char,if any)*/
        new_attributes.c_cc[VMIN]=1;// Minimum number of characters to read before returning error
        new_attributes.c_cc[VTIME]=1;// Set timeouts in tenths of second

        // Set baud rate
        switch(baud_rate)
        {
            case 50:speed = B50; break;
            case 75:speed = B75; break;
            case 110:speed = B110; break;
            case 134:speed = B134; break;
            case 150:speed = B150; break;
            case 200:speed = B200; break;
            case 300:speed = B300; break;
            case 600:speed = B600; break;
            case 1200:speed = B1200; break;
            case 1800:speed = B1800; break;
            case 2400:speed = B2400; break;
            case 4800:speed = B4800; break;
            case 9600:speed = B9600; break;
            case 19200:speed = B19200; break;
            case 38400:speed = B38400; break;
            case 57600:speed = B57600; break;
            case 115200:speed = B115200; break;
            case 230400:speed = B230400; break;
    #ifdef B307200
            case 307200:speed = B307200; break;
    #endif
    #ifdef B460800
            case 460800:speed = B460800; break;
    #endif
            default:
            {
                ::close(pimpl->fd);
                setErrorMessage("Unsupported baud rate");
                return false;
            }
        }

        cfsetospeed(&new_attributes, speed);
        cfsetispeed(&new_attributes, speed);

        // Make changes effective
        status=tcsetattr(pimpl->fd, TCSANOW, &new_attributes);
        if(status<0)
        {
            ::close(pimpl->fd);
            setErrorMessage("Can't set port attributes");
            return false;
        }

        // Clear the O_NONBLOCK flag
        status = fcntl(pimpl->fd, F_GETFL, 0);
        if(status != -1) 
            fcntl(pimpl->fd, F_SETFL, status & ~O_NONBLOCK);
    }
    
#endif

    setErrorStatus(false);//If we get here, no error
    pimpl->open = true; //Port is now open

    thread t(&SerialPort::doRead, this);
    pimpl->backgroundThread.swap(t);
    return true;
}

bool SerialPort::isOpen() const
{
    return pimpl->open;
}

bool SerialPort::errorStatus() const
{
    lock_guard<mutex> l(pimpl->errorMutex);
    return pimpl->error;
}

void SerialPort::close()
{
    if(!isOpen()) 
        return;

    pimpl->open = false;

#if defined(_WIN32) || defined(__CYGWIN__)
	::CloseHandle(pimpl->fd);
#elif defined(__APPLE__) || defined(__linux__)
    ::close(pimpl->fd); //The thread waiting on I/O should return
#endif

    pimpl->backgroundThread.join();
}

void SerialPort::write(const char *data, size_t size)
{
    printf("SerialPort::write()\n");
#if defined(_WIN32) || defined(__CYGWIN__)
	
    HANDLE io_handler_ = (HANDLE)pimpl->fd;
	DWORD bytes_sent = 0;
	printf("SerialPort before WriteFile\n");
    ::WriteFile(io_handler_, (void*)data, size, &bytes_sent, NULL);
    printf("SerialPort after WriteFile\n");
    if(bytes_sent < size)
        setErrorStatus(true);

#elif defined(__APPLE__) || defined(__linux__)
    if(::write(pimpl->fd,data,size) != size) 
        setErrorStatus(true);
#endif
}

void SerialPort::write(const std::vector<char>& data)
{
    write(data.data(), data.size());
}

void SerialPort::writeString(const std::string& s)
{
    write(s.c_str(), s.size());
}

SerialPort::~SerialPort()
{
    if(isOpen())
        close();
}

void SerialPort::doRead()
{
    //Read loop in spawned thread
    for(;;)
    {
#if defined(_WIN32) || defined(__CYGWIN__)

	    DWORD received;
	    ::ReadFile(pimpl->fd, pimpl->readBuffer, readBufferSize, &received, NULL);

#elif defined(__APPLE__) || defined(__linux__)

        int received = ::read(pimpl->fd, pimpl->readBuffer, readBufferSize);

#endif

        if(received < 0)
        {
            if(!isOpen()) 
                return; // Thread interrupted because port closed
            else 
            {
                setErrorStatus(true);
                continue;
            }
        }

        if(pimpl->callback) 
            pimpl->callback(pimpl->readBuffer, received);
    }

}

void SerialPort::setErrorStatus(bool e)
{
    lock_guard<mutex> l(pimpl->errorMutex);
    pimpl->error = e;
}

void SerialPort::setErrorMessage(const std::string& error)
{
    lock_guard<mutex> l(pimpl->errorMutex);
    pimpl->error = true;
    pimpl->errorMessage = error;
}

void SerialPort::setReadCallback(const std::function<void (const unsigned char*, size_t)>& callback)
{
    pimpl->callback = callback;
}

void SerialPort::clearReadCallback()
{
    pimpl->callback = nullptr;
}
