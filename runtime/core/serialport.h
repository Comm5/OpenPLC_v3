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

#ifndef RUNTIME_CORE_SERIALPORT_H
#define	RUNTIME_CORE_SERIALPORT_H

#include <vector>
#include <memory>
#include <functional>


/**
 * Used internally (pimpl)
 */
class SerialPortImpl;

/**
 * Asyncronous serial class.
 * Intended to be a base class.
 */
class SerialPort
{
public:
    SerialPort();

    /**
     * Constructor. Creates and opens a serial device.
     * \param devname serial device name, example "/dev/ttyS0" or "COM1"
     * \param baud_rate serial baud rate
     * \param opt_parity serial parity, default none
     * \param opt_csize serial character size, default 8bit
     * \param opt_flow serial flow control, default none
     * \param opt_stop serial stop bits, default 1
     * \throws boost::system::system_error if cannot open the
     * serial device
     */
    SerialPort(const std::string& devname, unsigned int baud_rate);

    virtual ~SerialPort();

    /**
    * Opens a serial device.
    * \param devname serial device name, example "/dev/ttyS0" or "COM1"
    * \param baud_rate serial baud rate
    * \param opt_parity serial parity, default none
    * \param opt_csize serial character size, default 8bit
    * \param opt_flow serial flow control, default none
    * \param opt_stop serial stop bits, default 1
    * \throws boost::system::system_error if cannot open the
    * serial device
    */
    bool open(const std::string& devname, unsigned int baud_rate);

    /**
     * \return true if serial device is open
     */
    bool isOpen() const;

    /**
     * \return true if error were found
     */
    bool errorStatus() const;

    /**
     * \return string error details for the last error
     */
    std::string errorMessage() const;

    /**
     * Close the serial device
     * \throws boost::system::system_error if any error
     */
    void close();

    /**
     * Write data asynchronously. Returns immediately.
     * \param data array of char to be sent through the serial device
     * \param size array size
     */
    void write(const char *data, size_t size);

     /**
     * Write data asynchronously. Returns immediately.
     * \param data to be sent through the serial device
     */
    void write(const std::vector<char>& data);

    /**
    * Write a string asynchronously. Returns immediately.
    * Can be used to send ASCII data to the serial device.
    * To send binary data, use write()
    * \param s string to send
    */
    void writeString(const std::string& s);

    /**
     * Set a read callback called from SerialPort working
     * thread context.
     */
    void setReadCallback(const std::function<void (const unsigned char*, size_t)>& callback);

    /**
     * To unregister the read callback in the derived class destructor so it
     * does not get called after the derived class destructor but before the
     * base class destructor
     */
    void clearReadCallback();


    /**
     * Read buffer maximum size
     */
    static const int readBufferSize=512;
private:

    /**
     * Callback called to start an asynchronous read operation.
     * This callback is called by the io_service in the spawned thread.
     */
    void doRead();


    std::shared_ptr<SerialPortImpl> pimpl;

protected:

    /**
     * To allow derived classes to report errors
     * \param e error status
     */
    void setErrorStatus(bool e);

    /**
     * To allow derived classes to report errors
     * \param e error status
     */
    void setErrorMessage(const std::string& error);
};


#endif //ASYNCSERIAL_H