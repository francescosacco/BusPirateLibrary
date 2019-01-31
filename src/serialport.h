/**
 *
 * @file serialport.h
 * @author Francesco Sacco
 * @date 2018-12-19
 * @brief It's a library to provide access for serial port.
 *
 * This library can provide access for a serial port if you are
 * using Windows operational system.
 * This code was based on SPinGW, from waynix (https://github.com/waynix/SPinGW).
 *
 * @see http://github.com/francescosacco/BinaryTools
 *
 * Copyright (c) 2018, Francesco Sacco
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the BusPirateLibrary project.
 * 
 **/
 
/**********
 *
 * Version log. 
 *
 * 2018-12-19 - 0.0.0 - Initial version.
 * 2019-01-29 - 0.1.0 - Add byte instructions and data size.
 *
 **********/
#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <stdint.h>

typedef enum STOPBIT_T
{
    Stopbit_one        ,
    Stopbit_oneAndHalf ,
    Stopbit_two
} Stopbit_t ;

typedef enum PARITY_T
{
    Parity_even ,
    Parity_odd  ,
    Parity_off  ,
    Parity_mark
} Parity_t ;

typedef enum HANDSHAKE_T
{
    Handshake_DTR_DSR  ,
    Handshake_RTS_CTS  ,
    Handshake_XON_XOFF ,
    Handshake_none
} Handshake_t ;

typedef enum SERIARET
{
    SerialRet_ok               ,
    SerialRet_errorParam       ,
    SerialRet_errorToOpen      ,
    SerialRet_errorAlreadyOpen ,
    SerialRet_errorNotOpen     ,
    SerialRet_errorDCB         ,
    SerialRet_errorTimeout     ,
    SerialRet_error
} SerialRet_t ;

/**
 * \brief Opens a new connection to a serial port.
 *
 * \param  portname  Name of the serial port(COM1 - COM9 or \\\\.\\COM1-COM256)
 * \param  baudrate  Baudrate of this port (for example 9600)
 * \param  dataSize  Number of data bits.
 * \param  stopbits  Number of stop-bits.
 * \param  parity    Parity (even, odd, off or mark)
 * \return           SerialRet_ok if there was no problem.
 **/
SerialRet_t openSerialPort( const char * portname , uint32_t baudrate , uint8_t dataSize , Stopbit_t stopbits , Parity_t parity , Handshake_t handshake ) ;

/**
 * \brief Read single byte from the serial port.
 *
 * \param  buffer  Pointer to the buffer where the read data will be written.
 * \return         SerialRet_ok if there was no problem.
 **/
SerialRet_t readByteFromSerialPort( uint8_t * buffer ) ;

/**
 * \brief Read data buffer from the serial port.
 *
 * \param  buffer      Pointer to the buffer where the read data will be written.
 * \param  buffersize  Maximum size of the buffer, also return number of written bytes.
 * \return             SerialRet_ok if there was no problem.
 **/
SerialRet_t readBufferFromSerialPort( uint8_t * buffer , uint32_t * pbuffersize ) ;

/**
 * \brief Write single byte to the serial port.
 *
 * \param  data  Data to be send to the serial port.
 * \return       SerialRet_ok if there was no problem.
 **/
SerialRet_t writeByteToSerialPort( uint8_t data ) ;

/**
 * \brief Write buffer to the serial port.
 *
 * \param  buffer   Pointer to the buffer where the read data will be read.
 * \param  plength  Maximum size of the buffer, also return number of read bytes.
 * \return          SerialRet_ok if there was no problem.
 **/
SerialRet_t writeBufferToSerialPort( uint8_t * data, uint32_t * plength ) ;

/**
 * \brief Close serial port.
 *
 * \return SerialRet_ok if there was no problem.
 **/
SerialRet_t closeSerialPort( void ) ;

#endif // SERIALPORT_H
