/**
 * @file serialport.h
 * @author Francesco Sacco
 * @date 19 Dec 2018
 * @brief It's a library to provide access for serial port.
 *
 * This library can provide access for a serial port if you are
 * using Windows operational system.
 * This code was based on SPinGW, from waynix (https://github.com/waynix/SPinGW).
 *
 * @see http://github.com/francescosacco/BinaryTools
 */

/**********
 *
 * Version log. 
 *
 * 2018-12-19 - 0.0.0 - Initial version.
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
    SerialRet_ok ,
    SerialRet_errorParam ,
    SerialRet_errorToOpen ,
    SerialRet_errorAlreadyOpen ,
    SerialRet_errorNotOpen ,
    SerialRet_errorDCB ,
    SerialRet_errorTimeout ,
    SerialRet_error
} SerialRet_t ;


/**
    \brief Opens a new connection to a serial port
    \param portname        name of the serial port(COM1 - COM9 or \\\\.\\COM1-COM256)
    \param baudrate        the baudrate of this port (for example 9600)
    \param stopbits        th nuber of stoppbits (one, onePointFive or two)
    \param parity        the parity (even, odd, off or mark)
    \return            HANDLE to the serial port
    */
SerialRet_t openSerialPort( const char * portname , uint32_t baudrate , Stopbit_t stopbits , Parity_t parity , Handshake_t handshake ) ;

/**
    \brief Read data from the serial port
    \param hSerial        File HANDLE to the serial port
    \param buffer        pointer to the area where the read data will be written
    \param buffersize    maximal size of the buffer area
    \return                amount of data that was read
    */
SerialRet_t readFromSerialPort( uint8_t * buffer , uint32_t * pbuffersize ) ;

/**
    \brief write data to the serial port
    \param hSerial    File HANDLE to the serial port
    \param buffer    pointer to the area where the read data will be read
    \param length    amount of data to be read
    \return            amount of data that was written
    */
SerialRet_t writeToSerialPort( uint8_t * data, uint32_t * plength ) ;

SerialRet_t closeSerialPort( void ) ;

#endif // SERIALPORT_H
