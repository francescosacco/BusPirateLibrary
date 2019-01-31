/**
 *
 * @file serialport.c
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
 * 2019-01-29 - 0.1.0 - Add byte instructions.
 *
 **********/

#include "serialport.h"

#include <windows.h>
#include <unistd.h>
#include <stdio.h>

#ifdef _UNICODE
  #define _T(x)                                  L ## x
#else
  #define _T(x)                                  x
#endif

#define ASCII_XON                                0x11
#define ASCII_XOFF                               0x13

static HANDLE hSerial = INVALID_HANDLE_VALUE ;

/**
 * \brief Opens a new connection to a serial port.
 * \param portname        name of the serial port(COM1 - COM9 or \\\\.\\COM1-COM256)
 * \param baudrate        the baudrate of this port (for example 9600)
 * \param stopbits        th number of stopbits (one, onePointFive or two)
 * \param parity        the parity (even, odd, off or mark)
 * \return            HANDLE to the serial port
 **/
SerialRet_t openSerialPort( const char * portname , uint32_t baudrate , uint8_t dataSize , Stopbit_t stopbits , Parity_t parity , Handshake_t handshake )
{
    BOOL boolRet ;

    DCB dcbSerialParams   = { 0 } ;
    COMMTIMEOUTS timeouts = { 0 } ;
    
    if( hSerial != INVALID_HANDLE_VALUE )
    {
        return( SerialRet_errorAlreadyOpen ) ;
    }
    
    /**********
     *
     * CreateFile parameters
     *
     * - LPCTSTR lpFileName
     *
     * - DWORD dwDesiredAccess: Type of access.
     *     - GENERIC_EXECUTE
     *     - GENERIC_READ
     *     - GENERIC_WRITE
     *
     * - DWORD dwShareMode:
     *     - 0 (Not share)
     *     - FILE_SHARE_READ  (Indicates that the open operations succeed only if read access is requested)
     *     - FILE_SHARE_WRITE (Indicates that the open operations succeed only if write access is requested)
     *
     * - LPSECURITY_ATTRIBUTES lpSecurityAttributes: Not supported.
     *
     * - DWORD dwCreationDisposition: Creation type.
     *     - CREATE_ALWAYS     (If the file exists, overwrites and clears the existing attributes)
     *     - OPEN_EXISTING     (The function fails if the file does not exist)
     *     - CREATE_NEW        (The function fails if the specified file already exists)
     *     - OPEN_ALWAYS       (If the file does not exist, the function creates the file as CREATE_NEW)
     *     - TRUNCATE_EXISTING (Once opened, the file is truncated so that its size is zero bytes. Depends on GENERIC_WRITE)
     *
     * - DWORD dwFlagsAndAttributes:
     *     - FILE_ATTRIBUTE_ARCHIVE    (Indicates that the file is archived)
     *     - FILE_ATTRIBUTE_COMPRESSED (Indicates that the file or directory is compressed)
     *     - FILE_ATTRIBUTE_HIDDEN     (Indicates that the file is hidden)
     *     - FILE_ATTRIBUTE_NORMAL     (Indicates that the file has no other attributes set. This attribute is valid only if used alone)
     *     - FILE_ATTRIBUTE_READONLY   (Indicates that the file is read-only)
     *     - FILE_ATTRIBUTE_ROMMODULE  (Indicates it is a DLL module)
     *     - FILE_ATTRIBUTE_SYSTEM     (Indicates that the file is part of or is used exclusively by the OS)
     *     - FILE_ATTRIBUTE_TEMPORARY  (Not supported)
     *     - FILE_FLAG_NO_BUFFERING    (Indicates that the file is opened with no system caching)
     *     - FILE_FLAG_OVERLAPPED      (This flag is not supported)
     *     - FILE_FLAG_RANDOM_ACCESS   (Indicates that the file is accessed randomly)
     *     - FILE_FLAG_WRITE_THROUGH   (Instructs the system to write through any intermediate cache and go directly to disk)
     *
     * - HANDLE hTemplateFile: Ignored.
     *
     **********/

    hSerial = CreateFile( _T( portname ) , ( GENERIC_READ | GENERIC_WRITE ) , 0 , 0 , OPEN_EXISTING , 0 , 0 ) ;
    if( hSerial == INVALID_HANDLE_VALUE )
    {
        return( SerialRet_errorToOpen ) ;
    }

    /**********
     *
     * DCB Structure
     *
     * - DWORD DCBlength: Size of DBC structure.
     *
     * - DWORD BaudRate: The baud rate at which the communications device operates.
     * - BYTE  ByteSize: The number of bits in the bytes transmitted and received.
     * - BYTE  StopBits: The number of stop bits to be used.
     *     - ONESTOPBIT   (1 stop bit)
     *     - ONE5STOPBITS (1.5 stop bits)
     *     - TWOSTOPBITS  (2 stop bits)
     * - DWORD fBinary: Windows does not support non-binary mode transfers, so this member must be TRUE.
     * - DWORD fNull: If this member is TRUE, null bytes are discarded when received.
     *
     * - DWORD fParity: If this member is TRUE, parity checking is performed and errors are reported.
     * - BYTE  Parity: The parity scheme to be used.
     *     - EVENPARITY  (Even parity)
     *     - MARKPARITY  (Mark parity)
     *     - NOPARITY    (No parity)
     *     - ODDPARITY   (Odd parity)
     *     - SPACEPARITY (Space parity)
     * - DWORD fErrorChar: Indicates whether bytes received with parity errors are replaced with the character specified by the ErrorChar member.
     * - char  ErrorChar: The value of the character used to replace bytes received with a parity error.
     * 
     * - DWORD fOutxCtsFlow: If this member is TRUE, the CTS (Clear To Send) signal is monitored for output flow control.
     * - DWORD fOutxDsrFlow: If this member is TRUE, the DSR (Data Set Ready) signal is monitored for output flow control.
     * - DWORD fDtrControl: The DTR (Data Terminal Ready) flow control.
     *     - DTR_CONTROL_DISABLE   (00h - Disables the DTR line when the device is opened and leaves it disabled)
     *     - DTR_CONTROL_ENABLE    (01h - Enables the DTR line when the device is opened and leaves it on)
     *     - DTR_CONTROL_HANDSHAKE (02h - Enables DTR handshaking)
     * - DWORD fDsrSensitivity: If this member is TRUE, the driver ignores any bytes received, unless the DSR modem input line is high.
     * - DWORD fRtsControl: The RTS (Request To Send) flow control.
     *     - RTS_CONTROL_DISABLE   (00h - Disables the RTS line when the device is opened and leaves it disabled)
     *     - RTS_CONTROL_ENABLE    (01h - Enables the RTS line when the device is opened and leaves it on)
     *     - RTS_CONTROL_HANDSHAKE (02h - Enables RTS handshaking)
     *     - RTS_CONTROL_TOGGLE    (03h - Specifies that the RTS line will be high if bytes are available for transmission)
     *
     * - DWORD fTXContinueOnXoff : Indicates whether transmission stops when the input buffer is full and the driver has transmitted the XoffChar character.
     * - WORD  XoffLim  : The minimum number of free bytes allowed in the input buffer before flow control is activated to inhibit the sender.
     * - WORD  XonLim   : The minimum number of bytes in use allowed in the input buffer before flow control is activated to allow transmission by the sender.
     * - char  XonChar  : The value of the XON character for both transmission and reception.
     * - char  XoffChar : The value of the XOFF character for both transmission and reception.
     * - DWORD fOutX    : Indicates whether XON/XOFF flow control is used during transmission.
     * - DWORD fInX     : Indicates whether XON/XOFF flow control is used during reception.
     *
     * - DWORD fAbortOnError : If it is TRUE, the driver terminates all read and write operations with an error status if an error occurs.
     * - char  EofChar       : The value of the character used to signal the end of data.
     * - char  EvtChar       : The value of the character used to signal an event.
     *
     * - DWORD fDummy2    : Reserved; do not use.
     * - WORD  wReserved  : Reserved; must be zero.
     * - WORD  wReserved1 : Reserved; do not use.
     *
     **********/

    dcbSerialParams.DCBlength = sizeof( dcbSerialParams ) ;
    boolRet = GetCommState( hSerial, &dcbSerialParams ) ;
    if( boolRet == FALSE )
    {
        return( SerialRet_errorDCB ) ;
    }

    dcbSerialParams.BaudRate = ( DWORD ) baudrate ;
    dcbSerialParams.ByteSize = ( uint8_t ) dataSize ;
    
    switch( stopbits )
    {
    case Stopbit_one :
        dcbSerialParams.StopBits = ONESTOPBIT ;
        break ;
    case Stopbit_oneAndHalf :
        dcbSerialParams.StopBits = ONE5STOPBITS ;
        break ;
    case Stopbit_two :
        dcbSerialParams.StopBits = TWOSTOPBITS ;
        break ;
    default :
        return( SerialRet_errorParam ) ;
    }

    switch( parity )
    {
    case Parity_even :
        dcbSerialParams.Parity = EVENPARITY ;
        break ;
    case Parity_odd :
        dcbSerialParams.Parity = ODDPARITY ;
        break ;
    case Parity_off :
        dcbSerialParams.Parity = NOPARITY ;
        break ;
    case Parity_mark :
        dcbSerialParams.Parity = MARKPARITY ;
        break ;
    default :
        return( SerialRet_errorParam ) ;
    }

    switch( handshake )
    {
    case Handshake_DTR_DSR :
        // Configure XON/XOFF.
        dcbSerialParams.fOutX = FALSE ;
        dcbSerialParams.fInX  = FALSE ;
        // Configure RTS/CTS.
        dcbSerialParams.fOutxCtsFlow = FALSE ;
        dcbSerialParams.fRtsControl  = RTS_CONTROL_DISABLE ;
        // Configure DSR/DTR.
        dcbSerialParams.fOutxDsrFlow = TRUE;
        dcbSerialParams.fDtrControl  = DTR_CONTROL_HANDSHAKE ;
        break ;
    case Handshake_RTS_CTS :
        // Configure XON/XOFF.
        dcbSerialParams.fOutX = FALSE;
        dcbSerialParams.fInX  = FALSE;
        // Configure RTS/CTS.
        dcbSerialParams.fOutxCtsFlow = TRUE ;
        dcbSerialParams.fRtsControl  = RTS_CONTROL_HANDSHAKE ;
        // Configure DSR/DTR.
        dcbSerialParams.fOutxDsrFlow = FALSE ;
        dcbSerialParams.fDtrControl  = DTR_CONTROL_DISABLE ;
        break ;
    case Handshake_XON_XOFF :
        // Configure XON/XOFF.
        dcbSerialParams.fOutX = TRUE ;
        dcbSerialParams.fInX  = TRUE ;
        dcbSerialParams.fTXContinueOnXoff = TRUE ;
        dcbSerialParams.XoffChar = ASCII_XOFF ;
        dcbSerialParams.XonChar  = ASCII_XON  ;
        // Configure RTS/CTS.
        dcbSerialParams.fOutxCtsFlow = FALSE ;
        dcbSerialParams.fRtsControl  = RTS_CONTROL_DISABLE ;
        // Configure DSR/DTR.
        dcbSerialParams.fOutxDsrFlow = FALSE ;
        dcbSerialParams.fDtrControl  = DTR_CONTROL_DISABLE ;
        break ;
    case Handshake_none :
        // Configure XON/XOFF.
        dcbSerialParams.fOutX = FALSE ;
        dcbSerialParams.fInX  = FALSE ;
        // Configure RTS/CTS.
        dcbSerialParams.fOutxCtsFlow = FALSE ;
        dcbSerialParams.fRtsControl  = RTS_CONTROL_DISABLE ;
        // Configure DSR/DTR.
        dcbSerialParams.fOutxDsrFlow = FALSE ;
        dcbSerialParams.fDtrControl  = DTR_CONTROL_DISABLE ;
        break ;
    default :
        return( SerialRet_errorParam ) ;
    }

    dcbSerialParams.fBinary = TRUE ;

    boolRet = SetCommState( hSerial , &dcbSerialParams ) ;
    if( boolRet == FALSE )
    {
        return( SerialRet_errorDCB ) ;
    }

    /**********
     *
     * COMMTIMEOUTS Structure
     *
     * DWORD ReadIntervalTimeout         : The maximum time allowed to elapse before the arrival of the next byte on the communications line, in milliseconds.
     * DWORD ReadTotalTimeoutMultiplier  : The multiplier used to calculate the total time-out period for read operations, in milliseconds.
     * DWORD ReadTotalTimeoutConstant    : A constant used to calculate the total time-out period for read operations, in milliseconds.
     * DWORD WriteTotalTimeoutMultiplier : The multiplier used to calculate the total time-out period for write operations, in milliseconds.
     * DWORD WriteTotalTimeoutConstant   : A constant used to calculate the total time-out period for write operations, in milliseconds.
     *
     **********/

    timeouts.ReadIntervalTimeout         = 50 ;
    timeouts.ReadTotalTimeoutConstant    = 50 ;
    timeouts.ReadTotalTimeoutMultiplier  = 10 ;
    timeouts.WriteTotalTimeoutConstant   = 50 ;
    timeouts.WriteTotalTimeoutMultiplier = 10 ;

    boolRet = SetCommTimeouts( hSerial , &timeouts ) ;
    if( boolRet == FALSE )
    {
        return( SerialRet_errorDCB ) ;
    }

    return( SerialRet_ok ) ;
}

SerialRet_t readByteFromSerialPort( uint8_t * buffer )
{
    DWORD bytesRead = 0 ;
    DWORD dwError ;
    BOOL  boolRet ;

    if( hSerial == INVALID_HANDLE_VALUE )
    {
        return( SerialRet_errorNotOpen ) ;
    }
    
    if( buffer == NULL )
    {
        return( SerialRet_errorParam ) ;
    }
    /**********
     *
     * ReadFile Parameters
     *
     * HANDLE       hFile                : A handle to the device.
     * LPVOID       lpBuffer             : A pointer to the buffer that receives the data read from a file or device.
     * DWORD        nNumberOfBytesToRead : The maximum number of bytes to be read.
     * LPDWORD      lpNumberOfBytesRead  : A pointer to the variable that receives the number of bytes read when using a synchronous hFile parameter.
     * LPOVERLAPPED lpOverlapped         : A pointer to an OVERLAPPED structure if the hFile was opened with FILE_FLAG_OVERLAPPED, otherwise it can be NULL.
     * 
     **********/
    boolRet = ReadFile( hSerial , ( LPVOID ) buffer , ( DWORD ) 1 , &bytesRead , NULL ) ;
    if( boolRet == FALSE )
    {
        /**********
         *
         * ClearCommError Parameters
         *
         * HANDLE    hFile    : A handle to the communications device.
         * LPDWORD   lpErrors : A pointer to a variable that receives a mask indicating the type of error.
         *     - CE_RXOVER   (0001h - An input buffer overflow has occurred)
         *     - CE_OVERRUN  (0002h - A character-buffer overrun has occurred)
         *     - CE_RXPARITY (0004h - The hardware detected a parity error)
         *     - CE_FRAME    (0008h - The hardware detected a framing error)
         *     - CE_BREAK    (0010h - The hardware detected a break condition)
         * LPCOMSTAT lpStat : A pointer to a COMSTAT structure in which the device's status information is returned, or null to ignore.
         *
         **********/
		ClearCommError( hSerial , &dwError , NULL ) ;

		if( dwError )
        {
            return( SerialRet_error ) ;
        }
        else
        {
            return( SerialRet_errorTimeout ) ;
        }
    }
    else if( bytesRead == 0 )
    {
        return( SerialRet_errorTimeout ) ;
    }
    
    return( SerialRet_ok ) ;
}

/**
 * \brief Read data from the serial port
 * \param hSerial        File HANDLE to the serial port
 * \param buffer        pointer to the area where the read data will be written
 * \param buffersize    maximal size of the buffer area
 * \return                amount of data that was read
 **/
SerialRet_t readBufferFromSerialPort( uint8_t * buffer , uint32_t * pbuffersize )
{
    DWORD bytesRead = 0 ;
    DWORD dwError ;
    BOOL  boolRet ;

    if( hSerial == INVALID_HANDLE_VALUE )
    {
        return( SerialRet_errorNotOpen ) ;
    }
    
    if( ( buffer == NULL ) || ( pbuffersize == NULL ) )
    {
        return( SerialRet_errorParam ) ;
    }
    
    if( *pbuffersize == 0 )
    {
        return( SerialRet_ok ) ;
    }
    
    /**********
     *
     * ReadFile Parameters
     *
     * HANDLE       hFile                : A handle to the device.
     * LPVOID       lpBuffer             : A pointer to the buffer that receives the data read from a file or device.
     * DWORD        nNumberOfBytesToRead : The maximum number of bytes to be read.
     * LPDWORD      lpNumberOfBytesRead  : A pointer to the variable that receives the number of bytes read when using a synchronous hFile parameter.
     * LPOVERLAPPED lpOverlapped         : A pointer to an OVERLAPPED structure if the hFile was opened with FILE_FLAG_OVERLAPPED, otherwise it can be NULL.
     * 
     **********/
    boolRet = ReadFile( hSerial , ( LPVOID ) buffer , ( DWORD ) *pbuffersize , &bytesRead , NULL ) ;
    if( boolRet == FALSE )
    {
        /**********
         *
         * ClearCommError Parameters
         *
         * HANDLE    hFile    : A handle to the communications device.
         * LPDWORD   lpErrors : A pointer to a variable that receives a mask indicating the type of error.
         *     - CE_RXOVER   (0001h - An input buffer overflow has occurred)
         *     - CE_OVERRUN  (0002h - A character-buffer overrun has occurred)
         *     - CE_RXPARITY (0004h - The hardware detected a parity error)
         *     - CE_FRAME    (0008h - The hardware detected a framing error)
         *     - CE_BREAK    (0010h - The hardware detected a break condition)
         * LPCOMSTAT lpStat : A pointer to a COMSTAT structure in which the device's status information is returned, or null to ignore.
         *
         **********/
		ClearCommError( hSerial , &dwError , NULL ) ;

		if( dwError )
        {
            return( SerialRet_error ) ;
        }
        else
        {
            return( SerialRet_errorTimeout ) ;
        }
    }
    else if( bytesRead == 0 )
    {
        return( SerialRet_errorTimeout ) ;
    }
    
    // Copy number of bytes.
    *pbuffersize = ( uint32_t ) bytesRead ;

    return( SerialRet_ok ) ;
}

SerialRet_t writeByteToSerialPort( uint8_t data )
{
    DWORD dwBytesRead = 0 ;
    BOOL boolRet ;

    if( hSerial == INVALID_HANDLE_VALUE )
    {
        return( SerialRet_errorNotOpen ) ;
    }
    
    /**********
     *
     * WriteFile Parameters
     *
     * HANDLE       hFile                  : A handle to the device.
     * LPCVOID      lpBuffer               : A pointer to the buffer containing the data to be written to the file or device.
     * DWORD        nNumberOfBytesToWrite  : The number of bytes to be written to the file or device.
     * LPDWORD      lpNumberOfBytesWritten : A pointer to the variable that receives the number of bytes written when using a synchronous hFile parameter.
     * LPOVERLAPPED lpOverlapped           : A pointer to an OVERLAPPED structure if the hFile was opened with FILE_FLAG_OVERLAPPED, otherwise it can be NULL.
     * 
     **********/
    boolRet = WriteFile( hSerial , ( LPCVOID ) &data , ( DWORD ) 1 , &dwBytesRead , NULL ) ;
    if( ( boolRet == FALSE ) || ( dwBytesRead != 1 ) )
    {
        return( SerialRet_error ) ;
    }

    return( SerialRet_ok ) ;
}

/**
 * \brief write data to the serial port
 * \param hSerial    File HANDLE to the serial port
 * \param buffer    pointer to the area where the read data will be read
 * \param length    amount of data to be read
 * \return            amount of data that was written
 **/
SerialRet_t writeBufferToSerialPort( uint8_t * data, uint32_t * plength )
{
    DWORD dwBytesRead = 0 ;
    BOOL boolRet ;

    if( hSerial == INVALID_HANDLE_VALUE )
    {
        return( SerialRet_errorNotOpen ) ;
    }
    
    if( ( data == NULL ) || ( plength == NULL ) )
    {
        return( SerialRet_errorParam ) ;
    }
    
    if( *plength == 0 )
    {
        return( SerialRet_ok ) ;
    }
    
    /**********
     *
     * WriteFile Parameters
     *
     * HANDLE       hFile                  : A handle to the device.
     * LPCVOID      lpBuffer               : A pointer to the buffer containing the data to be written to the file or device.
     * DWORD        nNumberOfBytesToWrite  : The number of bytes to be written to the file or device.
     * LPDWORD      lpNumberOfBytesWritten : A pointer to the variable that receives the number of bytes written when using a synchronous hFile parameter.
     * LPOVERLAPPED lpOverlapped           : A pointer to an OVERLAPPED structure if the hFile was opened with FILE_FLAG_OVERLAPPED, otherwise it can be NULL.
     * 
     **********/
    boolRet = WriteFile( hSerial , ( LPCVOID ) data , ( DWORD ) *plength , &dwBytesRead , NULL ) ;
    if( boolRet == FALSE )
    {
        return( SerialRet_error ) ;
    }

    // Copy number of bytes.
    *plength = ( uint32_t ) dwBytesRead ;

    return( SerialRet_ok ) ;
}

SerialRet_t closeSerialPort( void )
{
    if( hSerial == INVALID_HANDLE_VALUE )
    {
        return( SerialRet_errorNotOpen ) ;
    }

    CloseHandle( hSerial ) ;
    hSerial = INVALID_HANDLE_VALUE ;
    return( SerialRet_ok ) ;
}
