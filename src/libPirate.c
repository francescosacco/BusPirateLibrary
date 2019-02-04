/**
 *
 * @file libPirate.c
 * @author Francesco Sacco
 * @date 2018-12-22
 * @brief It's a header with external functions to use busPirate.
 *
 * Here there's the functions for busPirate device. This project was
 * developed and tested using Bus Pirate v3.6.
 *
 * @see http://github.com/francescosacco/BusPirateLibrary/
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
#include "libPirate.h"

#include <string.h>
#include "serialport.h"

#define BUSPIRATE_CMD_RESET                      0x00 // Enter raw bitbang mode, reset to raw bitbang mode.
#define BUSPIRATE_CMD_HARDRESET                  0x0F
#define BUSPIRATE_CMD_CFGPERIPH                  0x40 // 0100.xxxx
#define BUSPIRATE_CMD_ADC                        0x14

#define BUSPIRATE_CMD_SPIMODE                    0x01 // Enter binary SPI mode, responds "SPI1".
#define BUSPIRATE_CMD_SPICSLOW                   0x02 // 0000.001x
#define BUSPIRATE_CMD_SPICSHIGH                  0x03 // 0000.001x
#define BUSPIRATE_CMD_SPISPEED                   0x60 // 0110.0xxx
#define BUSPIRATE_CMD_SPIBULKTRANSFER            0x10 // 0001.xxxx
#define BUSPIRATE_CMD_SPICFG                     0x80 // 1000.xxxx

#define BUSPIRATE_MASK_CFGPERIPH_POWER           0x08 // 0000.1000
#define BUSPIRATE_MASK_CFGPERIPH_AUX             0x02 // 0000.0010

#define BUSPIRATE_MASK_SPICFG_OUTZ               0x00 // IDLE = 0 , CLOCK Fall
#define BUSPIRATE_MASK_SPICFG_OUTPP              0x08 // IDLE = 0 , CLOCK Raise

#define BUSPIRATE_MASK_SPICFG_IDLELOWCLKFALL     0x00 // IDLE = 0 , CLOCK Fall
#define BUSPIRATE_MASK_SPICFG_IDLELOWCLKRAISE    0x02 // IDLE = 0 , CLOCK Raise
#define BUSPIRATE_MASK_SPICFG_IDLEHIGHCLKRAISE   0x06 // IDLE = 1 , CLOCK Raise
#define BUSPIRATE_MASK_SPICFG_IDLEHIGHCLKFALL    0x04 // IDLE = 1 , CLOCK Fall

#define BUSPIRATE_CMD_I2CMODE                    0x02 // Enter binary I2C mode, responds "I2C1".
#define BUSPIRATE_CMD_SETI2CSPEED                0x60 // Set I2C Speed - 011000xx

#define BUSPIRATE_CMD_UARTMODE                   0x03 // Enter binary I2C mode, responds "ART1".
#define BUSPIRATE_CMD_SETUARTSPEED               0x60 // Set UART Speed - 0110xxxx
#define BUSPIRATE_CMD_CONFIGUREUART              0x80 // Configure UART - 100wxxyz

#define BUSPIRATE_RSP_RESET                      "BBIO1"
#define BUSPIRATE_RSP_SPIMODE                    "SPI1"
#define BUSPIRATE_RSP_I2CMODE                    "I2C1"
#define BUSPIRATE_RSP_UARTMODE                   "ART1"
#define BUSPIRATE_RSP                            0x01

#define BUSPIRATE_ATTEMPT_RESET                  20

enum LIBPPIRATE_MODE
{
    libPirate_uninitialized ,
    libPirate_spi ,
    libPirate_i2c ,
    libPirate_uart
} libPirate_mode = libPirate_uninitialized ;

uint8_t libPirate_configPeriph ;

static SerialRet_t libPirate_cmdAck( uint8_t cmd ) ;
static SerialRet_t libPirate_cmdRsp( uint8_t cmd , uint8_t * rsp ) ;

libPirate_t libPirate_init( const char * port )
{
    SerialRet_t serialRet ;
    uint8_t i ;

    /**********
     *
     * Open serial port:
     *   - Speed     : 115200bps
     *   - Data Size : 8 bits
     *   - Parity    : None
     *   - Stop bit  : 1
     *
     **********/
    serialRet = openSerialPort( port , 115200 , 8 , Stopbit_one , Parity_off , Handshake_none ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorSerial ) ;
    }

    for( i = 0 ; i < BUSPIRATE_ATTEMPT_RESET ; i++ )
    {
        serialRet = libPirate_cmdRsp( BUSPIRATE_CMD_RESET , BUSPIRATE_RSP_RESET ) ;
        if( serialRet == SerialRet_ok )
        {
            libPirate_mode = libPirate_uninitialized ;
            break ;
        }
        else if( serialRet == SerialRet_errorTimeout )
        {
            continue ;
        }
        else
        {
            return( libPirate_errorInit ) ;
        }
    }
    
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorInit ) ;
    }
    
    return( libPirate_ok ) ;
}

libPirate_t libPirate_reset( void )
{
    SerialRet_t serialRet ;
    uint8_t i , dummy ;

    // Send Hard Reset command.
    serialRet = writeByteToSerialPort( BUSPIRATE_CMD_HARDRESET ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorSerial ) ;
    }

    // Empty serial buffer.
    do
    {
        serialRet = readByteFromSerialPort( &dummy ) ;
    } while( serialRet == SerialRet_ok ) ;
  
    for( i = 0 ; i < BUSPIRATE_ATTEMPT_RESET ; i++ )
    {
        serialRet = libPirate_cmdRsp( BUSPIRATE_CMD_RESET , BUSPIRATE_RSP_RESET ) ;
        if( serialRet == SerialRet_ok )
        {
            break ;
        }
        else if( serialRet == SerialRet_errorTimeout )
        {
            continue ;
        }
        else
        {
            return( libPirate_errorInit ) ;
        }
    }
    
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorInit ) ;
    }
    
    libPirate_mode = libPirate_uninitialized ;
    return( libPirate_ok ) ;
}

libPirate_t libPirate_power( libPiratePower_t power )
{
    SerialRet_t serialRet ;

    if( libPirate_mode != libPirate_spi )
    {
        return( libPirate_errorNotInit ) ;
    }

    /**********
     *
     * Configure peripherals:
     *
     *    7     6     5     4     3     2     1     0
     * +-----+-----+-----+-----+-----+-----+-----+-----+
     * |  0  |  1  |  0  |  0  |  W  |  X  |  Y  |  Z  |
     * +-----+-----+-----+-----+-----+-----+-----+-----+
     *  \__________ __________/ \_ _/ \_ _/ \_ _/ \_ _/
     *             |              |     |     |     |
     *             |              |     |     |     +--> CS       : 1 - Enable, 0 - Disable.
     *             |              |     |     +--------> Aux      : 1 - Enable, 0 - Disable.
     *             |              |     +--------------> Pull-ups : 1 - Enable, 0 - Disable.
     *             |              +--------------------> Power    : 1 - Enable, 0 - Disable.
     *             +-----------------------------------> Command  : 4xh - Configure peripherals.
     *
     **********/
    
    libPirate_configPeriph &= 0x0F ;
    libPirate_configPeriph |= BUSPIRATE_CMD_CFGPERIPH ;
    if( power == libPiratePower_on )
    {
        libPirate_configPeriph |= BUSPIRATE_MASK_CFGPERIPH_POWER ;
    }
    else
    {
        libPirate_configPeriph &= ~BUSPIRATE_MASK_CFGPERIPH_POWER ;
    }
    
    serialRet = libPirate_cmdAck( libPirate_configPeriph ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_error ) ;
    }

    return( libPirate_ok ) ;
}

libPirate_t libPirate_aux( libPin_t aux )
{
    SerialRet_t serialRet ;

    if( libPirate_mode != libPirate_spi )
    {
        return( libPirate_errorNotInit ) ;
    }

    /**********
     *
     * Configure peripherals:
     *
     *    7     6     5     4     3     2     1     0
     * +-----+-----+-----+-----+-----+-----+-----+-----+
     * |  0  |  1  |  0  |  0  |  W  |  X  |  Y  |  Z  |
     * +-----+-----+-----+-----+-----+-----+-----+-----+
     *  \__________ __________/ \_ _/ \_ _/ \_ _/ \_ _/
     *             |              |     |     |     |
     *             |              |     |     |     +--> CS       : 1 - Enable, 0 - Disable.
     *             |              |     |     +--------> Aux      : 1 - Enable, 0 - Disable.
     *             |              |     +--------------> Pull-ups : 1 - Enable, 0 - Disable.
     *             |              +--------------------> Power    : 1 - Enable, 0 - Disable.
     *             +-----------------------------------> Command  : 4xh - Configure peripherals.
     *
     **********/

    libPirate_configPeriph &= 0x0F ;
    libPirate_configPeriph |= BUSPIRATE_CMD_CFGPERIPH ;
    if( aux == libPin_low )
    {
        libPirate_configPeriph |= BUSPIRATE_MASK_CFGPERIPH_AUX ;
    }
    else
    {
        libPirate_configPeriph &= ~BUSPIRATE_MASK_CFGPERIPH_AUX ;
    }
    
    serialRet = libPirate_cmdAck( libPirate_configPeriph ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_error ) ;
    }

    return( libPirate_ok ) ;
}

libPirate_t libPirate_adc( uint16_t * adc )
{
    SerialRet_t serialRet ;
    uint32_t    dataSize ;
    uint16_t    dataRet ;
    uint8_t     readAdc[ 2 ] ;

    if( libPirate_mode != libPirate_uninitialized )
    {
        return( libPirate_errorNotInit ) ;
    }

    if( adc == NULL )
    {
        return( libPirate_errorParam ) ;
    }
    
    serialRet = writeByteToSerialPort( BUSPIRATE_CMD_ADC ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorSerial ) ;
    }
    
    /**********
     *
     * ADC data:
     *
     *    7     6     5     4     3     2     1     0
     * +-----+-----+-----+-----+-----+-----+-----+-----+
     * |          MSB          |          LSB          |
     * +-----+-----+-----+-----+-----+-----+-----+-----+
     *
     **********/
    
    dataSize = 2 ;
    serialRet = readBufferFromSerialPort( readAdc , &dataSize ) ;
    if( ( serialRet != SerialRet_ok ) || ( dataSize != 2 ) )
    {
        return( libPirate_errorSerial ) ;
    }

    dataRet   = ( uint16_t ) readAdc[ 0 ] ;
    dataRet <<= 8 ;
    dataRet  |= ( uint16_t ) readAdc[ 1 ] ;
    *adc = dataRet ;

    return( libPirate_ok ) ;
}

libPirate_t libPirate_spiConfig( libSpiSpeed_t spiSpeed , libPin_t spiIdle , libSpiClk_t spiClk )
{
    SerialRet_t serialRet ;
    uint8_t data ;
    
    if( libPirate_mode != libPirate_uninitialized )
    {
        return( libPirate_errorNotInit ) ;
    }

    serialRet = libPirate_cmdRsp( BUSPIRATE_CMD_SPIMODE , BUSPIRATE_RSP_SPIMODE ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorInit ) ;
    }
    
    data = BUSPIRATE_CMD_SPICFG ;
    data |= BUSPIRATE_MASK_SPICFG_OUTPP ;

    if( spiIdle == libPin_low )
    {
        data |= ( spiClk == libSpiClk_fall ) ? ( BUSPIRATE_MASK_SPICFG_IDLELOWCLKFALL ) : ( BUSPIRATE_MASK_SPICFG_IDLELOWCLKRAISE ) ;
    }
    else
    {
        data |= ( spiClk == libSpiClk_fall ) ? ( BUSPIRATE_MASK_SPICFG_IDLEHIGHCLKFALL ) : ( BUSPIRATE_MASK_SPICFG_IDLEHIGHCLKRAISE ) ;
    }

    serialRet = libPirate_cmdAck( data ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorInit ) ;
    }

    data  = ( uint8_t ) spiSpeed ;
    data |= BUSPIRATE_CMD_SPISPEED ;
    serialRet = libPirate_cmdAck( data ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorInit ) ;
    }

    serialRet = libPirate_cmdAck( BUSPIRATE_CMD_SPICSHIGH ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorInit ) ;
    }

    libPirate_mode = libPirate_spi ;
    
    return( libPirate_ok ) ;
}

libPirate_t libPirate_i2cConfig( libI2cSpeed_t i2cSpeed )
{
    SerialRet_t serialRet ;
    uint8_t data ;
    
    if( libPirate_mode != libPirate_uninitialized )
    {
        return( libPirate_errorNotInit ) ;
    }

    serialRet = libPirate_cmdRsp( BUSPIRATE_CMD_I2CMODE , BUSPIRATE_RSP_I2CMODE ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorInit ) ;
    }

    data = ( uint8_t ) i2cSpeed ;
    data &= 0x03 ;
    data |= BUSPIRATE_CMD_SETI2CSPEED ;

    serialRet = libPirate_cmdAck( data ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorInit ) ;
    }

    libPirate_mode = libPirate_i2c ;
    
    return( libPirate_ok ) ;
}

libPirate_t libPirate_uartConfig( libUartSpeed_t uartSpeed , libPirateParity_t uartParity , libPirateStopbit_t uartStopbit )
{
    SerialRet_t serialRet ;
    uint8_t data ;
    
    if( libPirate_mode != libPirate_uninitialized )
    {
        return( libPirate_errorNotInit ) ;
    }

    serialRet = libPirate_cmdRsp( BUSPIRATE_CMD_UARTMODE , BUSPIRATE_RSP_UARTMODE ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorInit ) ;
    }

    /**********
     *
     * UART Speed:
     *
     *    7     6     5     4     3     2     1     0
     * +-----+-----+-----+-----+-----+-----+-----+-----+
     * |  0     1     1     0  |  X     X  |  Y  |  Z  |
     * +-----+-----+-----+-----+-----+-----+-----+-----+
     *  \__________ __________/ \__________ __________/
     *             |                       |
     *             |                       +-----------> Baudrate : 0000 -     300bps
     *             |                                                0001 -   1.200bps
     *             |                                                0010 -   2.400bps
     *             |                                                0011 -   4.800bps
     *             |                                                0100 -   9.600bps
     *             |                                                0101 -  19.200bps
     *             |                                                0110 -  31.250bps (MIDI)
     *             |                                                0111 -  38.400bps
     *             |                                                1000 -  57.600bps
     *             |                                                1010 - 115.200bps
     *             |
     *             +-----------------------------------> 6xh - Command
     *
     **********/

    data = ( uint8_t ) uartSpeed ;
    data &= 0x0F ;
    data |= BUSPIRATE_CMD_SETUARTSPEED ;

    serialRet = libPirate_cmdAck( data ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorInit ) ;
    }

    /**********
     *
     * Configure UART:
     *
     *    7     6     5     4     3     2     1     0
     * +-----+-----+-----+-----+-----+-----+-----+-----+
     * |  1     0     0  |  W  |  X     X  |  Y  |  Z  |
     * +-----+-----+-----+-----+-----+-----+-----+-----+
     *  \_______ _______/ \_ _/ \____ ____/ \_ _/ \_ _/
     *          |           |        |        |     |
     *          |           |        |        |     +--> Idle polarity - 0 to idle as 1, 1 to idle as 0.
     *          |           |        |        +--------> Top bit - 0 to 2 and 1 to 2.
     *          |           |        +-----------------> Databits and parity - 0 to 8N, 1 to 8E, 2 to 8O and 3 to 9N.
     *          |           +--------------------------> Pin Output - 0 to HiZ, 1 to 3V3.
     *          +--------------------------------------> 0x80 - Command
     *
     **********/
    
    data  = BUSPIRATE_CMD_CONFIGUREUART ; // 100x.xxxx - Command.
    data |= ( uint8_t ) 0x10            ; // xxx1.xxxx - Push Pull.
    data |= ( uint8_t ) uartParity      ; // xxxx.00xx - Parity and databits.
    data |= ( uint8_t ) uartStopbit     ; // xxxx.xx0x - Stopbits

    serialRet = libPirate_cmdAck( data ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorInit ) ;
    }

    libPirate_mode = libPirate_uart ;
    
    return( libPirate_ok ) ;
}

libPirate_t libPirate_spiCS( libSpiCS_t spiCS )
{
    SerialRet_t serialRet ;
    uint8_t data ;

    if( libPirate_mode != libPirate_spi )
    {
        return( libPirate_errorNotInit ) ;
    }

    data = ( spiCS == libSpiCS_low ) ? ( BUSPIRATE_CMD_SPICSLOW ) : ( BUSPIRATE_CMD_SPICSHIGH ) ;
    serialRet = libPirate_cmdAck( data ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorInit ) ;
    }

    return( libPirate_ok ) ;
}

libPirate_t libPirate_spiTransfer( uint8_t * buffer , uint16_t bufSize )
{
    SerialRet_t serialRet ;
    uint32_t blockTransfer , dataSize , indexTransfer = 0 ;
    uint8_t localBuffer[ 16 ] ;
    uint8_t data ;
    
    if( libPirate_mode != libPirate_spi )
    {
        return( libPirate_errorNotInit ) ;
    }

    if( bufSize == 0 )
    {
        return( libPirate_errorParam ) ;
    }

    do
    {
        // It can transfer maximum 16 bytes each iteration.
        blockTransfer = ( bufSize > 16 ) ? ( 16 ) : ( bufSize ) ;

        /**********
         *
         * Bulk SPI transfer:
         *
         *    7     6     5     4     3     2     1     0
         * +-----+-----+-----+-----+-----+-----+-----+-----+--- -- -   - -- ---+
         * |  0  |  0  |  0  |  1  |                       |                   |
         * +-----+-----+-----+-----+-----+-----+-----+-----+--- -- -   - -- ---+
         *  \__________ __________/ \__________ __________/ \________ ________/
         *             |                       |                     |
         *             |                       |                     +---------> Payload      : From 1 to 16 bytes.
         *             |                       +-------------------------------> Payload Size : Number of bytes - 1.
         *             +-------------------------------------------------------> Command      : 1xh
         *
         **********/
        
        data  = ( uint8_t ) ( blockTransfer - 1 ) ;
        data &= 0x0F ;
        data |= BUSPIRATE_CMD_SPIBULKTRANSFER ;
        serialRet = libPirate_cmdAck( data ) ;
        if( serialRet != SerialRet_ok )
        {
            return( libPirate_errorInit ) ;
        }

        // Write data.
        serialRet = writeBufferToSerialPort( buffer + indexTransfer , &blockTransfer ) ;
        if( serialRet != SerialRet_ok )
        {
            return( serialRet ) ;
        }

        // Read data.
        dataSize = blockTransfer ;
        serialRet = readBufferFromSerialPort( buffer + indexTransfer , &dataSize ) ;
        if( serialRet != SerialRet_ok )
        {
            return( serialRet ) ;
        }
        
        bufSize -= blockTransfer ;
        indexTransfer += blockTransfer ;
    } while( bufSize > 0 ) ;

    return( libPirate_ok ) ;
}

libPirate_t libPirate_deInit( void )
{
    SerialRet_t serialRet ;

    // Close serial port.
    serialRet = closeSerialPort() ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorInit ) ;
    }

    libPirate_mode = libPirate_uninitialized ;

    return( libPirate_ok ) ;
}

static SerialRet_t libPirate_cmdAck( uint8_t cmd )
{
    SerialRet_t serialRet ;
    uint8_t     rsp = 0x00 ;

    // Send command.
    serialRet = writeByteToSerialPort( cmd ) ;
    if( serialRet != SerialRet_ok )
    {
        return( serialRet ) ;
    }
    
    // Received confirmation.
    serialRet = readByteFromSerialPort( &rsp ) ;
    if( ( serialRet == SerialRet_ok ) && ( rsp != BUSPIRATE_RSP ) )
    {
        serialRet = SerialRet_error ;
    }

    return( serialRet ) ;
}

static SerialRet_t libPirate_cmdRsp( uint8_t cmd , uint8_t * rsp )
{
    SerialRet_t serialRet ;
    uint8_t rspReset[ 16 ] ;
    uint32_t dataSize ;
    int strRet ;

    // Send command.
    serialRet = writeByteToSerialPort( cmd ) ;
    if( serialRet != SerialRet_ok )
    {
        return( serialRet ) ;
    }
    
    // Received confirmation.
    dataSize = sizeof( rspReset ) - 1 ;
    ( void ) memset( rspReset , '\0' , dataSize ) ;
    serialRet = readBufferFromSerialPort( rspReset , &dataSize ) ;
    
    if( serialRet == SerialRet_ok )
    {
        strRet = strcmp( rspReset , rsp ) ;
        if( strRet != 0 )
        {
            serialRet = SerialRet_error ;
        }
    }

    return( serialRet ) ;
}
