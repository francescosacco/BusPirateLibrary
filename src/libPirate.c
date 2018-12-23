#include "libPirate.h"

#include <stdio.h>

#include <string.h>
#include "serialport.h"

#define BUSPIRATE_CMD_RESET                      0x00
#define BUSPIRATE_CMD_HARDRESET                  0x0F
#define BUSPIRATE_CMD_CFGPERIPH                  0x40 // 0100.xxxx
#define BUSPIRATE_CMD_ADC                        0x14

#define BUSPIRATE_CMD_SPIMODE                    0x01
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

#define BUSPIRATE_RSP_RESET                      "BBIO1"
#define BUSPIRATE_RSP_SPIMODE                    "SPI1"
#define BUSPIRATE_RSP                            0x01

#define BUSPIRATE_ATTEMPT_RESET                  20

enum LIBPPIRATE_MODE
{
    libPirate_uninitialized ,
    libPirate_spi ,
    libPirate_i2c
} libPirate_mode = libPirate_uninitialized ;

uint8_t libPirate_configPeriph ;

static SerialRet_t libPirate_cmdAck( uint8_t cmd ) ;
static SerialRet_t libPirate_cmdRsp( uint8_t cmd , uint8_t * rsp ) ;

libPirate_t libPirate_init( const char * port )
{
    SerialRet_t serialRet ;
    uint8_t i ;

    serialRet = openSerialPort( port , 115200 , Stopbit_one , Parity_off , Handshake_none ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorSerial ) ;
    }
    
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
    
    return( libPirate_ok ) ;
}

libPirate_t libPirate_reset( void )
{
    SerialRet_t serialRet ;
    uint32_t dataSize ;
    uint8_t i , cmd , dummy ;

    dataSize = 1 ;
    cmd = BUSPIRATE_CMD_HARDRESET ;
    serialRet = writeToSerialPort( &cmd , &dataSize ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorSerial ) ;
    }
    
    do
    {
        dataSize = 1 ;
        serialRet = readFromSerialPort( &dummy , &dataSize ) ;
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
    uint8_t     readAdc[ 2 ] ;
    uint8_t     cmd ;

    if( libPirate_mode != libPirate_uninitialized )
    {
        return( libPirate_errorNotInit ) ;
    }

    if( adc == NULL )
    {
        return( libPirate_errorParam ) ;
    }
    
    dataSize = 1 ;
    cmd = BUSPIRATE_CMD_ADC ;
    serialRet = writeToSerialPort( &cmd , &dataSize ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorSerial ) ;
    }
    
    dataSize = 2 ;
    serialRet = readFromSerialPort( readAdc , &dataSize ) ;
    if( serialRet != SerialRet_ok )
    {
        return( libPirate_errorSerial ) ;
    }

    *adc = readAdc[ 0 ] ;
    ( *adc ) <<= 8 ;
    *adc |= readAdc[ 1 ] ;
    
    return( libPirate_ok ) ;
}

libPirate_t libPirate_spiConfig( libSpiSpeed_t spiSpeed , libPin_t spiIdle , libSpiClk_t spiClk )
{
    SerialRet_t serialRet ;
    uint8_t data ;
    
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
        blockTransfer = ( bufSize > 16 ) ? ( 16 ) : ( bufSize ) ;
        
        data  = ( uint8_t ) ( blockTransfer - 1 ) ;
        data |= BUSPIRATE_CMD_SPIBULKTRANSFER ;
        serialRet = libPirate_cmdAck( data ) ;
        if( serialRet != SerialRet_ok )
        {
            return( libPirate_errorInit ) ;
        }
        
        serialRet = writeToSerialPort( buffer + indexTransfer , &blockTransfer ) ;
        if( serialRet != SerialRet_ok )
        {
            return( serialRet ) ;
        }
        
        dataSize = blockTransfer ;
        serialRet = readFromSerialPort( buffer + indexTransfer , &dataSize ) ;
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
    uint32_t    dataSize ;
    uint8_t     rsp = 0x00 ;

    dataSize = 1 ;
    serialRet = writeToSerialPort( &cmd , &dataSize ) ;
    if( serialRet != SerialRet_ok )
    {
        return( serialRet ) ;
    }
    
    dataSize = 1 ;
    serialRet = readFromSerialPort( &rsp , &dataSize ) ;
    
    if( serialRet == SerialRet_ok )
    {
        if( ( dataSize < 1 ) || ( rsp != BUSPIRATE_RSP ) )
        {
            serialRet = SerialRet_error ;
        }
    }

    return( serialRet ) ;
}

static SerialRet_t libPirate_cmdRsp( uint8_t cmd , uint8_t * rsp )
{
    SerialRet_t serialRet ;
    uint8_t rspReset[ 16 ] ;
    uint32_t dataSize ;
    int strRet ;

    dataSize = 1 ;
    serialRet = writeToSerialPort( &cmd , &dataSize ) ;
    if( serialRet != SerialRet_ok )
    {
        return( serialRet ) ;
    }
    
    dataSize = sizeof( rspReset ) ;
    ( void ) memset( rspReset , '\0' , dataSize ) ;
    serialRet = readFromSerialPort( rspReset , &dataSize ) ;
    
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