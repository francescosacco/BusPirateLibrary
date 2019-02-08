#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "libPirate.h"

void libPirate_printState( char * str , libPirate_t state )
{
    int sizeStr , i ;

    sizeStr = printf( str ) ;
    sizeStr = 40 - sizeStr ;
    
    for( i = 0 ; i < sizeStr ; i++ )
    {
        printf( " " ) ;
    }
    
    switch( state )
    {
    case libPirate_ok           :
        printf( "libPirate_ok\n" ) ;
        break ;
    case libPirate_errorNotInit :
        printf( "libPirate_errorNotInit\n" ) ;
        break ;
    case libPirate_errorInit    :
        printf( "libPirate_errorInit\n" ) ;
        break ;
    case libPirate_errorParam   :
        printf( "libPirate_errorParam\n" ) ;
        break ;
    case libPirate_errorSerial  :
        printf( "libPirate_errorSerial\n" ) ;
        break ;
    case libPirate_error        :
        printf( "libPirate_error\n" ) ;
        break ;
    default :
        printf( "%04Xh\n" , ( uint16_t ) state ) ;
        break ;
    }
}

int main( void )
{
    libPirate_t libRet ;
    uint8_t buffer[ 20 ] ;
    
    memset( buffer +  0 , 0x5A , 10 ) ;
    memset( buffer + 10 , 0x3C , 10 ) ;
    
    libRet = libPirate_init( "\\\\.\\COM12" ) ;
    libPirate_printState( "libPirate_init()" , libRet ) ;

    libRet = libPirate_spiConfig( libSpiSpeed_1MHz , libPin_low , libSpiClk_rise ) ;
    libPirate_printState( "libPirate_spiConfig()" , libRet ) ;

    libRet = libPirate_power( libPiratePower_on ) ;
    libPirate_printState( "libPiratePower_on()" , libRet ) ;

    printf( "buffer =" ) ;
    for( int i = 0 ; i < 20 ; i++ )
    {
        printf( " %02X" , buffer[ i ] ) ;
    }
    printf( "\n" ) ;

    libRet = libPirate_spiCS( libSpiCS_low ) ;
    libPirate_printState( "libPirate_spiCS()" , libRet ) ;
    
    libRet = libPirate_spiTransfer( buffer , 20 ) ;
    libPirate_printState( "libPirate_spiTransfer()" , libRet ) ;
    
    libRet = libPirate_spiCS( libSpiCS_high ) ;
    libPirate_printState( "libPirate_spiCS()" , libRet ) ;
    
    printf( "buffer =" ) ;
    for( int i = 0 ; i < 20 ; i++ )
    {
        printf( " %02X" , buffer[ i ] ) ;
    }
    printf( "\n" ) ;
    
    libRet = libPirate_reset() ;
    libPirate_printState( "libPirate_reset()" , libRet ) ;

    printf( "libPirate_adc() == " ) ;
    for( int i = 0 ; i < 10 ; i++ )
    {
        uint16_t adc ;
        libRet = libPirate_adc( &adc ) ;
        if( libRet == libPirate_ok )
        {
            printf( " %04X" , adc ) ;
        }
        else
        {
            printf( "ERROR" ) ;
            break ;
        }
    }
    printf( "\n" ) ;
    
    libRet = libPirate_i2cConfig( libI2cSpeed_400kHz ) ;
    libPirate_printState( "libPirate_i2cConfig()" , libRet ) ;
    
    libRet = libPirate_reset() ;
    libPirate_printState( "libPirate_reset()" , libRet ) ;
    
    libRet = libPirate_uartConfig( libUartSpeed_9600bps , libPirateParity_off , libPirateStopbit_one ) ;
    libPirate_printState( "libPirate_uartConfig()" , libRet ) ;
    
    libRet = libPirate_uartSend( buffer , 20 ) ;
    libPirate_printState( "libPirate_uartSend()" , libRet ) ;

    libRet = libPirate_deInit() ;
    libPirate_printState( "libPirate_deInit()" , libRet ) ;

    return( 0 ) ;
}