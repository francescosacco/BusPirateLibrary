#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "libPirate.h"

int main( void )
{
    libPirate_t libRet ;
    uint8_t buffer[ 20 ] ;
    
    memset( buffer +  0 , 0x5A , 10 ) ;
    memset( buffer + 10 , 0x3C , 10 ) ;
    
    libRet = libPirate_init( "\\\\.\\COM12" ) ;
    printf( "libPirate_init() == %02Xh\n" , ( uint8_t ) libRet ) ;

    libRet = libPirate_spiConfig( libSpiSpeed_1MHz , libPin_low , libSpiClk_rise ) ;
    printf( "libPirate_spiConfig() == %02Xh\n" , ( uint8_t ) libRet ) ;

    libRet = libPirate_power( libPiratePower_on ) ;
    printf( "libPirate_power() == %02Xh\n" , ( uint8_t ) libRet ) ;

    printf( "buffer =" ) ;
    for( int i = 0 ; i < 20 ; i++ )
        printf( " %02X " , buffer[ i ] ) ;
    printf( "\n" ) ;

    libRet = libPirate_spiCS( libSpiCS_low ) ;
    printf( "libPirate_spiCSLow() == %02Xh\n" , ( uint8_t ) libRet ) ;
    
    libRet = libPirate_spiTransfer( buffer , 20 ) ;
    printf( "libPirate_spiTransfer() == %02Xh\n" , ( uint8_t ) libRet ) ;
    
    libRet = libPirate_spiCS( libSpiCS_high ) ;
    printf( "libPirate_spiCSLow() == %02Xh\n" , ( uint8_t ) libRet ) ;
    
    printf( "buffer =" ) ;
    for( int i = 0 ; i < 20 ; i++ )
        printf( " %02X " , buffer[ i ] ) ;
    printf( "\n" ) ;
    
    libRet = libPirate_reset() ;
    printf( "libPirate_reset() == %02Xh\n" , ( uint8_t ) libRet ) ;

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
    
    libRet = libPirate_deInit() ;
    printf( "libPirate_deInit() == %02Xh\n" , ( uint8_t ) libRet ) ;

    return( 0 ) ;
}