#ifndef LIBPIRATE_H
#define LIBPIRATE_H

#include <stdint.h>

typedef enum LIBPIRATE_T
{
    libPirate_ok ,
    libPirate_errorNotInit ,
    libPirate_errorInit ,
    libPirate_errorParam ,
    libPirate_errorSerial ,
    libPirate_error
} libPirate_t ;

typedef enum LIBSPISPEED_T
{
    libSpiSpeed_30kHz  = 0x00 ,
    libSpiSpeed_125kHz = 0x01 ,
    libSpiSpeed_250kHz = 0x02 ,
    libSpiSpeed_1MHz   = 0x03 ,
    libSpiSpeed_2MHz   = 0x04 ,
    libSpiSpeed_2M6Hz  = 0x05 ,
    libSpiSpeed_4MHz   = 0x06 ,
    libSpiSpeed_8MHz   = 0x07
} libSpiSpeed_t ;

typedef enum LIBPIN_T
{
    libPin_low ,
    libPin_high
} libPin_t ;

typedef enum LIBSPICLK_T
{
    libSpiClk_fall ,
    libSpiClk_rise
} libSpiClk_t ;

typedef enum LIBSPICS_T
{
    libSpiCS_low ,
    libSpiCS_high
} libSpiCS_t ;

typedef enum LIBPIRATEPOWER_T
{
    libPiratePower_off ,
    libPiratePower_on
} libPiratePower_t ;

libPirate_t libPirate_init( const char * port ) ;
libPirate_t libPirate_reset( void ) ; 
libPirate_t libPirate_power( libPiratePower_t power ) ; 
libPirate_t libPirate_aux( libPin_t aux ) ; 
libPirate_t libPirate_adc( uint16_t * adc ) ;

libPirate_t libPirate_spiConfig( libSpiSpeed_t spiSpeed , libPin_t spiIdle , libSpiClk_t spiClk ) ;
libPirate_t libPirate_spiCS( libSpiCS_t spiCS ) ;
libPirate_t libPirate_spiTransfer( uint8_t * buffer , uint16_t bufSize ) ;

libPirate_t libPirate_deInit( void ) ;

#endif // LIBPIRATE_H
