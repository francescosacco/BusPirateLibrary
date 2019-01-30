/**
 *
 * @file libPirate.h
 * @author Francesco Sacco
 * @date 2018-12-22
 * @brief It's a header with external functions to use busPirate.
 *
 * Here there's the external functions for busPirate device. This project was
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
#ifndef LIBPIRATE_H
#define LIBPIRATE_H

#include <stdint.h>

typedef enum LIBPIRATE_T
{
    libPirate_ok           ,
    libPirate_errorNotInit ,
    libPirate_errorInit    ,
    libPirate_errorParam   ,
    libPirate_errorSerial  ,
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
