/*************************************************
 *  (C) TDK Electronics GmbH & Co
 *  All rights reerved
 *  
 *  Title     : TDK_USSM_Ultrasonic_Sensor
 *  Release   : V1.0
 *  Version   : 2023-08-07 
 *  Descrption: 
 *    TDK Ultrasonic Sensor Class HAL connections
 *
 *  -------- ABSOLUTE NO WARRENTY OF ANY KIND -------
 *
 *   THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, 
 *   EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
 *   OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE. MICROSOFT CORPORATION
 *   SHALL NOT BE LIABLE FOR ANY TECHNICAL OR EDITORIAL ERRORS OR OMISSIONS CONTAINED HEREIN. 
 * 
 *************************************************/

#ifndef _TDK_USSM_HAL_H_
 #define _TDK_USSM_HAL_H_


 //-----------------------  Customized TDK USSM HAL Connectors
 #include "tdk_ussm_platform.h"  
    
 //----------------------- Default Generic Arduino HAL Connectors  

    //-- IO Levels and Modes
    #ifndef TdkUssm_HIGH
        #define TdkUssm_HIGH           HIGH
    #endif
    #ifndef TdkUssm_LOW
        #define TdkUssm_LOW            LOW
    #endif

    #ifndef TdkUssm_INPUT
        #define TdkUssm_INPUT          INPUT
    #endif 
    #ifndef TdkUssm_OUTPUT
        #define TdkUssm_OUTPUT         OUTPUT
    #endif
    #ifndef TdkUssm_INPUT_PULLUP
        #define TdkUssm_INPUT_PULLUP   INPUT_PULLUP
    #endif

    //-- Digital Pin IO
    #ifndef TdkUssm_pinMode
        #define TdkUssm_pinMode(_pin,_mode)         pinMode(_pin,_mode)
    #endif
    #ifndef TdkUssm_digitalRead
        #define TdkUssm_digitalRead(_pin)           digitalRead(_pin)
    #endif
    #ifndef TdkUssm_digitalWrite
        #define TdkUssm_digitalWrite(_pin,_val)     digitalWrite(_pin,_val)
    #endif
    
    //-- Analog Pin In
    #ifndef TdkUssm_analogInInit
        #define TdkUssm_analogInInit()            {}  // By default nothing todo analogRead will do the job
    #endif
    #ifndef TdkUssm_analogRead 
        #define TdkUssm_analogRead(_pin)          analogRead(_pin)
    #endif
    #ifndef TdkUssm_analogBufRead // When DMA is used with ADC.
        #define TdkUssm_analogBufRead(_index)     TdkUssm_analogRead(_index)               
    #endif
    
    //-- Timing shall be accurate to better than 10us
    // For OS-based platforms, you can use a high-priority timer with 10 or 5us period 
    #ifndef TdkUssm_delayMicroseconds
        #define TdkUssm_delayMicroseconds(_us)    delayMicroseconds(_us)
    #endif
    #ifndef TdkUssm_micros
        #define TdkUssm_micros()                  micros()    // If not microsecond accurate transform to microseconds
    #endif
    
    //-- Bootloader activation
    #ifndef HAL_START_BOOTLOADER
        #define HAL_START_BOOTLOADER()            {}
    #endif


#endif //  _TDK_USSM_HAL_H_
/************************ (C) COPYRIGHT TDK Electronics *****END OF FILE****/
