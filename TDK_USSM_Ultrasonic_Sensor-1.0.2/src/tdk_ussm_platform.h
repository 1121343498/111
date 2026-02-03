/*************************************************
 *  (C) TDK Electronics GmbH & Co
 *  All rights reerved
 *  
 *  Title     : TDK_USSM_Ultrasonic_Sensor
 *  Release   : V1.0
 *  Version   : 2023-08-07 
 *  Descrption: 
 *    TDK Ultrasonic Sensor Platform Definitions
 *
 *  -------- ABSOLUTE NO WARRENTY OF ANY KIND -------
 *
 *   THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, 
 *   EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
 *   OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE. MICROSOFT CORPORATION
 *   SHALL NOT BE LIABLE FOR ANY TECHNICAL OR EDITORIAL ERRORS OR OMISSIONS CONTAINED HEREIN. 
 * 
 *************************************************/

#ifndef _TDK_USSM_PLATFORM_H_
 #define _TDK_USSM_PLATFORM_H_


 //-----------------------  Customize your platform here !
 #ifdef ARDUINO
    #include "Arduino.h"
 #endif
 
 //----------------------- Default Generic Arduino HAL Connectors  
 #ifdef TDK_USSM_Sensor_Board
    #include "variant_TDK_USSM_Sensor_Board.h"
    
    //-- Bootloader activation function
    #define HAL_START_BOOTLOADER()    TDK_USSM_Sensor_Board_hal_start_bootloader();
       
    //-- Analog Pin requires special treatment. ADC is handeled by DMA
    #define TDK_USSM_ADC_DMA_BUF      TDK_USSM_adc_dma_buf    
    #define TdkUssm_analogInInit()    TDK_USSM_Sensor_Board_hal_adc_start() 
    
 #endif

#endif //  _TDK_USSM_PLATFORM_H_
/************************ (C) COPYRIGHT TDK Electronics *****END OF FILE****/
