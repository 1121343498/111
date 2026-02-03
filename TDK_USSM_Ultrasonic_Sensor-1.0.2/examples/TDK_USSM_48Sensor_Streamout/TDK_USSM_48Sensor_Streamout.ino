/******************************************************************************************\
 *  (C) TDK Electronics GmbH & Co
 *  All rights reerved
 *  
 *  Title     : TDK_USSM_frame_status
 *  Release   : V1.0
 *  Version   : 2022-07-05 
 *  Desclaimer: This example is provided as a simple guidance WITOUT ANY WARRENTY.
 *    
 *  DESCRIPTION:
 *  ===========
 *  This example code demonstrates a simple use of TDK Ultrsonic Sensor
 *  Arduino Library to use same library to run up to 48 Sensors on Arduino-MEGA2560.
 *        
 *  HARDWARE SETUP:
 *  ==============
 *    - TDK Ultrasonic Sensor has 3 pins:
 *      1- VS : to be connected to a power supply in the range of 8V to 18V. (Default 12V or 9V)
 *      2- IO : Is a bidirectional IO with signal range in 0 to VS.
 *      3- GND: Ground signal shall be same as Arduino GND
 *      
 *    - Sensor IO voltage level adapater:
 *      1- Level-Shifter using N-MOS transistor (Recommended):
 *          - Gate  to Arduino 5V.
 *          - Drain to IO Line with a pull-up resistor:
 *              - 15KOhm to VS if sensor or cable already has a builtin 6.8Kohm Pullup.
 *              - 6.8KOhm to 8.2KOhm if Sensor and cable have no pullups to VS.
 *          - Source to Arduino side IO pin (i.e. D2 or any other analog or digital pin)             
 *              - Board io pin shall be configured as INPUT_PULLUP when receiving.
 *              - Or use an external pull-up to VDDIO (5V or 3V3 depending of the board)
 *      2- Separate Tx/Rx using Low-side-switch:
 *          - Tx: Drain with pullup to IO Line, Source to Ground, Gate to Tx MCU output.
 *          - Rx: Voltage divider with resistors on IO-Line, Rx in the middle (>100K).
 *      3- LIN Transceiver: Tx to TXD, RX to RXD, LIN to Sensor IO.
\*******************************************************************************************/

#include <TDK_USSM.h>

//-- Sensors Pinmap Sensors
#define N_SENSORS         8*6   // 48 Sensors

//-- Arduino MEGA2560 Pinmap with Level-Shifter driver.
const int SensorPin[N_SENSORS] = {A0, A1,  A2,  A3,  A4,  A5 , A6,  A7, 
                                  A8, A9, A10, A11, A12, A13, A14, A15,
                                  22, 24,  26,  28,  30,  32,  34,  36, 
                                  38, 40,  42,  44,  46,  48,  50,  52,
                                  23, 25,  27,  29,  31,  33,  35,  37, 
                                  39, 41,  43,  45,  47,  49,  51,  53                           
                                  }; // Trigger & Echo Pins 

TDK_USSM TdkUssm(SensorPin[0]); // Single instance mode ==> Drive using IOPins instead of index

//---------------------------------
//-- Setup
void setup()
{
  Serial.begin(115200);
}

//---------------------------------
//-- Runtime.
void loop()
{ 
  while(1)
  {
    for(int i=0; i<N_SENSORS; i++) 
    {
      Serial.print( TdkUssm.GetDistanceCm(SensorPin[i], SensorPin[i]) );  // Prints Distance in cm of sensor i using pins not index.
      Serial.print(" ");
      //delay(20);
    }
    Serial.println();  // Terminate single capture line
    delay(2); 
  }
}
