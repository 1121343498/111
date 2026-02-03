/******************************************************************************************\
 *  (C) TDK Electronics GmbH & Co
 *  All rights reerved
 *  
 *  Title     : TDK_USSM_single_cm
 *  Release   : V1.0
 *  Version   : 2021-11-19 
 *  Desclaimer: This example is provided as a simple guidance WITOUT ANY WARRENTY.
 *    
 *  DESCRIPTION:
 *  ===========
 *  This example code demonstrates a simple use of TDK Ultrsonic Sensor
 *  Arduino Library to measure distance in centimeter (cm).
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

//-- Instanciate Single Sensor
TDK_USSM TdkUssm(2); //Initialize Sensor Pins (IoPin)

//-- Setup 
void setup()
{ 
  Serial.begin(115200); 
}

//-- Runtime.
void loop()
{ 
  while(1){
    Serial.println( TdkUssm.GetDistanceCm() );  // Prints Distance in cm.
    delay(200); 
  }
}
