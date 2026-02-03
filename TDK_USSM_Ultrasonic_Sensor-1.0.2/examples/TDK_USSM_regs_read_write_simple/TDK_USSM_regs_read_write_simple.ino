/*************************************************
 *  (C) TDK Electronics GmbH & Co
 *  All rights reerved
 *  
 *  Title     : TDK_USSM_regs_read_write
 *  Release   : V1.0
 *  Version   : 2022-08-23 
 *  Desclaimer: This example is provided as a simple guidance WITOUT ANY WARRENTY.
 *    
 *  DESCRIPTION:
 *  ===========
 *  This example demonstrates a simple use of TDK Ultrsonic Sensor to measure distance in cm
 *        
 *  HARDWARE SETUP:
 *  ==============
 *    - TDK Ultrasonic Sensor has 3 pins:
 *      1- VS : to be connected to a power supply in the range of 8V to 18V. (Default 12V or 9V)
 *      2- IO : Is a bidirectional IO with signal range in 0 to VS.
 *      3- GND: Ground signal shall be same as Arduino GND
 *      
 *    - Level-Shifter using N-MOS transistor:
 *      1- Gate  to Arduino 5V.
 *      2- Drain to IO Line with a pull-up resistor:
 *         - 15KOhm to VS if sensor or cable already has a builtin 6.8Kohm Pullup.
 *         - 6.8KOhm to 8.2KOhm if Sensor and cable have no pullups to VS.
 *      3- Source to Arduino side IO pin (i.e. D2 or any other analog or digital pin)             
 *         - Board io pin shall be configured as INPUT_PULLUP when receiving.
 *         - Or use an external pull-up to VDDIO (5V or 3V3 depending of the board)
 *
 *************************************************/

#include <TDK_USSM.h>
#include "tdk_ussm_config.h"


//-- Sensors Pinmap Sensors
#define N_SENSORS  4   // Can support up to 16 Sensors simulatnously
#define ALL_SENSORS_MASK ((1<<N_SENSORS)-1)


//-- Easy way to select which Driver is used
#define IS_LEVEL_SHIFTER  1 
/* 
 *  In case of Level-Shifter driving circuit:
 *    - Trigger and Echo-Pins are same
 *    - LOW to drive LOW 
 *    - HIGH to drive HIGH
 *    - Sensing is same polarity
*/

#define IS_LIN_DRIVER   2 
/* 
 *  In case of LIN-Driver driving circuit:
 *    - Trigger pins are Tx
 *    - Echo pins are Rx (different from Tx)
 *    - LOW   to drive LOW 
 *    - HIGH  to drive HIGH
 *    - Sensing is same polarity
*/

#define IS_LOW_SIDE_SWITCH_DRIVER   3 
/* 
 *  In case of Low-Side-Switch driving circuit:
 *    - Trigger pins are Tx
 *    - Echo pins are Rx (different from Tx)
 *    - LOW   to drive HIGH ==> Polarity inverted !
 *    - HIGH  to drive LOW  ==> Polarity inverted !
 *    - Sensing is same polarity
*/

#define DRIVER_CIRCUIT   IS_LOW_SIDE_SWITCH_DRIVER // IS_LEVEL_SHIFTER  // IS_LIN_DRIVER  

//-- Sensors Pinmap Sensors
#define N_SENSORS  2 //4 //12   // Can support up to 16 Sensors simulatnously
#define ALL_SENSORS_MASK ((1<<N_SENSORS)-1)

//-- Sensor Pin-map
#if(DRIVER_CIRCUIT == IS_LEVEL_SHIFTER)
  const int TriggerPin[N_SENSORS] = {2, 3}; //, 4, 5 }; //, 6, 7, 8, 9, 10, 11, 12, 13}; // Trigger Pin of Ultrasonic Sensor
  #define EchoPin   TriggerPin  // Same pins
  TDK_USSM TdkUssm(TriggerPin, EchoPin, N_SENSORS); //Initialize Sensor Pins (TxPins , RxPins) // Basic Init for LIN and Level shifter
#else 
  // Different Rx Pins
  const int TriggerPin[N_SENSORS] = {A0, A2}; //, A3, A4 }; //, A5, A6, A7 }; // Echo Pin of Ultrasonic Sensor are same in case of Level-Shifter
  const int EchoPin[N_SENSORS]    = {A1, A3}; //, A3, A4 }; //, A5, A6, A7 }; // Echo Pin of Ultrasonic Sensor are same in case of Level-Shifter
  //TDK_USSM(const int txPin[],const int rxPin[],int n, const int anaPin[]=NULL, int txLow=TdkUssm_LOW, int txHigh=TdkUssm_HIGH, int rxPinMode=TdkUssm_INPUT_PULLUP, int rxLowUs=TBIT0_CMP);   //Constructor for multiple sensors full config
  TDK_USSM TdkUssm(TriggerPin,EchoPin ,N_SENSORS, NULL, HIGH, LOW, INPUT, TBIT0_CMP);  // Advanced Init to use for Low-Side Switch Driver to force Tx polarity inversion
#endif

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
  // === EXAMPLE 1 === Changing G_ANA (analog gain)
  // G_ANA is a parameter which controls the onboard analog amplifier. This has the effect of emitting a higher-magnitude signal.
  // G_ANA possible values range from 39.2 dB to 56.0 dB. These are mapped through a lookup table across all binary values that can be represented with 3 bits, such that the steps taken between the possible values are equal. 
  // If the user wants to set a value which is not in the lookup table, the closest value in terms of magnitude present in the lookup table will be used instead.

  
  // = EXAMPLE 1A =
  // Setting a value present in the lookup table => 41.6 dB
  TdkUssm.RegisterFieldSetRealValue(G_ANA, 0, 41.6);

  // The expected output should is 41.6
  Serial.println(TdkUssm.RegisterFieldGetRealValue(G_ANA, 0));

  // = EXAMPLE 1B =
  // Setting a value NOT present in the lookup table => 49.0 dB
  TdkUssm.RegisterFieldSetRealValue(G_ANA, 0, 49.0);
  // The set value is the closest value present in the lookup table (in this case 48.8 dB). Therefore, the expected output should be 48.8
  Serial.println(TdkUssm.RegisterFieldGetRealValue(G_ANA, 0));




  // === EXAMPLE 2 === Changing G_DIG (digital gain)
  // G_DIG is a parameter which artificially amplifies the amplitude of the received signal (including noise).
  // G_DIG possible values range from 0 dB to 47.89 dB. Unlike the previous example, the number of values that can be taken by G_DIG is much higher (spread across 8 bits). Although no lookup table is present here, the values are computed through polynomial approximation, through knowing the start, end, and step size.
  // Therefore, it is unlikely that the user wants to set a value which is not in the lookup table, the closest value in terms of magnitude present in the lookup table will be used instead.

  
  // = EXAMPLE 2A =
  // Setting a value present in the lookup table => 0.0 dB
  TdkUssm.RegisterFieldSetRealValue(G_DIG, 0, 0.0);

  // The expected output should is 0.0
  Serial.println(TdkUssm.RegisterFieldGetRealValue(G_DIG, 0));

  // = EXAMPLE 2B =
  // Setting a value NOT present in the lookup table => 45 dB
  TdkUssm.RegisterFieldSetRealValue(G_DIG, 0, 45.0);
  // The set value will be 45dB, or a value very close to this.
  Serial.println(TdkUssm.RegisterFieldGetRealValue(G_DIG, 0));
  
  Serial.println();
  delay(2500);
}
