/******************************************************************************************\
 *  (C) TDK Electronics GmbH & Co
 *  All rights reerved
 *  
 *  Title     : TDK_USSM_frame_status
 *  Release   : V1.0
 *  Version   : 2022-05-25 
 *  Desclaimer: This example is provided as a simple guidance WITOUT ANY WARRENTY.
 *    
 *  DESCRIPTION:
 *  ===========
 *  This example code demonstrates a simple use of TDK Ultrsonic Sensor
 *  Arduino Library to show detailed measurement frame status.
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


#define N_SENSORS 1
TDK_USSM TdkUssm(2); //Initialize Sensor Pins (IoPin)

//-- Work Variables
char    str[64];

//---------------------------------
// Functions prototypes

//-- Test Functions
void testFrameStatus(int device_mask=0x1, int tx_mask=0x1);

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
    testFrameStatus(0x1, 0x1); // single sensor
    Serial.println("-------------------"); 
    delay(200); 
  }
}



//---------------------------------
//-----<<< HELP FUNCTIONS >>>------
//---------------------------------


//---------------------------------
//----- <<<<< TEST >>>>>-----------
//---------------------------------
//-- Report Frame Status
void testFrameStatus(int device_mask, int tx_mask)
{
  int echo_sel=0;
  int rx_ptr=0;
  uint8_t val=0;
  int max_echos = 3;
  eSensorData *pdata = &TdkUssm.dataBuf;

  /*------------- 
   * 8 bit MCUs add some significant delay of up to 10us per device 
   * which needs to be compensated to avoid sensor misinterpreting the command
   * This delay also depends of the Total pull-up resistor on Sensor IO Line
   */
  #ifdef ARDUINO_ARCH_AVR  
    TdkUssm.TimeAdjustUs = 30;
  #endif

  rx_ptr = TdkUssm.WaveSendReceive(device_mask, tx_mask); // for all sensors

  //-- Process and Print result
  for(int d=0; d<N_SENSORS; d++) {
    if((1<<d) & device_mask) { // Enabled 
      TdkUssm.WaveDecodeData(d);
          
      //-- N Echos      
      sprintf(str,"N_Echos[%d] = { %d }", d,  pdata->n_echos); Serial.println(str);
      
      //-- Distance mm
      sprintf(str,"Distance_mm[%d] = { ", d); Serial.print(str);
      for(int e=0;e<max_echos; e++){
        sprintf(str,"%5d ",  pdata->distances[e]); Serial.print(str);
      }
      Serial.println("}");
      
      //-- Time of flights
      sprintf(str,"ToF_us[%d] = { ", d); Serial.print(str);
      max_echos = pdata->n_echos;
      if(max_echos > 3) max_echos=3;
      for(int e=0;e<max_echos; e++){
        sprintf(str,"%5d ",  pdata->tn_echos[e]); Serial.print(str);
      }
      Serial.println("}");
      
      //-- Echo width
      sprintf(str,"Width_us[%d] = { ", d); Serial.print(str);
      for(int e=0;e<max_echos; e++){
        sprintf(str,"%5d ",  pdata->len_echos[e]); Serial.print(str);
      }
      Serial.println("}");

      //-- Echo height
      sprintf(str,"Height[%d] = { ", d); Serial.print(str);
      for(int e=0;e< pdata->n_echo_heights; e++){
        sprintf(str,"%3d ",  pdata->hight_echos[e]); Serial.print(str);
      }
      Serial.println("}");

      //-- Status bits
      val = pdata->echo_status[0];
      sprintf(str,"Flags[%d] = { ", d); Serial.println(str);
      sprintf(str," [1:0]-F_DEV   = %d", (val>>0)&0x3); Serial.println(str); 
      sprintf(str,"   [2]-INVALID = %d", (val>>2)&0x1); Serial.println(str);
      sprintf(str,"   [3]-NOISE   = %d", (val>>3)&0x1); Serial.println(str);
      sprintf(str,"   [5]-NFD_Flag= %d", (val>>5)&0x1); Serial.println(str);
      Serial.println("}");
    }
  }
}
