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

//-- Sensors Pinmap Sensors
#define N_SENSORS  2 //4 //12   // Can support up to 16 Sensors simulatnously
#define ALL_SENSORS_MASK ((1<<N_SENSORS)-1)
const int TriggerPin[N_SENSORS] = {2, 3}; //, 4, 5 }; //, 6, 7, 8, 9, 10, 11, 12, 13}; // Trigger Pin of Ultrasonic Sensor
const int EchoPin[N_SENSORS]    = {2, 3}; //, 4, 5 }; //, 6, 7, 8, 9, 10, 11, 12, 13}; // Echo Pin of Ultrasonic Sensor. 

//-- Note: Echo Pins could be same as Trigger pin if a bidir level shifter is used.

//TDK_USSM TdkUssm(TriggerPin,EchoPin ,N_SENSORS, NULL, LOW, HIGH, INPUT, TBIT0_CMP);  // Advanced Init to adapt to Driver
TDK_USSM TdkUssm(TriggerPin, EchoPin, N_SENSORS); //Initialize Sensor Pins (TxPins , RxPins) // Basic Init for LIN and Level shifter

//-- Work Variables
uint8_t buf[32];
char    str[32];
int     tx_sensors_mask = 1; // we will play a rotation, multiple sensors can be selected also

//-- You need this only for MCU with very little SRAM, for others just define TDK_USSM_FULL
//#define            WAVE_BUF_SIZE 32  //Default is 256 but uses a lot of memory
//eWaveStruct_t      wave_buf[WAVE_BUF_SIZE];
//eSensorData        _data;
//---------------------------------
// Functions prototypes

//-- Test Functions
void testSendReceiveStreamout(int device_mask, int tx_mask, int mode);

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
    testSendReceiveStreamout(0x3, 0x1, 0); 
    //Serial.println();  // Terminate single capture line
    delay(2); 
  }
}



//---------------------------------
//-----<<< HELP FUNCTIONS >>>------
//---------------------------------


//---------------------------------
//----- <<<<< TEST >>>>>-----------
//---------------------------------
//-- Parallel Sensors operation in Send/Receive mode
void testSendReceiveStreamout(int device_mask, int tx_mask, int mode)
{
  int echo_sel=0;
  int rx_ptr=0;

  /*------------- 
   * 8 bit MCUs add some significant delay of up to 10us per device 
   * which needs to be compensated to avoid sensor misinterpreting the command
   * This delay also depends of the Total pull-up resistor on Sensor IO Line
   */
  #ifdef ARDUINO_ARCH_AVR  
    TdkUssm.TimeAdjustUs = 30;
  #endif

  rx_ptr = TdkUssm.WaveSendReceive(device_mask, tx_mask/*, CMD_SEND_RECEIVE_A, &wave_buf[0], WAVE_BUF_SIZE, 0, time_adjust_us*/);

  //-- Process and Print result
  // for(int d=N_SENSORS-1; d>=0; d--) 
  for(int d=0; d<N_SENSORS; d++) 
  {
    if((1<<d) & device_mask) 
    { //-- Enabled Device
      // STATUS_CFG set to 0 to decode only ToF 
      TdkUssm.WaveDecodeData(d /*, CMD_SEND_RECEIVE_A, STATUS_CFG_NONE, &wave_buf[0] , rx_ptr, &_data*/);

      switch(mode)
      {
        default:
          //-- First Echo on Sending/Tx Sensor is the burst itself.
          echo_sel = (0 != ((1<<d) & tx_mask)) ? 1 : 0; 
          sprintf(str,"%5d ", TdkUssm.dataBuf.distances[echo_sel]);
          Serial.print(str);
          break;
       case 1 : 
          //-- First Echo Distance and Echo Height.
          echo_sel = (0 != ((1<<d) & tx_mask)) ? 1 : 0; 
          sprintf(str,"%5d %3d ", TdkUssm.dataBuf.distances[echo_sel], TdkUssm.dataBuf.hight_echos[0]);
          Serial.print(str);
          break;       
      }
    }
  }
   Serial.println();
}
