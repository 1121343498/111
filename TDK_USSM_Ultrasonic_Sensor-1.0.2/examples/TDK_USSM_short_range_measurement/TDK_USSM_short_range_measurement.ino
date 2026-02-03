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

#define DRIVER_CIRCUIT    IS_LEVEL_SHIFTER  // IS_LIN_DRIVER // or IS_LOW_SIDE_SWITCH_DRIVER


//-- Sensors Pinmap Sensors
#define N_SENSORS  2 //4 //12   // Can support up to 16 Sensors simulatnously
#define ALL_SENSORS_MASK ((1<<N_SENSORS)-1)

//-- Sensor Pin-map
const int TriggerPin[N_SENSORS] = {2, 3}; //, 4, 5 }; //, 6, 7, 8, 9, 10, 11, 12, 13}; // Trigger Pin of Ultrasonic Sensor
#if(DRIVER_CIRCUIT == IS_LEVEL_SHIFTER)
  #define EchoPin   TriggerPin  // Same pins
  TDK_USSM TdkUssm(TriggerPin, EchoPin, N_SENSORS); //Initialize Sensor Pins (TxPins , RxPins) // Basic Init for LIN and Level shifter
#else 
  // Different Rx Pins
  const int EchoPin[N_SENSORS]    = {A0, A1}; //, A3, A4 }; //, A5, A6, A7 }; // Echo Pin of Ultrasonic Sensor are same in case of Level-Shifter
  //TDK_USSM(const int txPin[],const int rxPin[],int n, const int anaPin[]=NULL, int txLow=TdkUssm_LOW, int txHigh=TdkUssm_HIGH, int rxPinMode=TdkUssm_INPUT_PULLUP, int rxLowUs=TBIT0_CMP);   //Constructor for multiple sensors full config
  TDK_USSM TdkUssm(TriggerPin,EchoPin ,N_SENSORS, NULL, HIGH, LOW, INPUT, TBIT0_CMP);  // Advanced Init to use for Low-Side Switch Driver to force Tx polarity inversion
#endif


//-- Application Send/Receive Pattern
typedef struct {
  int rx_mask;
  int tx_mask;
} TypeDef_SendReceiveMask_t;

#define DISTANCE_OFFSET_CM  10 // 10cm offset by default
#define TX_RX_PATTERN_SIZE  2
const TypeDef_SendReceiveMask_t  tx_rx_pattern[TX_RX_PATTERN_SIZE] = {
      {0x3, 0x1},   // Sensor0 : sending & sensors 0 and 1 : receiving 
      {0x3, 0x2}    // Sensor1 : sending & sensors 0 and 1 : receiving 
      };


//-- Select Reporting style here by Comment/Uncomment one of the following modes
#define REPORT_SEQUENCE_MIN_DISTANCE  
//#define REPORT_STEP_MIN_DISTANCE
//#define REPORT_SENSORS_DISTANCE


//-- Work Variables
uint8_t buf[32];
char    str[32];
int     tx_sensors_mask = 1; // we will play a rotation, multiple sensors can be selected also
int     min_distances[TX_RX_PATTERN_SIZE];  // Min distance per sequence step
int     min_distance_cm = 10000; // Entire sequence min_distance (if needed)
volatile int  cntr = 0; // Pattern Index or counter

//-- You need this only for MCU with very little SRAM, for others just define TDK_USSM_FULL
//#define            WAVE_BUF_SIZE 32  //Default is 256 but uses a lot of memory
//eWaveStruct_t      wave_buf[WAVE_BUF_SIZE];
//eSensorData        _data;
//---------------------------------
// Functions prototypes

//-- Test Functions
int SendReceive_cm(int device_mask, int tx_mask);

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
    //-- Do single measurement (multi-sensors) and report if needed
    min_distances[cntr] = SendReceive_cm(tx_rx_pattern[cntr].rx_mask, tx_rx_pattern[cntr].tx_mask); 
    if(min_distance_cm > min_distances[cntr]) min_distance_cm = min_distances[cntr];
    #ifdef REPORT_STEP_MIN_DISTANCE
      sprintf(str,"%5d\n", min_distances[cntr]);
      Serial.print(str);
    #endif
    
    //-- Increment the step counter and check if the sequence is done 
    cntr++; 
    if(cntr >= TX_RX_PATTERN_SIZE) {     // end of sequence
      #ifdef REPORT_SEQUENCE_MIN_DISTANCE
        sprintf(str,"%5d\n", min_distance_cm);
        Serial.print(str);
      #endif
      cntr = 0; // Reset  
      min_distance_cm = 10000; // reset
    }

    //-- Some delay of needed
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
int SendReceive_cm(int device_mask, int tx_mask)
{
  int echo_sel=0;
  int rx_ptr=0;
  int d_cm=0, min_d_cm = 10000;

  /*------------- 
   * 8 bit MCUs add some significant delay of up to 10us per device 
   * which needs to be compensated to avoid sensor misinterpreting the command
   * This delay also depends of the Total pull-up resistor on Sensor IO Line
   */
  #ifdef ARDUINO_ARCH_AVR  
    TdkUssm.TimeAdjustUs = 30;
  #endif

  //-- Run the entire sequence and record 
  rx_ptr = TdkUssm.WaveSendReceive(device_mask, tx_mask/*, CMD_SEND_RECEIVE_A, &wave_buf[0], WAVE_BUF_SIZE, 0, time_adjust_us*/);

  //-- Process and Print result
  min_d_cm = 10000;
  for(int d=0; d<N_SENSORS; d++) 
  {
    if((1<<d) & device_mask) 
    { //-- Enabled Device
      // STATUS_CFG set to 0 to decode only ToF 
      TdkUssm.WaveDecodeData(d /*, CMD_SEND_RECEIVE_A, STATUS_CFG_NONE, &wave_buf[0] , rx_ptr, &_data*/);
      //-- First Echo on Sending/Tx Sensor is the burst itself.
      echo_sel = (0 != ((1<<d) & tx_mask)) ? 1 : 0; 
      d_cm = TdkUssm.dataBuf.distances[echo_sel] - DISTANCE_OFFSET_CM;
      if(d_cm<0) d_cm =0;
      if(min_d_cm > d_cm) min_d_cm = d_cm;
      #ifdef REPORT_SENSORS_DISTANCE          
          sprintf(str,"%5d ", TdkUssm.dataBuf.distances[echo_sel]);
          Serial.print(str);
      #endif
    }
  }
  #ifdef REPORT_SENSORS_DISTANCE
    Serial.println();
  #endif

  //-- return min distance of current measurement
  return min_d_cm;
}
