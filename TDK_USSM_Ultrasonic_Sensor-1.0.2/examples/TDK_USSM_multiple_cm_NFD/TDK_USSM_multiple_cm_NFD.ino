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
 *      
 *  OUTPUT:
 *  =======
 *    - Binary Streamout (Packaed 9 bytes makred by header byte 0xff) 
 *      ff<sensor[0]> ... <sensor[7]
 *    - Ascii streamout 
 *      <sensor[0]> <sensor[1]> ... <sensor[7]> <nfd_bit_mask> <capture_time in ms> <processing in ms> 
\*******************************************************************************************/

#include <TDK_USSM.h>
#include "tdk_ussm_config.h"

//------------------------------------------


//------------------------------------------
// Comment out for Binary Streamout 
#define    REPORT_MODE_ASCII      'a'   
#define    REPORT_MODE_BINARY     'b' 
#define    REPORT_MODE_COMPACT    'c'
#define    REPORT_MODE_DEATAILED  'd'
#define    REPORT_MODE_EXTENDED   'e'

//-------------------------------------------
  #define   TDK_USSM_DEFAULT_MAX_ECHOS 1
  #define   BOARD_NAME            "TDK_USSM_Sensor_Board"
  #define   BOARD_VERSION         "20240624"
  #define   BOARD_FIRMWARE        COMMN_FIRMWARE_VERSION // Use common Firmware Version
  #define   N_SENSORS             9

  #define   BUZZER_PIN           PA9  // Same as UART1_TX

#ifdef TDK_USSM_Sensor_Board_3V3_IO
  const int TriggerPin[N_SENSORS] = {PA1 , PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1}; // Trigger Pin of Ultrasonic Sensor
  const int EchoPin[N_SENSORS]    = {PA1 , PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1}; // Echo Pin of Ultrasonic Sensor. Could same as Trigger pin if a bidir level shifter is used
  #define     IO_DRIVE_LOW        LOW    // Direct !
  #define     IO_RELEASE          HIGH   // Direct !
  #define     IO_IN_MODE          INPUT  // INPUT_PULLUP  
#else //==== Default 12V-IO ======
  const int TriggerPin[N_SENSORS] = {PC15, PC14, PB7, PB6, PB5, PB4, PB3, PA15, PA8}; // Trigger Pin of Ultrasonic Sensor  
  const int EchoPin[N_SENSORS]    = {PA1 , PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1}; // Echo Pin of Ultrasonic Sensor. Could same as Trigger pin if a bidir level shifter is used
  #define     IO_DRIVE_LOW        HIGH    // Inverted !
  #define     IO_RELEASE          LOW     // Inverted !
  #define     IO_IN_MODE          INPUT // INPUT_PULLUP
#endif
  const int AnaPin[N_SENSORS]     = {PA1 , PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1}; // Analog IO Pin used for envelop over IO
  const int ProgPin[N_SENSORS]    = {NC  , NC , NC , NC , NC , NC , NC , NC , NC }; // Dummy-Unused Eeprom PRogramming Pin
  const int SYS_LED               = PH3; 
  const int EN_12V                = NC;  // Enable 12V:  Unused  

  //-- Communication ports
  #define  UART1_TX       PA9   //-- Not recognized by Arduino by default, have to setup in the code
  #define  UART1_RX       PA10 
  #define  LIN_TX         UART1_TX   //-- Working with Arduino as normal UART ==> Add LIN Setup
  #define  LIN_RX         UART1_RX 
  #define  I2C_SCL        UART1_TX   //-- Default Arduino Setup for Wire
  #define  I2C_SDA        UART1_RX
  
//  HardwareSerial           Serial1(UART1_RX, UART1_TX); // Rx=PB7, Tx=PB6 ==> Might not be compatible with BootLoader using UART ?!
  // #define   PC_Serial      Serial1 
  #define   PC_Serial      Serial   // USB Serial

  //-- Hardware Timers
  #include "HardwareTimer.h"
  #define   HAL_GP_TIMER          TIM6
  #define   HAL_GP_TIMER_PERIOD_US   500000ul // 0.5 second
  HardwareTimer *hal_gp_timer = new HardwareTimer(HAL_GP_TIMER); // Use TIM6 timer
  
  //-- HAL Section
  #define           _HAL_DMA_BUF_SIZE  256   
  #define           _HAL_DMA_CHANNELS  3  
  #define           _HAL_ADC_INPUTS    9 

  int               _dma_timer_count = _HAL_DMA_CHANNELS; // Enabled only for PA0 .. PA3 ==> Sensor0 .. Sensor3      
  int               _hal_adc_n_channels = _HAL_ADC_INPUTS;
  
  int               _hal_n_channels = _HAL_DMA_CHANNELS;
  uint32_t          _hal_timer_freq_Mhz = 40;
  
  uint16_t          _dma_timer_mask  = 0x00ul;  // No DMA Timer Channel to have same refs for all sensors
  uint32_t          _hal_dma_buf[_HAL_DMA_CHANNELS][_HAL_DMA_BUF_SIZE]; //4 bytes x _HAL_DMA_BUF_SIZE = 2 KBytes
  uint32_t          _hal_adc_buf[_HAL_ADC_INPUTS]; 

  ADC_HandleTypeDef hadc1;
  TIM_HandleTypeDef htim2;
  DMA_HandleTypeDef hdma_adc1;
  DMA_HandleTypeDef hdma_tim2_ch2_ch4;

  void              DevBoard_hal_init(void);
  void              DevBoard_hal_ain_init(void);
  void              DevBoard_hal_adc_init(void);
  inline void       DevBoard_hal_adc_start(void);
  inline void       DevBoard_hal_adc_stop(void);  
  inline int        DevBoard_hal_adc_get_value(int index);      
  void              DevBoard_hal_timer_init(void); 
  inline uint32_t   DevBoard_hal_timer_start(int mask);
  inline void       DevBoard_hal_timer_stop(void); 

  #define    Board_AnalogInInit() DevBoard_hal_ain_init()
  #define    Board_AnalogStart()  DevBoard_hal_adc_start()
  #define    Board_AnalogRead(d)  DevBoard_hal_adc_get_value(d)  
  #define    Board_AnalogStop()   DevBoard_hal_adc_stop()
  #define    Board_TimerStart(d)  DevBoard_hal_timer_start(d)
  #define    Board_TimerStop()    DevBoard_hal_timer_stop()

  #define    SoftwareReset()      HAL_NVIC_SystemReset()    
  void       Board_DevBoardDiagnosis(int sel);
  //-- End of HAL Section
  
  #define     ANA_BUF_SIZE        512
  #define     ANA_BUF_SIZE_TOTAL  (ANA_BUF_SIZE*N_SENSORS)  
  #define     WAVE_BUFF_SIZE      1024
  #define     TX_WAVE_BUFF_SIZE   256
  #define     COM_BUF_SIZE        256

  #define     IO_TIME_ADJUST_US   0    // Adjust high/low timing (Low-Tadj), (High+Tadj)

  #define     PROG_ENABLE         HIGH
  #define     PROG_DISABLE        LOW

//------------------------------------------------------------
//-- Work Variables
TDK_USSM TdkUssm(TriggerPin, EchoPin, N_SENSORS, EchoPin, IO_DRIVE_LOW, IO_RELEASE, IO_IN_MODE);   //Constructor for multiple sensors full config

char     str[64];
volatile char     cmdstr[16]; 
volatile byte     meas_distance[N_SENSORS];
volatile byte     meas_status[N_SENSORS];
volatile uint16_t meas_nfd = 0x0; 
volatile int      meas_dt[2];
  
const    int      meas_d_offset_mm = 100; // 10 cm offset by default as we use detection by echo peack

//------------------------------------------------------------
//-- Functions prototypes
void GetDistancesAndNfd(int device_mask=0x1, int tx_mask=0x1);
void setNFDValues(int winVal=0, int thresVal=0, int toffVal=0, int device_mask=0xFF);

//------------------------------------------------------------
//-- Setup 
void setup()
{ 
  //-- Serial Communication Initialization
  Serial.begin(115200);
  
  //-- IMPORTANT => Ensure you set NFD-relevant (NFD_WIN, NFD_THRES, NFD_TOFF) REAL values before running. This is critical in order to ensure correct NFD_FLAG operation
  //setNFDValues(0, 0, 0, 0xff);
}

//-- Runtime.
void loop()
{ 
  while(1)
  {
    GetDistancesAndNfd(0xff, 0xff); 
    PrintReport(0xff, REPORT_MODE_ASCII);
    //delay(50); 
  }
}

//---------------------------------
//----<<< HELPER FUNCTIONS >>>-----
//---------------------------------
void setNFDValues(int winVal, int thresVal, int toffVal, int device_mask) {
  for(int d=0; d<N_SENSORS; d++) {
    if((1<<d) & device_mask) {
      TdkUssm.RegisterFieldSetRealValue(NFD_WIN, d, winVal);
      TdkUssm.RegisterFieldSetRealValue(NFD_THRES, d, thresVal);
      TdkUssm.RegisterFieldSetRealValue(NFD_TOFF, d, toffVal);
    }
  }
}


//---------------------------------
//--------<<<<< REPORT >>>>>-------
//---------------------------------
void PrintReport(int device_mask, int mode)
{
  int mask = 0;
  switch(mode) 
  {
    /**************
     * Report in ASCII Mode
     *  - If NFD flag detected ==> Report 0
     *  - Else report distance
     */
    case REPORT_MODE_ASCII      : // 'a'  
      for(int d=0; d<N_SENSORS; d++) {
        mask = (1<<d) & device_mask;
        if(mask) { // Enabled 
          if(mask & meas_nfd) { // NFD detected ==> Write 0
            Serial.print("  0 ");
          } else {
            sprintf(str,"%3d ", meas_distance[d] ); Serial.print(str);
          }
        }
      }
      Serial.println();
      break;

     /**************
     * Report in BINARY Mode
     *  - Header ffff Marker
     *  - If NFD flag detected ==> Report 0
     *  - Else report distance
     */  
    case REPORT_MODE_BINARY     : // 'b'
      Serial.write((uint8_t)0xff);
      Serial.write((uint8_t)0xff);
      for(int d=0; d<N_SENSORS; d++) {
        mask = (1<<d) & device_mask;
        if(mask) { // Enabled 
          if(mask & meas_nfd) { // NFD detected ==> Write 0
            Serial.write((uint8_t)0);
          } else { // NFD not detected write distance
            Serial.write(meas_distance[d]);
          }
        }
      }
      break;

     /**************
     * Report in BINARY Mode
     *  - Report ffff Marker
     *  - Report sensors distances
     *  - Report NFD 2 bytes to cover 9 sensors
     */  
    case REPORT_MODE_COMPACT : // 'c'
      Serial.write((uint8_t)0xff);
      Serial.write((uint8_t)0xff);
      for(int d=0; d<N_SENSORS; d++) {
        mask = (1<<d) & device_mask;
        if(mask) { // Enabled 
          Serial.write(meas_distance[d]);
        }
      }
      Serial.write(meas_nfd>>8); // NFD MSB
      Serial.write(meas_nfd&0xff); // NFD LSB
      break;

     /**************
     * Report in ASCII Mode
     *  - Sensors distances
     *  - Report NFD 2 bytes as decimal
     *  - Report Measurement time in ms
     *  - Report Processing loop in ms
     */ 
    case REPORT_MODE_DEATAILED   : // 'd'
      for(int d=0; d<N_SENSORS; d++) {
        mask = (1<<d) & device_mask;
        if(mask) { // Enabled 
          sprintf(str,"%3d ", meas_distance[d] ); Serial.print(str);
        }
      }
      sprintf(str,"%d  %d %d ", meas_nfd, meas_dt[0], meas_dt[1]); Serial.println(str); 
      break;

     /**************
     * Report in ASCII Mode
     *  - Pairs of Sensors distance (Decimal) and Status (Hexadecimal)
     *  - comma separator
     *  - Report Measurement time in ms
     *  - Report Processing loop in ms
     */ 
    case REPORT_MODE_EXTENDED   : // 'e'
      for(int d=0; d<N_SENSORS; d++) {
        mask = (1<<d) & device_mask;
        if(mask) { // Enabled 
          sprintf(str,"%3d %02x , ", meas_distance[d], meas_status[d]); Serial.print(str);
        }
      }
      Serial.println();
      break;
      
    default :
      break;
  }
}

//---------------------------------
//------<<<<< ALGORITHM >>>>>------
//---------------------------------
void GetDistancesAndNfd(int device_mask, int tx_mask)
{
  int rx_ptr=0;
  uint8_t val=0;
  int aux;
  eSensorData *pdata = &TdkUssm.dataBuf;
  uint32_t dt[3], t0; 
  
  /*------------- 
   * 8 bit MCUs add some significant delay of up to 10us per device 
   * which needs to be compensated to avoid sensor misinterpreting the command
   * This delay also depends of the Total pull-up resistor on Sensor IO Line
   */
  #ifdef ARDUINO_ARCH_AVR  
    TdkUssm.TimeAdjustUs = 30;
  #endif

  t0 = millis();
  rx_ptr = TdkUssm.WaveSendReceive(device_mask, tx_mask); // for all sensors
  dt[0] = millis() - t0;   t0 = millis();
  
  //-- Process and Print result
  meas_nfd = 0;
  for(int d=0; d<N_SENSORS; d++) {
    aux = 0;
    if((1<<d) & device_mask) { // Enabled 
      TdkUssm.WaveDecodeData(d);

      //-- Status and NFD
      // { NFD_FLAG  , 5  , 1 , 0 , "NFD_FLAG" , "Near Field flag set"  , C_LUT_NULL }, 
      meas_status[d] = TdkUssm.dataBuf.echo_status[0];
      val = ( meas_status[d]>>5) & 0x1;
      meas_nfd |= val<<d;

      //-- Distance mm
      if((1<<d) & tx_mask) { // Send/Receive
        aux = TdkUssm.dataBuf.distances[1];
      }
      else { // Receive only
        aux = TdkUssm.dataBuf.distances[0];
      }
      
      //-- Adjust distance offset
      if(aux <= meas_d_offset_mm){
        aux = 0;
      } else {
        aux = aux - meas_d_offset_mm;
      }
      //-- convert from mm to cm
      meas_distance[d] = aux / 10; 
    }
  }
  dt[1] = millis() - t0;   t0 = millis();

  meas_dt[0] = dt[0];
  meas_dt[1] = dt[1];
}
