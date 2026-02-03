/*************************************************
 *  (C) TDK Electronics GmbH & Co
 *  All rights reerved
 *  
 *  Title     : TDK_USSM_single_cm
 *  Release   : V0.9.3
 *  Version   : 2022-05-25 
 *  Descrption: 
 *    TDK Ultrasonic Sensor Arduino Demo Software  
 *    
 *  DESCRIPTION:
 *  ===========
 *  This example code demonstrates a simple use of TDK Ultrsonic Sensor
 *  Arduino Library to Streamout MEasurement from 16 Sensors used concurrently in Send/Receive mode.
 *  This Example is also optimized to wor with Arduin MEGA2560. but not limited to it
 *        
 *  HARDWARE SETUP:
 *  ==============
 *    - TDK Ultrasonic Sensor has 3 pins:
 *      1- VS : to be connected to a power supply in the range of 7V to 18V. (Default 12V or 9V)
 *      2- IO : Is a bidirectional IO with signal range in 0 to VS
 *      3- GND: Ground signal shall be same as Arduino GND
 *      
 *    - Using Different Hardware:
 *      1- Driver which could be:
 *        - N-MOS : to operate as a Low-Side switch:
 *          + Gate  to Arduino Trigger Pin
 *          + Drain to IO Line.
 *          + Source to GND
 *          => Configuration: TdkUssm(TriggerPin, EchoPin, N_SENSORS, HIGH, LOW, INPUT_PULLUP)
 *        - LIN Transceiver :
 *          + Vsup same as Sensor VS
 *          + GND same as Sensor & Arduino GND
 *          + TX to Arduino Trigger Pin
 *          => Configuration: TdkUssm(TriggerPin, EchoPin, N_SENSORS, LOW, HOGH, INPUT_PULLUP) // Default
 *      2- Receiver which could be: 
 *        - Voltage adapater using Simple resistor-divider:
 *          + Upper-end to IO Line
 *          + Lower-end to GND
 *          + Mid-point to Arduino Echo Pin insuring that max voltage is within Arduino voltage range (5V or 3V3 depending of hardware version)  
 *          => Configuration: rxPinMode=INPUT 
 *        - Zenner diode + Resistor
 *          + Upper-end (Zenner Kathode) to IO Line
 *          + Lower-end (Resistor) to GND
 *          + Mid-point to Arduino Echo Pin insuring that max voltage is within Arduino voltage range (5V or 3V3 depending of hardware version)
 *          => Configuration: rxPinMode=INPUT 
 *        - LIN Transceiver :
 *          + RX to Arduino Echo Pin
 *          => Configuration: rxPinMode=INPUT_PULLUP // Default
 *          
 *          
 *  GENERAL CONSIDERATION
 *  =====================
 *  To be able to fully Evaluate the advanatge of Parallel Sensor-Receive
 *  It is highly recommended to have Sensor Pin assignment aligned with MCU Port pins
 *  This will allow:
 *   - Perfect Sensors sycnrhonization
 *   - Better measurement accuracy
 *   - Large set of sensors (up to 16) to run in parallel
 *  Note: With High Speed MCU (Cortex-M) you can also use random MCU pins with reduced impact on accuracy.
 *
 *  THIS EXAMPLE:
 *  =============
 *   - Pin-map based on Arduin MEGA2560 using pins: A0 .. A15 which are aligned on PF, PK ports
 *   - We developped basic HAL functions to allow fast port Read/write halATMEGA2560_xxx for reference.
 *   - A0 .. A15 are used in bidirectional mode which means that we used simple N-MOS Level-shifter.
 *   - This Pin-map and assignment might also allow Analog Envelop capture over Sensor IO line.
 *              
 *   Note:    
 *    - IO Line requires a Pull-up to VS. Some of sensors have it inside, but some others variants do not have it.
 *    - If your sensor does not have an internal pull-up, you need to add an external 6.8KOhm pull-up resistor between IO and VS.
 *
 *************************************************/

#include <TDK_USSM.h>

//-- Sensors Pinmap 
//-------------------------------
/* 

*/
//-- Sensor Pin Map
#define N_SENSORS                16  // Can support up to 16 Sensors simulatnously
#define ALL_SENSORS_MASK        ((1<<N_SENSORS)-1)

const int SensorPins[N_SENSORS] = {A0, A1, A2, A3, A4, A5 , A6, A7, A8, A9, A10, A11, A12, A13, A14, A15}; // IO Pins of Ultrasonic Sensors
#define TriggerPin  SensorPins
#define EchoPin     SensorPins
#define AnalogPin   SensorPins

//-- Note: Echo Pins could be same as Trigger pin if a bidir level shifter is used.
TDK_USSM TdkUssm(SensorPins,SensorPins ,N_SENSORS, NULL, LOW, HIGH, INPUT_PULLUP, TBIT0_CMP);  // Advanced Init to adapt to Driver

//-- Work Variables
#define BUF_SIZE  256*1
uint8_t buf[BUF_SIZE];
char    str[32];
int     tx_sensors_mask = 1; // we will play a rotation, multiple sensors can be selected also

eSensorData     _data;
eWaveStruct_t   _user_wave_buf[TDK_USSM_WAVE_BUF_SIZE];
//---------------------------------
// Functions prototypes

//-- Arduin MEGA5260 HAL Functions
int  halATMEGA2560_SendReceiveA(int device_mask, int tx_mask, eWaveStruct_t *_rx_buf, int _rx_buf_size) ;
int  halATMEGA2560_EnvSendReceiveA(int device_mask, int tx_mask, int rx_index, eWaveStruct_t *_rx_buf, int _rx_buf_size, int sample_dly_ms) ;
void inline halATMEGA2560_FastWrite(int pin, int val, int nBit=N_SENSORS);
uint16_t inline halATMEGA2560_FastRead(void);
void halATMEGA2560_PortMode(uint8_t pin , uint8_t mode);

//-- Print Formating
void printHexArray(const char *cname, int id, uint8_t *pbuf, int nbyte);
void printPassFail(const char *cname, int id, bool pass);

//-- Test Functions
void testSensorRegistersRead(int n);
void testSensorRegisterWrite(int n); 
void testSensorRegisters(int device_mask);

void testPrintRxWave(void);

void testSensorDistanceMm(int n);
void testSensorDistanceStreamout(int device_mask);

void testSendReceive(int device_mask, int tx_mask); 
void testSendReceiveStreamout(int device_mask, int tx_mask, int mode=0);

void testSendReceiveEnvelop(int device_mask, int tx_mask, int rx_index, int n_sample, int sample_dly_us);

//---------------------------------
//-- Setup 
void setup()
{ 
  Serial.begin(115200); 
 
  //-- TEST0: Read Write & Check All Sensor Registers
  testSensorRegisters(ALL_SENSORS_MASK);
 
}

//---------------------------------
//-- Runtime.
void loop()
{ 
  int cntr = 0;

  while(1)
  { 

      //-----------------------------------------
      // Basic Register Read/Write Test
      //-----------------------------------------
      //-- TEST0: Read Write & Check All Sensor Registers
      //-- Visualize on: Serial Monitor 
      testSensorRegisters(ALL_SENSORS_MASK);  delay(500);      //--> Uncomment me to test
    
      //-----------------------------------------
      // Cascaded Measurements (1 Sensor at time)
      //-----------------------------------------
      //-- TEST1: Simple 1st Echo Distance Measurement
      //-- Visualize on: Serial Monitor 
      testSensorDistanceMm(ALL_SENSORS_MASK);                 //--> Uncomment me to test

      //-- TEST2: 1st Echo Distance Streamout for easy visualization in Serial Tools->Plotter
      //-- Visualize on: Serial Plotter or Monitor 
      testSensorDistanceStreamout(ALL_SENSORS_MASK);          //--> Uncomment me to test

      //-----------------------------------------
      //-- Parallel Measurements (Send/Receive)
      //-----------------------------------------
      //-- TEST3: Detailed Measurement
      //-- Visualize on: Serial Monitor
      //-- 3.1: All Sensors in Send Receive : Arduino MEGA256 can NOT captured properly
      testSendReceive(ALL_SENSORS_MASK, 0x1111); delay(500);  //--> Uncomment me to test
      //-- 3.2: One Sensor in Send Receive : Arduino MEGA 2560 CAN capture and decode Status bits properly
      testSendReceive(0x1, 0x1); delay(500);  //--> Uncomment me to test
      //-- 3.3: Two Sensors in Send Receive : Arduino MEGA 2560,can NOT capture status bits properly
      testSendReceive(0x3, 0x1); delay(500);  //--> Uncomment me to test 

      //-- TEST4: 1st Echo Distance Streamout from concurrent Measurement
      //-- Visualize on: Serial Plotter or Monitor 
      testSendReceiveStreamout(ALL_SENSORS_MASK, 0x1111);     //--> Uncomment me to test

      //-- TEST5: 1st Echo Distance & Echo height Streamout from concurrent Measurement
      //-- Visualize on: Serial Plotter or Monitor 
      testSendReceiveStreamout(0x1, 0x1, 1);     //--> Uncomment me to test


      //-----------------------------------------
      //-- Analog Envelop Measurement (Send/Receive)
      //-----------------------------------------
      //-- TEST20: Analog Envelop Streamout for 1 Selected sensor using rx_index
      //-- Visualize on: Serial Plotter or Monitor 
      testSendReceiveEnvelop(ALL_SENSORS_MASK, 0x1111, 0, 128, 0); delay(50);  //--> Uncomment me to test : Envelop of Sending Sensor 0
      testSendReceiveEnvelop(ALL_SENSORS_MASK, 0x1111, 1, 128, 0); delay(50);  //--> Uncomment me to test : Enevelop of Receiving Sensor 1 


      //-----------------------------------------
      //-- RxWave if Needed (Not a Sensor Function)
      //-----------------------------------------
      //-- TEST30: PRint Rx Wave of previous operation if needed
      //-- Visualize on: Serial Monitor 
      testPrintRxWave();                                    //--> Uncomment me to test
      
      cntr++;
  }
}



//-------------------------------------------------
//-----<<< HELP FUNCTIONS FOR VSIUALIZATION >>>----
//-------------------------------------------------
void printHexArray(const char *cname, int id, uint8_t *pbuf, int nbyte)
{
  sprintf(str,"%s[%d] = { ", cname, id);
  Serial.print(str);
  for(int i=nbyte-1; i>=0; i--) // MSB First
  {
    sprintf(str,"%02x ", pbuf[i]);
    Serial.print(str);
  }
  Serial.println("}"); 
}


//------------
void printPassFail(const char *cname, int id, bool pass)
{
  sprintf(str,"%s[%d] = ", cname, id);
  Serial.print(str);
  if(pass){
     Serial.println("{ PASS }");
  } else {
    Serial.println("{ FAIL }");
  }
}

//-------------------------------------------------
//---<<< TEST & EVAL FUNCTIONS FOR REFERENCE >>>---
//-------------------------------------------------
void testSensorRegisters(int device_mask)
{
  for(int i=0; i<N_SENSORS; i++) 
  {
    if((1<<i) & device_mask) 
    { // Enabled 
      testSensorRegistersRead(i);
      testSensorRegisterWrite(i); 
    }
  }
}

//-----------
void testSensorRegistersRead(int n)
{
  TdkUssm.ReadId(buf, n);         printHexArray("READ_ID", n, buf, 3);        delay(20);
  TdkUssm.ReadEeprom(buf, n);     printHexArray("EE_READ", n, buf, 28);       delay(20);
  TdkUssm.CalibRead(buf, n);      printHexArray("CALIB_READ", n, buf, 10);    delay(20);
  TdkUssm.MeasRead(buf, n);       printHexArray("MEAS_READ", n, buf, 10);     delay(20);
  TdkUssm.ThresARead(buf, n);     printHexArray("THRES_A_READ", n, buf, 10);  delay(20);
  TdkUssm.ThresBRead(buf, n);     printHexArray("THRES_B_READ", n, buf, 10);  delay(20);
  TdkUssm.ReadStatus(buf, n);     printHexArray("READ_STATUS", n, buf, 3);    delay(20);
  TdkUssm.ReadNfdStatus(buf, n);  printHexArray("READ_NFD_STATUS", n, buf, 3);delay(20);
  TdkUssm.ReadTemp(buf, n);       printHexArray("READ_TEMP", n, buf, 2);      delay(20);
}

//-----------
void testSensorRegisterWrite(int n)
{
  uint8_t rd_data[32];
  uint8_t wr_data[32];
  uint8_t my_data = 0xab;
  bool    pass = false;

  //-- Clear buffer
  memset(rd_data, 0, 32);
  //-- Read register data
  TdkUssm.CalibRead(rd_data, n);      printHexArray("CALIB_READ", n, rd_data, 10);    delay(100);

  //-- Copy some data as it contains critical settings
  memcpy(wr_data, rd_data, 32);

  //-- Change Customer register field bytes 9 an 10 (32 bit LSB)
  wr_data[1] = n; // differentiate data for each sensor
  wr_data[0] = my_data + n;
  TdkUssm.CalibWrite(wr_data, n);     printHexArray("CALIB_WRITE", n, wr_data, 10);    delay(100);

  //-- Read back 
  TdkUssm.CalibRead(rd_data, n);      printHexArray("CALIB_READ", n, rd_data, 10);    delay(100);

  //-- Check
  if((wr_data[0] ==rd_data[0]) && (wr_data[1] ==rd_data[1]))
  {
    pass = true;
  }
  printPassFail("REG_WRITE_READ_CHECK", n, pass);
   
}


//---------
void testSensorDistanceMm(int device_mask)
{
  for(int i=0; i<N_SENSORS; i++) 
  {
    if((1<<i) & device_mask) 
    { // Enabled 
      sprintf(str,"DISTANCE_MM[%d] = { %5d }", i, TdkUssm.GetDistanceMm(i));
      Serial.println(str);
    }
  }
  Serial.println();
}



//---------
void testSensorDistanceStreamout(int device_mask)
{
  for(int i=0; i<N_SENSORS; i++) {
    if((1<<i) & device_mask) 
    { // Enabled 
      sprintf(str,"%5d ", TdkUssm.GetDistanceMm(i));
      Serial.print(str);
    }
  }
  Serial.println();
  //-- You have all sensors distances in distances buffer if needed for post-ptocessin :-)
}



//---------
void testSendReceiveStreamout(int device_mask, int tx_mask, int mode)
{
  int echo_sel=0;
  int rx_ptr=0;

  #ifdef ARDUINO_AVR_MEGA2560
     rx_ptr = halATMEGA2560_SendReceiveA(device_mask, tx_mask, &_user_wave_buf[0], TDK_USSM_WAVE_BUF_SIZE);
  #else
     rx_ptr = TdkUssm.WaveSendReceive(device_mask, tx_mask, CMD_SEND_RECEIVE_A, &_user_wave_buf[0], TDK_USSM_WAVE_BUF_SIZE, 0);
  #endif
 

  //-- Process and Print result
  // for(int d=N_SENSORS-1; d>=0; d--) 
  for(int d=0; d<N_SENSORS; d++) 
  {
    if((1<<d) & device_mask) 
    { // Enabled 
      TdkUssm.WaveDecodeData(d, CMD_SEND_RECEIVE_A, 2, &_user_wave_buf[0] , rx_ptr, &_data, TDK_USSM_DATA_SIZE, 3, 0ul);

      switch(mode)
      {
        default:
          //-- First Echo on Sending/Tx Sensor is the burst itself.
          echo_sel = (0 != ((1<<d) & tx_mask)) ? 1 : 0; 
          sprintf(str,"%5d ", _data.distances[echo_sel]);
          Serial.print(str);
          break;
       case 1 : 
          //-- First Echo Distance and Echo Height.
          echo_sel = (0 != ((1<<d) & tx_mask)) ? 1 : 0; 
          sprintf(str,"%5d %3d ", _data.distances[echo_sel], _data.hight_echos[0]);
          Serial.print(str);
          break;       
      }
    }
  }
   Serial.println();
}



//---------
void testSendReceive(int device_mask, int tx_mask)
{
  uint8_t d=1, max_echos=3;

  uint32_t t0, ti;
  eSensorData  *pdata = &_data;
  int rx_ptr=0;


  int masterMask = tx_mask;
  int deviceMask = device_mask;


  #ifdef ARDUINO_AVR_MEGA2560
    rx_ptr = halATMEGA2560_SendReceiveA(device_mask, tx_mask, &_user_wave_buf[0], TDK_USSM_WAVE_BUF_SIZE);
  #else
    rx_ptr = TdkUssm.WaveSendReceive(device_mask, tx_mask, CMD_SEND_RECEIVE_A, &_user_wave_buf[0], TDK_USSM_WAVE_BUF_SIZE, 0);
  #endif
  
  sprintf(str,"SEND_RECEIVE [%0x , %0x] >>", device_mask, tx_mask); Serial.println(str);
  
  
  //-- Process and Print result
  for(d=0; d<N_SENSORS; d++) {
    if((1<<d) & device_mask) { // Enabled 
      TdkUssm.WaveDecodeData(d, CMD_SEND_RECEIVE_A, 2, &_user_wave_buf[0] , rx_ptr, &_data, TDK_USSM_DATA_SIZE, 3, 0ul);
          
      max_echos = pdata->n_echos;
      if(max_echos > 3) max_echos=3;

      //-- Status bits      
      printHexArray("STATUS", d,  pdata->echo_status, 3);
      sprintf(str,"N_Echos[%d] = { %d }", d,  pdata->n_echos); Serial.println(str);
      
      //-- Distance mm
      sprintf(str,"Distance_mm[%d] = { ", d); Serial.print(str);
      for(int e=0;e<max_echos; e++){
        sprintf(str,"%5d ",  pdata->distances[e]); Serial.print(str);
      }
      Serial.println("}");
      
      //-- Time of flights
      sprintf(str,"ToF_us[%d] = { ", d); Serial.print(str);
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
    }
  }
  Serial.println();
}


//---------
void testPrintRxWave(void)
{
  //-- RxWave
  sprintf(str,"RxWave[%d] = { ", TdkUssm._rx_buf_ptr); Serial.println(str);
  int t0 = _user_wave_buf[0].time_us;
  int ti = t0;
  for(int i=0; i<TdkUssm._rx_buf_ptr; i++) {
      // Serial.print(_user_wave_buf[i].time_us - t0);  // Absolute ToF
      Serial.print(_user_wave_buf[i].time_us - ti);       ti = _user_wave_buf[i].time_us; // Relative ToF (mainly for Status bit check
      sprintf(str," %02x", _user_wave_buf[i].val);
      Serial.println(str);
  }
  sprintf(str," } ");
}



//---------
void testSendReceiveEnvelop(int device_mask, int tx_mask, int rx_index, int n_sample, int sample_dly_us)
{
  int echo_sel=0;
  int rx_ptr=0;
  int skip_tx=0;
 
  #ifdef ARDUINO_AVR_MEGA2560
    rx_ptr = halATMEGA2560_EnvSendReceiveA(device_mask, tx_mask, rx_index, &_user_wave_buf[0], TDK_USSM_WAVE_BUF_SIZE, n_sample, sample_dly_us);
  #else
    Serial.println("[TODO]: TdkUssm_Envelop Send/Receive");
    return;
  #endif

  //-- Process and Print result
  for(int i=0; i<rx_ptr; i++) {
     Serial.println(buf[i]);
  }

 
//  for(int i=0; i<rx_ptr; i++) {
//    for(int d=0; d<N_SENSORS; d++) 
//    {
//      if((1<<d) & device_mask) 
//      { // Enabled 
//        sprintf(str,"%3d ", buf[i]);
//        Serial.print(str);
//      }
//    }
//    Serial.println();
//  }

}


//-------------------------------------------------
//-----<<< Arduino MEGA2560 HAL FUNCTIONS >>>------
//-------------------------------------------------
int halATMEGA2560_SendReceiveA(int device_mask, int tx_mask, eWaveStruct_t *_rx_buf, int _rx_buf_size)
{
  uint32_t t0, ti, timeout;
  bool  looped, done;
  uint16_t   val, val_z;
  int  _rx_buf_ptr = 0; 
  
  int dm = ~device_mask;
  int dtm = dm | tx_mask;

  //-- Set Active Sensors for Tx mode
  for(int i=0; i<N_SENSORS; i++) {
      if((1<<i)&device_mask) {
        pinMode(TriggerPin[i], OUTPUT);
      } 
  }
  
  //-- Trigger Measurements  
  halATMEGA2560_FastWrite(TriggerPin[0], dtm); 
  if(N_SENSORS>8) halATMEGA2560_FastWrite(TriggerPin[8], (dtm>>8)); 
  delayMicroseconds(50-10);
  
  halATMEGA2560_FastWrite(TriggerPin[0], dm);  
  if(N_SENSORS>8) halATMEGA2560_FastWrite(TriggerPin[8], (dm>>8));   
  delayMicroseconds(100-10);
  
  halATMEGA2560_FastWrite(TriggerPin[0], 0xff);
  if(N_SENSORS>8) halATMEGA2560_FastWrite(TriggerPin[8], 0xff);   

  //-- Capture Rx Wave
  for(int i=0; i<N_SENSORS; i++) {
      if((1<<i)&device_mask) {
        pinMode(EchoPin[i], INPUT_PULLUP);
      } 
  }
  
  t0          = micros();
  timeout     = t0 + 50000; //TIMEOUT; 
  looped      = (t0 > timeout) ? true : false;
  done        = false;  
  _rx_buf_ptr = 0;  // Reset _rx_buf_ptr
  val         = halATMEGA2560_FastRead();
  val_z       = ~val;  // force first data sampling

  while (! done) 
  {
    ti =  micros();
    val  = halATMEGA2560_FastRead(); 
    if(val != val_z) 
    {
      val_z = val;
      _rx_buf[_rx_buf_ptr].val      = val; 
      _rx_buf[_rx_buf_ptr].time_us  = ti; 
      _rx_buf_ptr++;
      if(_rx_buf_ptr >= _rx_buf_size){
        done = true; //  Force exit       
      }
    } else // Keep watching
    { 
      if(timeout < ti)
      {
        if(looped) {
          if((ti & 0xffff0000) == 0x0ul) done = true;
        } else {
          done = true;
        }
      }
    } // if changed
  }

  //-- Print Waveform for Debug
/*   
  char str[64];
  sprintf(str,"RxWave[0x%x][%d] = { ", device_mask, _rx_buf_ptr); Serial.println(str);
  t0 = _rx_buf[0].time_us;
  for(int i=0; i<_rx_buf_ptr; i++) {
      ti = _rx_buf[i].time_us - t0;
      sprintf(str," %02x", _rx_buf[i].val);
      Serial.print(str);
  }
  Serial.println();
*/
  //-- return number of vectors received
  return _rx_buf_ptr;
  
}


//---------------------
int  halATMEGA2560_EnvSendReceiveA(int device_mask, int tx_mask, int rx_index, eWaveStruct_t *_rx_buf, int _rx_buf_size, int n_sample, int sample_dly_us)
{
  int dm = ~device_mask;
  int dtm = dm | tx_mask;
  uint8_t sensor_list[N_SENSORS], n_active_sensor=0;
  uint32_t bitVal=0;
  int _tx_ptr=0, _rx_ptr=0, sample_cntr;
  
  uint8_t *_abuf = (uint8_t *)(&_rx_buf[0]);
  int      _abuf_size = BUF_SIZE; /* sizeof(eWaveStruct_t) * _rx_buf_size; */
  

  //----
  const eWaveGenCmd  WAV_ENV_SEND_RECEIVE[] = {
    { IO_OUT_BUS  , 0x00000000 , TCMD           },
    { IO_OUT_BUS  , 0xffffffff , TD             },
    { IO_OUT_BUS  , 0x00000000 , (1*TBIT_PHASE) }, // Send Bit =  1
    { IO_OUT_BUS  , 0xffffffff , (2*TBIT_PHASE) }, 
    { IO_OUT_BUS  , 0x00000000 , (2*TBIT_PHASE) }, // Send Bit =  0
    { IO_OUT_BUS  , 0xffffffff , (1*TBIT_PHASE) }, 
    { IO_OUT_BUS  , 0x00000000 , (2*TBIT_PHASE) }, // Send Bit =  0
    { IO_OUT_BUS  , 0xffffffff , (1*TBIT_PHASE) }, 
    { IO_OUT_BUS  , 0x00000000 , TBIT_PHASE     }, // start BIT0
    { IO_OUT_BUS  , 0x0000ffff , TBIT_PHASE     }, // SND = 0, REC = 1
    { IO_OUT_BUS  , 0xffffffff , TBIT_PHASE     }, // end BIT 0
    { IO_OUT_BUS  , 0x00000000 , TBIT_PHASE     }, // start BIT1
    { IO_OUT_BUS  , 0xffff0000 , TBIT_PHASE     }, // SND = 1, REC = 0
    { IO_OUT_BUS  , 0xffffffff , TBIT_PHASE     }, // end BIT 1
    { IO_INIT     , IO_IN_ANA  , 0           }, // Change IO Mode to Input
    { IO_IN_ANA   , 0xffffffff , 40000ul     },  // Analog In ==> Use ADC for sampling
    { IO_DONE     , 0xffffffff , 0           }
  };
  //---
  
  //-- Build Tx-Wave
  _tx_ptr = 0;
  while(IO_INIT != WAV_ENV_SEND_RECEIVE[_tx_ptr].cmd)
  {
    bitVal = WAV_ENV_SEND_RECEIVE[_tx_ptr].val;
    _rx_buf[_tx_ptr].time_us = WAV_ENV_SEND_RECEIVE[_tx_ptr].arg;
    _rx_buf[_tx_ptr].val     = ( ((bitVal>>WAVEGEN_SND_OFFSET) & tx_mask) | ((bitVal>>WAVEGEN_RCV_OFFSET) & ~tx_mask) | dm) & WAVEGEN_BUS_MASK; 
    _tx_ptr++; // Next Vector
  }

  //-- Set Active Sensors for Tx mode
  n_active_sensor = 0;
  for(int i=0; i<N_SENSORS; i++) {
      if((1<<i)&device_mask) {
        pinMode(TriggerPin[i], OUTPUT);
        sensor_list[n_active_sensor] = i; // Capture Active sensor
        n_active_sensor++;
      } 
  }

  //-- 
  n_active_sensor = 1; // TODO : This is only for Debug
  if(n_active_sensor > 0){
    if((n_sample*n_active_sensor) > _abuf_size) n_sample = _abuf_size/n_active_sensor; 
  }


  //-- Trigger Measurements
  for(int i=0; i<_tx_ptr; i++)
  {  
    halATMEGA2560_FastWrite(TriggerPin[0], _rx_buf[i].val); 
    if(N_SENSORS>8) halATMEGA2560_FastWrite(TriggerPin[0], (_rx_buf[i].val>>8)); 
    delayMicroseconds(_rx_buf[i].time_us); // Remove 10us to compensate CPU delay time
  }

 
  //-- Prepare pins for analog capture
  for(int i=0; i<N_SENSORS; i++) {
    if((1<<i)&device_mask) {
      pinMode(AnalogPin[i], INPUT_PULLUP);
    } 
  }


  //-- Capture Analog Envelop
  _rx_ptr = 0 ;
  for(int s=0; s<n_sample; s++){
     //-- Simplified Version
     buf[_rx_ptr++] = analogRead(AnalogPin[rx_index]);
     
     //-- Enhanced version
//    for(int i=0; i<n_active_sensor; i++){
//      buf[_rx_ptr] = analogRead(AnalogPin[sensor_list[i]]);
//      _rx_ptr++;
//    }
//    if(sample_dly_us) delayMicroseconds(sample_dly_us);
  }

  //-- Return number of samples
  return _rx_ptr;
}


//---------------------
void inline halATMEGA2560_FastWrite(int pin, int val, int nBit)
{
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *out;

  out = portOutputRegister(port);

  uint8_t oldSREG = SREG;
  cli();
  *out = val;
  SREG = oldSREG;
}


//---------------------
uint16_t inline halATMEGA2560_FastRead(void)
{
  volatile uint8_t port = digitalPinToPort(EchoPin[0]);
  volatile uint16_t val=0; 
  val  = *portInputRegister(port);
  if(N_SENSORS>8)
  {
     port = digitalPinToPort(EchoPin[8]);
     val |= (*portInputRegister(port)<<8);
  }
  return val;
}


//------------
void halATMEGA2560_PortMode(uint8_t pin , uint8_t mode)
{
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *reg, *out;

  if (port == NOT_A_PIN) return;

  reg = portModeRegister(port);
  out = portOutputRegister(port);    // everything above is to look up the registers for the given pin 

  if (mode == INPUT) { 
    uint8_t oldSREG = SREG;    // save current state register (for resuming interrupt)
    cli();                     // disable interrupts 
    *reg = 0;              // clear the bit e.g. 0111 &= ~0010 --> 0101
    *out = 0;
    SREG = oldSREG;            // restore state register (enable interrupts)
  } else if (mode == INPUT_PULLUP) {
    uint8_t oldSREG = SREG;
    cli();
    *reg = 0;
    *out = 0xff;               // set the bit e.g. 1000 |= 0010 --> 1010
    SREG = oldSREG;
  } else {
    uint8_t oldSREG = SREG;
    cli();
    *reg = 0xff;
    SREG = oldSREG;
  }
}
