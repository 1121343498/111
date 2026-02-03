/******************************************************************************************\
 *  (C) TDK Electronics GmbH & Co
 *  All rights reerved
 *  
 *  Title     : TDK_USSM_No_Library_cm
 *  Release   : V1.0
 *  Version   : 2022-08-18 
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

//-- Sensor(s) IO Pins configuration: 1 IO pin per sensor.
#define N_USSM_SENSORS  1 /*4*/ // To handle multiple sensors set N_USSM_SENSORS accordingly
const int ussm_io_pins[N_USSM_SENSORS] = {2}; /*{2, 3, 4, 5};*/ // add other sensor pins if needed.

//-- Ultrasound propagation and Sensor constants
#define SOUND_VELOCITY_CM     29
#define DISTANCE_OFFSET_CM     9
#define SOUND_VELOCITY_INCH   74
#define DISTANCE_OFFSET_INCH   4

//-- Help functions prototypes
long TDK_USSM_GetTimeOfFlight(int ioPin);
int  TDK_USSM_GetDistanceInch(int ioPin) { 
  return (TDK_USSM_GetTimeOfFlight(ioPin) / SOUND_VELOCITY_INCH / 2) - DISTANCE_OFFSET_INCH; 
}
int  TDK_USSM_GetDistanceCentimeter(int ioPin) { 
  return (TDK_USSM_GetTimeOfFlight(ioPin) / SOUND_VELOCITY_CM /2) - DISTANCE_OFFSET_CM; 
}

//-- Setup 
void setup()
{ 
  Serial.begin(115200); 
}

//-- Runtime.
void loop()
{ 
  while(1){
    //-- Enveloping in for loop to allow single or multiple sensors handling 
    for(int i=0; i<N_USSM_SENSORS; i++){ 
      Serial.print( TDK_USSM_GetDistanceCentimeter(ussm_io_pins[0]) ); // Distance in cm
      Serial.print(" "); // add some space to print values in line.
    }
    Serial.println(); // end of line = end of one measurement cycle
    delay(50); // Delay in ms to define frame rate. Could be set to 0 for maximum speed. 
  }
}



//-------------------------------------------
// TDK Ultrasonic Sensor Driver
// Time Of Flight Basic Polling function
// Arguments: ioPin 
// Returns  : >0 : Time-of-flight in us 
//            0  : if no echo received.
//-------------------------------------------
long TDK_USSM_GetTimeOfFlight(int ioPin)
{
  const long COMMAND_TIMEOUT = 18000ul; // 18 ms > default frame length 15ms
  const int  WAVE_BUFF_SIZE  = 4; // We need 3 edges: 2 for burst + 1 for echo
  
  int val=1, val_z, cntr=0;
  uint32_t timeout=0, t0, ti; 
  bool done = false;
  bool looped = false;
  long duration_us = 0;
  long _rx_buf[WAVE_BUFF_SIZE];
  int  _rx_buf_ptr = 0;
  int txp = ioPin;
  int rxp = ioPin;

  t0 =  micros();
  timeout = t0 + COMMAND_TIMEOUT; // Fixed frame timeout
  looped = (t0 > timeout) ? true : false;
  
  //-- Trigger/Tx Mode 
  pinMode(txp, OUTPUT);
  digitalWrite(txp, LOW);
  delayMicroseconds(100-20); // -20us to compensate pull-ups RC delay. 

  //-- Echo/Rx Mode  
  digitalWrite(txp, HIGH);
  delayMicroseconds(5);
  pinMode(rxp, INPUT_PULLUP);
  val = digitalRead(rxp);
  val_z = val;

  //-- Generic wave capture algorithm which can handle all sensor frames.
  _rx_buf_ptr = 0;
  while (! done) {
     val = digitalRead(rxp);
     ti =  micros();
     if(val != val_z) {
        val_z = val;
        _rx_buf[_rx_buf_ptr] = ti;
        _rx_buf_ptr++;
        if(_rx_buf_ptr >= WAVE_BUFF_SIZE){
           done = true; //  Force exit       
        }
     } else { // Keep watching
        if(timeout < ti){
          if(looped) {
            if((ti & 0xffff0000) == 0x0ul) done = true;
          } else {
            done = true;
          }
        }
     } // if changed
  } // while not yet done

  //-- Process the distance and send the value
  if(_rx_buf_ptr>=3) {
    duration_us = _rx_buf[3] - _rx_buf[1];  // neg-edge to neg-edge 
  } else {
    duration_us = 0;  
  }

  if((duration_us<600) || (duration_us>=COMMAND_TIMEOUT)) {
    return 0; // filter invalid measurements
  } else {
    return duration_us;
  }
}
