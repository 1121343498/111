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


//-- Sensors Pinmap Sensors
#define N_SENSORS  4   // Can support up to 16 Sensors simulatnously
#define ALL_SENSORS_MASK ((1<<N_SENSORS)-1)

//-- Sensors IO Pins using voltage level-shifter adapater
const int IoPin[N_SENSORS] = { 2, 3, 4, 5 }; // Trigger & Echo on same pin 

//-- We select constructor using IOPins. use of TxPin, RxPin instance is also fine.
TDK_USSM TdkUssm(IoPin, N_SENSORS);  // Advanced Init to adapt to Driver

//-- Work Variables
uint8_t buf[32]; // for registers bytes
char    str[32]; // for printing

//---------------------------------
// Functions prototypes

//-- Print Formating
void printHexArray(const char *cname, int id, uint8_t *pbuf, int nbyte);
void printPassFail(const char *cname, int id, bool pass);

//-- Test Functions
void testSensorRegistersRead(int n);
void testSensorRegisterWrite(int n); 
void testSensorRegisters(int device_mask);

//---------------------------------
//-- Setup 
void setup()
{ 
  Serial.begin(115200); 
  Serial.println("TDK_USSM Sensors registers read/write test ..."); 
}

//---------------------------------
//-- Runtime.
void loop()
{ 
  int cntr = 0;

  while(1)
  { 
      testSensorRegisters(ALL_SENSORS_MASK);
      delay(2000);
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
  TdkUssm.ReadEeprom(buf, n);     printHexArray("EE_READ", n, buf, 29);       delay(20);
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
