/******************************************************************************************\
 *  (C) TDK Electronics GmbH & Co
 *  All rights reerved
 *  
 *  Title     : TDK_USSM_Chasm_Detection
 *  Release   : V1.0
 *  Version   : 2024-02-17 
 *  Desclaimer: This example is provided as a simple guidance WITHOUT ANY WARRENTY.
 *    
 *  DESCRIPTION:
 *  ===========
 *  This example code demonstrates a simple use of TDK Ultrsonic Sensor
 *  Arduino Library for detecting chasms (i.e., gaps) in the ground surface.
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
#include <tdk_ussm_config.h>

#define PC_Serial Serial
#define ComServer_Println(s)      PC_Serial.println(s)  // Macro for print-out
#define N_SENSORS                 2

#define ADC_BUF_SIZE  256
  
//-- Instanciate Single Sensor
TDK_USSM TdkUssm(A0); //Initialize Sensor Pins (IoPin)

uint8_t adc_buf[N_SENSORS];

//-- Setup 
void setup()
{ 
  Serial.begin(115200); 
}

//-- Runtime.
void loop()
{ 
  Chasm_Detection_Application();
}

// =============================================================================
// Chasm Detection Application Code => Transpiled from MATLAB version
// =============================================================================
void Chasm_Detection_Application() {
  //-- Initialise constants required for calculation
  int num_adc_samples = 100;                  // number of adc samples per envelope analysis
  int ts = 20;                                // sampling time of envelope analysis (ms)
  int real_time = 0;                          // placeholder for recording last timestamp in envelope analysis
  float max_threshold = 383;                  // minimum value for threshold values
  float min_threshold = 10;                   // maximum value for threshold values
  float alpha = 0.45;                         // slope controller for threshold value curves
  int missing_count = 0;                      // counter to keep track of how many consecutive iterations do not observe a ground echo
  int missing_reference = 5;                  // target reference to trigger STOP signal
  float detection_window[] = {20.5, 23.5};    // detection window for ground echo scans
  int no_agg_frames = 5;                      // number of adc samples per analysed iteration
  int ds = 0;                                 // sample distance
  uint32_t val = 0;                           // aux value for changing register values
  uint8_t reg[32];                            // aux placeholder for new register
  char str[64];                               // string placeholder variable for building variable printouts
  
  //-- Start Connection
  // ComServer_Listen();

  //-- Write configuration for MEAS register
  // NPULSES_A = 8; T_MEAS_A=8.75ms
  TdkUssm.RegisterFieldSetRealValue(NPULSE_A, 0, 8);
  TdkUssm.RegisterFieldSetRealValue(TMEAS_A, 0, 8.75);
 
  //-- Write initial configuration for CALIB register
  // V_DRV = default V_DRV + 1; G_ANA = default G_ANA + 1; G_DIG = default G_DIG

  //-- G_ANA
  TdkUssm.RegisterFieldSetRealValue(G_ANA, 0, 48.8);
  
  while(true) {
    uint16_t values[num_adc_samples];                      // carrier for cumulated values
    bool trigger = false;                                  // logic value for activating WARNING mode
    uint8_t curr_envelope[num_adc_samples];

    for(int i=0; i<num_adc_samples; i++) {
      values[i] = 0;
    }
    for (int i = 0; i < no_agg_frames; i++) {
      //-- Measure envelope and hold in 1d array
      real_time = Capture_Envelop(curr_envelope, num_adc_samples, ts);
      //-- Add envelope analysis in values 1d array
      for (int j = 0; j < num_adc_samples; j++) {
        values[j] = (uint16_t) (values[j] + curr_envelope[j]);      
      }
      
      //-- Add Inter-frame delay
      delay(20);
    }

    //-- Building threshold values array
    float threshold[num_adc_samples];             // placeholder array for threshold values
    for (int i = 0; i < num_adc_samples; i++) {      
      if (ts*i*1E-4*343/2 <= 15) {
        threshold[i] = max_threshold;
      }
      else {
        threshold[i] = max(threshold[i-1] * alpha, min_threshold);
      }
    }
    
    for (int i = 0; i < num_adc_samples; i++) {
      if(ts*i*1E-4*343/2 >= detection_window[0] && ts*i*1E-4*343/2 < detection_window[1] && values[i] > threshold[i]) {
        trigger = true;
        missing_count = 0;
        
        //-- Reset G_ANA & G_DIG to original values
        TdkUssm.RegisterFieldSetRealValue(G_ANA, 0, 48.8);
        TdkUssm.RegisterFieldSetRealValue(G_DIG, 0, 0.0);
      }
    }
    
    if (trigger == true) {
      ComServer_Println("OK");
    }

    //-- Trigger STOP state
    if(missing_count > missing_reference) {
      ComServer_Println("STOP! Chasm detected!");
    }

    //-- Trigger WARNING state
    if(!trigger) {
      missing_count = missing_count + 1;
      if (missing_count <= missing_reference) {
        ComServer_Println("WARNING! Ground echo not detected!"); 
      }
      if (missing_count < missing_reference && missing_count % 2 == 1) {
        //-- Increase G_ANA by one step & write to register
        val = TdkUssm.RegisterFieldGet(G_ANA, 0);
        if(val<7)  val++;
        TdkUssm.RegisterFieldSet(G_ANA, 0, val);
      }

      else if (missing_count < missing_reference && missing_count % 2 == 0) {
        //-- Increase G_DIG by 3 steps & write to register
        val = TdkUssm.RegisterFieldGet(G_DIG, 0);
        if(val<127)  val = val + 3;
        TdkUssm.RegisterFieldSet(G_DIG, 0, val); 
      }
    }
  }
}

uint32_t Capture_Envelop(uint8_t *env, int num_adc_samples, int ts){
  eTdkUssmCmdExecStruct myCmd;
  eTdkUssmCmdExecStruct *pCmd = &myCmd;
  
  uint8_t   adc_min[N_SENSORS];
  uint8_t   adc_max[N_SENSORS];
  float     adc_gain[N_SENSORS];
  char      str[128], str1[32], str2[128];
  int       cntr=0;
  int       str_size;
  long      tframe_us=0;
  int       real_ts_us = 0;
  int       adc_corrected_val=0;
  int       s_mask;

  //-- Build my command
  for (int i = 0; i < TDK_USSM_N_CMD; i++)
  {
    if (!strncmp(C_TDK_USSM_CMD[i].cmd, "esa", 3))
    {
      pCmd->pdef = &C_TDK_USSM_CMD[i];
    }
  }

    pCmd->masters         = 0x1;
    pCmd->devices         = 0x1; 
    pCmd->n_sample        = num_adc_samples;
    pCmd->sample_time_us  = ts;
   

  //-- Perform Measurement
  tframe_us = TdkUssm.EnvelopSendA(0x1, adc_buf, N_SENSORS);
  if(pCmd->n_sample <= 0) {
    real_ts_us = 0;
  } else {
    real_ts_us = tframe_us; 
  }

  //-- ADC Offset values
  cntr = 0;
  for(int d=0; d<N_SENSORS; d++) {   // Init
    adc_min[d] = 255;
     if(((0x1<<d) & pCmd->devices) != 0){   
      cntr++; // Active Sensors     
     }
  }
  for(int i=0; i<pCmd->n_sample; i=i){ // Search for offset value
    for(int d=0; d<N_SENSORS; d++) {
      if(((0x1<<d) & pCmd->devices) != 0){   
        if(adc_min[d]>adc_buf[i]) adc_min[d] = adc_buf[i];
        i++; // Next sample
      }
    }
  }
  
  //-- Result print with corrected values
  cntr = 0; tframe_us=0;
  for(int i=0; i<pCmd->n_sample; i++){
    env[i] = (uint8_t)((adc_buf[i] - adc_min[0]));
  }
  
  return real_ts_us;
}
