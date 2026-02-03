/*************************************************
 *  (C) TDK Electronics GmbH & Co
 *  All rights reerved
 *  
 *  Title     : TDK_USSM_Ultrasonic_Sensor
 *  Release   : V1.0
 *  Version   : 2023-08-07 
 *  Descrption: 
 *    TDK Ultrasonic Sensor(s) Class Implementation
 *
 *  -------- ABSOLUTE NO WARRENTY OF ANY KIND -------
 *
 *   THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, 
 *   EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
 *   OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE. MICROSOFT CORPORATION
 *   SHALL NOT BE LIABLE FOR ANY TECHNICAL OR EDITORIAL ERRORS OR OMISSIONS CONTAINED HEREIN. 
 * 
 *************************************************/


#include "TDK_USSM.h"
#include "tdk_ussm_platform.h"
#include "tdk_ussm_hal.h"
#include "tdk_ussm_defs.h"
#include "tdk_ussm_config.h"

////////////////////////////////////consttruct/destruct
void TDK_USSM::Init(const int txPin[],const int rxPin[], int n, const int anaPin[], int txLow, int txHigh, int rxPinMode, int rxLowUs)
{
    this->txPins    = (int*) txPin;
    this->rxPins    = (int*) rxPin;
    this->anaPins   = (int*) anaPin;
    this->nSensors  = n;
    
    this->IO_DRIVE_LOW      = txLow;
    this->IO_RELEASE        = txHigh;
    this->IO_IN_MODE        = rxPinMode;
    this->RX_TBIT_LOW_US    = rxLowUs;
    
    for(int i=0;i<n;i++)
    { 
        TdkUssm_pinMode(this->txPins[i], TdkUssm_OUTPUT);
        TdkUssm_digitalWrite(this->txPins[i], IO_RELEASE);
        TdkUssm_pinMode(this->rxPins[i], TdkUssm_INPUT); 
    }

    //-- HAL default Init
    _hal_timer_freq_hz  = 16000000;  // 16 MHz by default
    this->TimeAdjustUs  = 10; // 10us Timing Adjust by default 
       
}


TDK_USSM::TDK_USSM(int ioPin)
{
    this->Init(new int[1]{ioPin},new int[1]{ioPin},1);
}

TDK_USSM::TDK_USSM(int txPin,int rxPin)
{
    this->Init(new int[1]{txPin},new int[1]{rxPin},1);
}


TDK_USSM::TDK_USSM(const int ioPins[], int n, int txLow, int txHigh, int rxPinMode, int rxLowUs)
{
    this->Init(ioPins,ioPins,n, ioPins, txLow, txHigh, rxPinMode, rxLowUs);
}

TDK_USSM::TDK_USSM(const int txPin[],const int rxPin[],int n, const int anaPin[], int txLow, int txHigh, int rxPinMode, int rxLowUs)
{
    this->Init(txPin,rxPin,n, anaPin, txLow, txHigh, rxPinMode, rxLowUs);
}

TDK_USSM::~TDK_USSM()
{
    delete[] this->txPins;
    delete[] this->rxPins;
    delete[] this->anaPins;
    ~this->nSensors;
}

///////////////////////////////////////////////////
//-------------------------------------------
// TDK Ultrasonic Sensor Driver
// Time Of Flight Polling functions
//-------------------------------------------
long TDK_USSM::GetTimeOfFlight(int txPin, int rxPin) const
{
  const long cmd_timeout    = 18000ul;
  const long wave_buf_size  = 4;
  
  int val=1, val_z, cntr=0;
  uint32_t timeout=0, t0, ti; 
  bool done = false;
  bool looped = false;
  long duration_us = 0;
  long _rx_buf[wave_buf_size];
  int  _rx_buf_ptr = 0;
  int txp = txPin;
  int rxp = rxPin;

  t0 =  TdkUssm_micros();
  timeout = t0 + cmd_timeout; // 
  looped = (t0 > timeout) ? true : false;
  
  //-- Trigger/Tx Mode 
  TdkUssm_pinMode(txp, TdkUssm_OUTPUT);
  TdkUssm_digitalWrite(txp, IO_DRIVE_LOW);
  TdkUssm_delayMicroseconds(100-this->TimeAdjustUs ); 

  //-- Echo/Rx Mode  
  TdkUssm_digitalWrite(txp, IO_RELEASE);
  TdkUssm_delayMicroseconds(5);
  TdkUssm_pinMode(rxp, IO_IN_MODE);
  val = TdkUssm_digitalRead(rxp);
  val_z = val;
  
  _rx_buf_ptr = 0;
  while (! done) {
     val = TdkUssm_digitalRead(rxp);
     ti =  TdkUssm_micros();
     if(val != val_z) {
        val_z = val;
        _rx_buf[_rx_buf_ptr] = ti;
        _rx_buf_ptr++;
        if(_rx_buf_ptr >= wave_buf_size){
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
  } // while

  //-- Process the distance and send the value
  if(_rx_buf_ptr>=3) {
    duration_us = _rx_buf[3] - _rx_buf[1];
  } else {
    duration_us = 0;  
  }

  if((duration_us<0) || (duration_us>=cmd_timeout)) {
    return 0; // invalid measurements
  } else {
    return duration_us;
  }
}

//-------------------------------------------
// Cascaded Time-of-flight Functions / shortcuts
//-------------------------------------------
long TDK_USSM::GetTimeOfFlight()      const{return this->GetTimeOfFlight(0);}
long TDK_USSM::GetTimeOfFlight(int n) const{return this->GetTimeOfFlight(this->txPins[n], this->rxPins[n]);}

long TDK_USSM::GetDistanceMm()        const{return this->GetDistanceMm(0);}
long TDK_USSM::GetDistanceMm(int n)   const{return this->MicrosecondsToMillimeters(this->GetTimeOfFlight(n));}
long TDK_USSM::GetDistanceMm(int txPin, int rxPin)   const{return this->MicrosecondsToMillimeters(this->GetTimeOfFlight(txPin, rxPin));}

long TDK_USSM::GetDistanceCm()        const{return this->GetDistanceCm(0);}
long TDK_USSM::GetDistanceCm(int n)   const{return this->MicrosecondsToCentimeters(this->GetTimeOfFlight(n));}
long TDK_USSM::GetDistanceCm(int txPin, int rxPin)   const{return this->MicrosecondsToCentimeters(this->GetTimeOfFlight(txPin, rxPin));}

long TDK_USSM::GetDistanceInch()      const{return this->GetDistanceInch(0);}
long TDK_USSM::GetDistanceInch(int n) const{return this->MicrosecondsToInches(this->GetTimeOfFlight(n));}
long TDK_USSM::GetDistanceInch(int txPin, int rxPin)   const{return this->MicrosecondsToInches(this->GetTimeOfFlight(txPin, rxPin));}



//-------------------------------------------
// TDK Ultrasonic Sensor Driver
// Waveform Encoder
//-------------------------------------------
long TDK_USSM::WaveGenCapture(eWaveStruct_t *rxWave, int rxWaveSize,  int txrxWaveAppend, const eWaveGenCmd *inCmdWave, uint8_t *anaBuf, int anaBufSize,  int n_sample, int sample_time_us, uint8_t *txBits, int nTxBits, int devices, int masters) 
{
  int val=1, val_z;
  uint32_t timeout=0, t0, ti; 
  bool done = false;
  bool looped = false;
  long duration_us = 0;
  int sample_cntr = 0;
  int device_mask = ~devices;
  int master_mask = masters;
  int dma_flags = 0;
  const eWaveGenCmd *pwave;
  unsigned int  bitVal, bitDrive;

  _tx_buf_ptr   = 0;
  _rx_buf_ptr   = 0;
  eWaveStruct_t *_tx_buf = rxWave;
  eWaveStruct_t *_rx_buf = rxWave;

  //-- Init DMA if Any
  #ifdef USE_HAL_DMA
  pwave = inCmdWave;  
  while(pwave->cmd != IO_DONE){
    if(IO_INIT == pwave->cmd){
        HalDmaStart_CallBack(pwave->val, devices, anaBuf, anaBufSize, n_sample, sample_time_us);
        break;
    } 
    pwave++; 
  }
  #endif // USE_HAL_DMA

  //-- Command Wave Send (Tx)
  _tx_buf[_tx_buf_ptr].time_us = TdkUssm_micros();
  _tx_buf[_tx_buf_ptr++].val   = WAVEGEN_BUS_MASK;  
  for(int d=0; d<nSensors; d++) 
  {
    TdkUssm_pinMode(txPins[d], TdkUssm_OUTPUT); 
    TdkUssm_digitalWrite(txPins[d], IO_RELEASE);
  }

  //-- Start Wave Generation
  pwave = inCmdWave;  
  done = false;
  while(! done) 
  {
    switch(pwave->cmd)
    {
      case IO_INIT:
//        for(int d=0; d<N_SENSORS; d++) TdkUssm_digitalWrite(txPin[d], IO_RELEASE);
//        _tx_buf[_tx_buf_ptr].time_us = TdkUssm_micros();
//        _tx_buf[_tx_buf_ptr++].val   = WAVEGEN_BUS_MASK; 
        if(txrxWaveAppend) {
          _rx_buf = &_tx_buf[_tx_buf_ptr]; 
        } else {
          _rx_buf = rxWave; 
        }
        
        switch(pwave->val)
        {
          case IO_IN_ANA:
            TdkUssm_analogInInit();
            done=true;
            break;    
          case IO_IN_PU : 
          default :
            for(int d=0; d<nSensors; d++) {
               //--TODO: Move this to Timer Start HAL Function               
               if(!(dma_flags & (1ul<<d))) {
                  TdkUssm_pinMode(rxPins[d], IO_IN_MODE);  
               } else {
                
               }
               
            }
            done=true;
            break;
        } // end IO mode change
        pwave++; // Next wave vector
        break;
        
      case IO_OUT_0:
        _tx_buf[_tx_buf_ptr].time_us = TdkUssm_micros();
        _tx_buf[_tx_buf_ptr++].val   = device_mask; 
        for(int d=0; d<nSensors; d++) { 
          if((1<<d) & device_mask) TdkUssm_digitalWrite(txPins[d], IO_RELEASE);
          else TdkUssm_digitalWrite(txPins[d], IO_DRIVE_LOW);
        }
        TdkUssm_delayMicroseconds(pwave->arg); 
        pwave++; // Next wave vector
       break;
        
      case IO_OUT_1:
        _tx_buf[_tx_buf_ptr].time_us = TdkUssm_micros();
        _tx_buf[_tx_buf_ptr++].val   = WAVEGEN_BUS_MASK; 
        for(int d=0; d<nSensors; d++) TdkUssm_digitalWrite(txPins[d], IO_RELEASE);
        TdkUssm_delayMicroseconds(pwave->arg); 
        pwave++; // Next wave vector
       break;
        
      case IO_OUT_PROG:
        // if(progPins == NULL)
        // {
        //   pwave++;
        //   break;
        // }
        bitVal   = pwave->val;
        bitDrive = bitVal & ~device_mask; // only enabled devices
        _tx_buf[_tx_buf_ptr].time_us = TdkUssm_micros();
        _tx_buf[_tx_buf_ptr++].val   = WAVEGEN_BUS_MASK; 
        for(int d=0; d<nSensors; d++) 
        {
          TdkUssm_digitalWrite(txPins[d], IO_RELEASE);
          // TdkUssm_pinMode(progPin[d], TdkUssm_OUTPUT);
          // if((1<<d) & bitDrive) TdkUssm_digitalWrite(progPin[d], PROG_ENABLE);
          // else TdkUssm_digitalWrite(progPin[d], PROG_DISABLE); // Apply VPP
        }
        TdkUssm_delayMicroseconds(pwave->arg); 
        pwave++; // Next wave vector
       break;    

      case IO_OUT_BUS: 
        bitVal = pwave->val;
        bitDrive = ( ((bitVal>>WAVEGEN_SND_OFFSET) & master_mask) | ((bitVal>>WAVEGEN_RCV_OFFSET) & ~master_mask) | device_mask) & WAVEGEN_BUS_MASK; 
        _tx_buf[_tx_buf_ptr].time_us = TdkUssm_micros();
        _tx_buf[_tx_buf_ptr++].val   = bitDrive; 
        for(int d=0; d<nSensors; d++)
        { 
          if((1<<d) & bitDrive) TdkUssm_digitalWrite(txPins[d], IO_RELEASE);
          else TdkUssm_digitalWrite(txPins[d], IO_DRIVE_LOW);
        }
        TdkUssm_delayMicroseconds(pwave->arg); 
        pwave++; // Next wave vector
       break;
        
      case IO_OUT_BIT: 
      case IO_OUT_VAL:
        //-- Prepare bit Drive Vector
        switch(pwave->cmd){
          case IO_OUT_BIT: 
            bitVal = pwave->arg;
            bitDrive = (bitVal) ?  WAVEGEN_BUS_MASK :  device_mask;
            pwave++; // Next wave vector
            break;
          case IO_OUT_VAL:
            nTxBits-- ;
            bitVal = (nTxBits>=0) ? txBits[nTxBits] : WAVEGEN_BUS_MASK;
            bitDrive = ( bitVal | device_mask) & WAVEGEN_BUS_MASK; 
            if(nTxBits <= 0){
              pwave++; // Next wave vector
            }
            break;
        }
        //-- Bit Phase 0
        _tx_buf[_tx_buf_ptr].time_us = TdkUssm_micros();
        _tx_buf[_tx_buf_ptr++].val   = device_mask; 
        for(int d=0; d<nSensors; d++)
        { 
          if((1<<d) & device_mask) TdkUssm_digitalWrite(txPins[d], IO_RELEASE);
          else TdkUssm_digitalWrite(txPins[d], IO_DRIVE_LOW);
        }
        TdkUssm_delayMicroseconds(TBIT_PHASE-2);  // Bit phase 0
        //-- Bit Phase 2
        _tx_buf[_tx_buf_ptr].time_us = TdkUssm_micros();
        _tx_buf[_tx_buf_ptr++].val   = bitDrive; 
        for(int d=0; d<nSensors; d++)
        { 
          if((1<<d) & bitDrive) TdkUssm_digitalWrite(txPins[d], IO_RELEASE);
          else TdkUssm_digitalWrite(txPins[d], IO_DRIVE_LOW);
        }
        TdkUssm_delayMicroseconds(TBIT_PHASE-2);  // Bit phase 1
        //-- Bit Phase 3
        _tx_buf[_tx_buf_ptr].time_us = TdkUssm_micros();
        _tx_buf[_tx_buf_ptr++].val   = WAVEGEN_BUS_MASK; 
        for(int d=0; d<nSensors; d++) TdkUssm_digitalWrite(txPins[d], IO_RELEASE);
        TdkUssm_delayMicroseconds(TBIT_PHASE-2);  // Bit phase 2 ==> Consider 10us for processing pre & post
        break; 
    
      case IO_DONE: 
        done = true;
        _rx_buf_ptr = 0;
        return 0;  
        break;
                
      default: // Just exit
        done = true; 
        pwave++; // Next wave vector
        break;
    } 
  } // end of Drive part


  //-- Command Wave Receive (Rx)
  done = false;  
  t0 =  TdkUssm_micros();
  timeout = t0 + pwave->arg; 
  looped = (t0 > timeout) ? true : false;
  _rx_buf_ptr = 0;
  switch(pwave->cmd)
  { 
    case IO_IN_EDGE: // Digital In Edges
      while (! done) {
        val = 0; 
        bitDrive=1;
        ti =  TdkUssm_micros();
        for(int d=0; d<nSensors; d++) 
        {
          bitVal = TdkUssm_digitalRead(rxPins[d]);
          if(bitVal) val |= bitDrive;
          bitDrive = bitDrive << 1;
        }
        val |= device_mask; // mask non needed waveforms
        if(val != val_z) 
        {
            val_z = val;
            _rx_buf[_rx_buf_ptr].time_us = ti;
            _rx_buf[_rx_buf_ptr].val =  val;  
            _rx_buf_ptr++;
            if(_rx_buf_ptr >= rxWaveSize){
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
      } // while
      break;

    case IO_IN_ANA: // Analog waveform capture
      sample_cntr = 0;
      done = false;
      if(anaPins == NULL) break;
      while (! done) 
      {
        if(sample_cntr < n_sample) {
          ti =  TdkUssm_micros();
          for(int d=0; d<nSensors; d++) 
          {
            if((1<<d) &  devices) // enabled devices
            {
              anaBuf[_rx_buf_ptr++] = HalRxAnaRead(d, 2); // (TdkUssm_analogRead((anaPin[d]) >> 2); // 10 bit to 8 bit
            }
          }
          sample_cntr++;
          if(_rx_buf_ptr >= anaBufSize)
          {
            done = true; //  Force exit       
          }
          val = sample_time_us - (TdkUssm_micros()-ti); // Auto-Correct sample time
          if(val>0) TdkUssm_delayMicroseconds(val);        
        } else {
          done = true;
        }
      }
      HalDmaStop_CallBack();
      break;
  } // End Switch

  //-- Return Status
  return TdkUssm_micros() - t0; // elapsed time on RX mode in us
}



//-------------------------------------------
// TDK Ultrasonic Sensor Driver
// Save Sensor Registers from bytes buffer
//-------------------------------------------
int TDK_USSM::SaveRegisterData(eSensorRegisters *pRegs, const int cmdId, uint8_t *pBytes) 
{
  //-- Copy data over
  switch(cmdId) {
    case CMD_THRES_A_READ   : 
    case CMD_THRES_A_WRITE  : 
        memcpy(pRegs->thres_a, pBytes, 11);     break;
    case CMD_THRES_B_READ   : 
    case CMD_THRES_B_WRITE  : 
        memcpy(pRegs->thres_b, pBytes, 11);     break;
    case CMD_MEAS_READ      : 
    case CMD_MEAS_WRITE     :     
        memcpy(pRegs->meas , pBytes, 10);       break;
    case CMD_CALIB_READ     : 
    case CMD_CALIB_WRITE     : 
        memcpy(pRegs->calib , pBytes, 10);      break;
    
    case CMD_READ_STATUS    : memcpy(pRegs->status , pBytes, 3);      break;
    case CMD_READ_TEMP      : // Temperature: temp_val = a*(temp -442) + 25
                              // temp_25 = 442 [430 454], slop a=0.513 [0.488 0.541]
                              {
                                int tv=0;
                                tv = GetBitsU32(pBytes, 0, 24);   
                                pRegs->temp_val = (0.513f * (tv - 442)) + 25;
                                memcpy(pRegs->temp , pBytes, 4); 
                              }
                              break;
    case CMD_READ_ID        : memcpy(pRegs->id , pBytes, 4);          break;
    case CMD_READ_NFD_STATUS: memcpy(pRegs->nfd_status , pBytes, 4);  break;
    case CMD_EE_READ        : memcpy(pRegs->ee_data , pBytes, 10);    break;
    default :
            // no data stored
          break;
  } // end switch
  return cmdId;
}

//-------------------------------------------
// TDK Ultrasonic Sensor Driver
// Waveform Decode into Bytes
//-------------------------------------------
long TDK_USSM::WaveDecodeBytes(int n, eWaveStruct_t *rxWave, int rxWaveSize, int stopPos, eSensorData *rxData, int rxByteSize)
{
  int dtl, dth;
  uint8_t bit_val = 0;
  uint32_t tp = 0, tn = 0, t0 = 0;  
  eIOState ios; 
  
  char str[100];

  //-- Default Selection
  eWaveStruct_t *_rx_buf       = &this->waveBuf[0];
  int           _rx_buf_size   = this->waveBufSize;
  eSensorData   *pdata         = &this->dataBuf;
  int           _rx_byte_size  = TDK_USSM_DATA_SIZE;
  int           rxBitId        = n; // Expecting Sensor wave at same bit ID as the sensor Id

  //-- Alternative Selections
  if(NULL != rxWave) 
  {
    _rx_buf       = rxWave;
    _rx_buf_size  = rxWaveSize;
  }
  
  if(NULL != rxData)
  {
    pdata         = rxData;
    _rx_byte_size = rxByteSize;
  } 


  //-- Prepare data rehister:  Clear to avoid leading non-0 values 
  pdata->rx_data_len = 0;
  memset(pdata->rx_data, 0, _rx_byte_size);  
  //-- Eval pulse waveform
  t0 = _rx_buf[0].time_us;
  tp = _rx_buf[_rx_buf_ptr-1].time_us - t0;
  tn = tp;
  for(int i=(_rx_buf_ptr-1); i>=stopPos; i--) {
    if(_rx_byte_size < GetBitsNbyte(pdata->rx_data_len)) break; // Stop if number of bits more than max bytes buffer size 
    ios = GetIoState(_rx_buf, i, rxBitId);
    switch(ios) {
      case IO_IN_POSEDGE :
        tp  = _rx_buf[i].time_us - t0;
        dth = tn - tp;
        break;  
      case IO_IN_NEGEDGE :
        tn  = _rx_buf[i].time_us - t0;
        dtl = tp - tn; 
       //-- TODO : Enhance Bit decoding using learnining algorithm instead of fixed Value
        bit_val = (dtl > TBIT0_CMP) ? 0 : 1;
        SetBit(pdata->rx_data, pdata->rx_data_len, bit_val);             
        pdata->rx_data_len++;                 
        break;
      default :
        break;
    } // end switch
  } // End for(rx_buf_ptr)

  //-- Process number of bytes received
  pdata->rx_data_nbyte = GetBitsNbyte(pdata->rx_data_len); 

  //-- Return number of bits decoded
  return pdata->rx_data_len; 
} 


//-------------------------------------------
// TDK Ultrasonic Sensor Driver
// Distance Processing Function
//-------------------------------------------
long TDK_USSM::WaveDecodeData(int n, const int cmdId, const int statusCfg, eWaveStruct_t *rxWave, int rxWaveSize, eSensorData *rxData, int rxByteSize, int maxEchos, uint32_t dmaMask)
{
  int      bitId    = n;
  int      sensorId = n;
  int      status_cfg=statusCfg;
  float    _hal_timer_freq_Mhz = ((float)_hal_timer_freq_hz)/1000000;
  eIOState ios;
  int      pcnt=0;  
  int      val0=0;
  int      sync_pos=0;
  float    t_pre, d_pre;
  uint32_t t0, tn_us, tp_us, dt, dist; 
  int      ref_index = 2;
  uint32_t ref_us    = 50;

  //-- Default Selection
  eWaveStruct_t *_rx_buf       = &this->waveBuf[0];
  int           _rx_buf_size   = this->_rx_buf_ptr;
  eSensorData   *pdata         = &this->dataBuf;
  int           _rx_byte_size  = TDK_USSM_DATA_SIZE;
  int           rxBitId        = n; // Expecting Sensor wave at same bit ID as the sensor Id
  int           max_echos      = maxEchos;
  int           cmd_id         = cmdId;

  //-- Alternative Selections
  if(NULL != rxWave) 
  {
    _rx_buf       = rxWave;
    _rx_buf_size  = rxWaveSize;
  }
  if(NULL != rxData)
  {
    pdata         = rxData;
    _rx_byte_size = rxByteSize;
  }


   //-- Clear buffers
  for(int i=0; i<max_echos; i++) {
    pdata->tn_echos[i]   = 0;
    pdata->len_echos[i]  = 0;
    pdata->distances[i]  = 0;
  }
  pdata->n_echos = 0;

  //-- Parse status bits
  switch(status_cfg){
    case 3 :
      sync_pos = _rx_buf_size - ((18+2)*2);
      pdata->n_echo_heights = 2;
      break;
    case 2 :
      sync_pos = _rx_buf_size - ((12+2)*2);
      pdata->n_echo_heights = 1;
      break;
    case 1 :
      sync_pos = _rx_buf_size - ((6+2)*2);
      pdata->n_echo_heights = 0;
      break;
    default :
      sync_pos = _rx_buf_size - ((0+2)*2); 
      pdata->n_echo_heights = 0;
      break;
  }   
  
  if(sync_pos<0) sync_pos = 0; 

  //--------------------------------
  //-- Extract Pulses data measurements
  //--------------------------------
  if(cmd_id == CMD_CAL_PULSE) {
    //--------------------------------
    //-- Extract Cal Pulses
    //-------------------------------- 
    pcnt=0;  
    val0=0;
    //-- Clear buffer
    for(int i=0; i<4; i++) { // Clear buffer
      pdata->cal_pulse[i] = 0ul;
    }

    //-- Evaluate cal_pulse 
    // By Default Use Software IO Sampling
    for(int i=0; i<_rx_buf_size; i++) {
      ios = GetIoState(_rx_buf, i, bitId);
      switch(ios) {
        case IO_IN_POSEDGE :
        case IO_IN_NEGEDGE :
              if(pcnt == 0) {
                val0 = _rx_buf[i].time_us - t0;
              }
              pdata->cal_pulse[pcnt++] = (_rx_buf[i].time_us - t0) - val0; 
            break;
        default: // do nothing
            break;
      }
      if(pcnt >= 4) {
        //-- Evaluate TCAL And frequency
        pdata->tcal_us  = pdata->cal_pulse[2]-pdata->cal_pulse[0]; 
        break;
      }
    }
    //-- If DMA Enabled reevaluate value from DMA Buffers
    #ifdef USE_HAL_DMA
    bool use_dma_buf = (dmaMask & (1<<n)) && (_hal_dma_buf[0] != 0);
    if(use_dma_buf){

      for(int i=0; i<4; i++){
        pdata->cal_pulse[i] = (_hal_dma_buf[i+12] - _hal_dma_buf[12])/_hal_timer_freq_Mhz;
      }
      float tcal_raw = _hal_dma_buf[12+2] - _hal_dma_buf[12];
      pdata->tcal_us  =  ((float)tcal_raw)/_hal_timer_freq_Mhz;
    } 
    #endif // USE_HAL_DMA
    
    //-- Eval Frequency
    if(pdata->tcal_us > 0) {
      pdata->freq_hz = (uint32_t)((496 * 1000000U)/ pdata->tcal_us);
    } else {
      pdata->freq_hz = 0; //Error invalid measurement or no sensor response
    }
  } else if(IsSendReceiveCommand(cmd_id)) {
    //--------------------------------
    //-- Extract Time of Fligh & Distances
    //--------------------------------  
    //-- Extract Status Bits
    WaveDecodeBytes(n, _rx_buf, _rx_buf_size, sync_pos, pdata, rxByteSize);
    memcpy(pdata->echo_status, pdata->rx_data, 3); // Max is 24 bits
    
    //-- Extract Time of flight
    ref_index = 0;
    for(int i=0; i<_rx_buf_size; i++) {  // Till Sync bits position
        if(ref_index < sync_pos) {
            if((IO_IN_NEGEDGE == GetIoState(_rx_buf, i, bitId))) {  // negedge
                tn_us = _rx_buf[i].time_us - _rx_buf[0].time_us;
                pdata->tn_echos[pdata->n_echos] = tn_us;
                ref_index++;
            } else if((IO_IN_POSEDGE  == GetIoState(_rx_buf, i, bitId)) && (pdata->tn_echos[pdata->n_echos]>0)){ // posedge
                tp_us = _rx_buf[i].time_us - _rx_buf[0].time_us;
                pdata->len_echos[pdata->n_echos] = tp_us - tn_us;
                t_pre = pdata->tn_echos[pdata->n_echos] - pdata->t_offset_us;
                d_pre = (int)MicrosecondsToMillimeters(t_pre); // (int)(t_pre * ((float)_const_c/(2.0*1000)));
                pdata->distances[pdata->n_echos] = d_pre - pdata->d_offset_mm; // Distance in mm. T is in us
                pdata->n_echos++;
                ref_index++;
            }
            if(pdata->n_echos >= max_echos) break; // Max Echos reached
            } else { // pulses extractions done
            i = _rx_buf_size; // move to the end
        }
    }
    
    //-- Extract Echo Hights
    pdata->hight_echos[0] = ( (GetBitsU32(pdata->rx_data,  6 ,11) & 0x3f)<<2);
    pdata->hight_echos[1] = ( (GetBitsU32(pdata->rx_data, 12 ,17) & 0x3f)<<2);

    //-- If DMA is used, correct data using DMA Buffer values
    #ifdef USE_HAL_DMA
    if((dmaMask&(1<<sensorId)) && (_hal_dma_buf[0]!=0)) { // If Channel has DMA-Timer
        //-- Ofssets for measurements
        if(IsSendReceiveA(cmd_id)) {
        ref_index = 1;
        ref_us = _hal_dma_buf[ref_index]; // posedge of cmd pulse
        } else {
        ref_index = 5;      
        ref_us = _hal_dma_buf[ref_index]+(50*_hal_timer_freq_Mhz); // Last posedge + TD (50us)  
        }   
        //-- Do Measurements
        pdata->n_echos=0;
        for(int i=0; i<sync_pos; i++) {  // Till Sync bits position
        if((i>ref_index) && !(i&0x1u)) {
            tn    = _hal_dma_buf[i];
            tp    = _hal_dma_buf[i+1];
            tn_us = (tn - ref_us)/_hal_timer_freq_Mhz;
            tp_us = (tp - ref_us)/_hal_timer_freq_Mhz;
            t_pre = tn_us - pdata->t_offset_us;
            d_pre = (int)MicrosecondsToMillimeters(t_pre); // d_pre = t_pre * ((float)_const_c/(2.0*1000));
            dt    = tp_us - tn_us;
            dist  = d_pre - pdata->d_offset_mm;
            //-- Store results
            pdata->tn_echos[pdata->n_echos]  = (uint32_t)tn_us;
            pdata->len_echos[pdata->n_echos] = (uint32_t)dt;
            pdata->distances[pdata->n_echos] = (int)dist; // Distance in mm. T is in us
            pdata->n_echos++;
        }
        if(pdata->n_echos >= max_echos) break; // Max Echos reached
        }
    } // If DMA Enabled
    #endif  // USE_HAL_DMA
  } // Is Send Receive command


 //-- Non Elmos ASIC Sensors
 //-- Simple trigger echo mode with time-of-flight coded on Echo pulse width
 #ifdef SENSOR_TYPES
  switch(SensorType[sensorId]) {
    case HC_SR04:
    case JSN_SR04T_V3:
        pdata->n_echos = 0;
        for(int i=1; i<_rx_buf_size; i++) {  // i=0 is default value set to 1 by default
          if((IO_IN_POSEDGE == GetIoState(_rx_buf, i, bitId))) { 
              tp_us = _rx_buf[i].time_us - _rx_buf[0].time_us;
              pdata->tn_echos[pdata->n_echos] = tp_us;
          } else if(IO_IN_NEGEDGE  == GetIoState(_rx_buf, i, bitId)){ 
              tn_us = _rx_buf[i].time_us - _rx_buf[0].time_us;
              t_pre = tn_us - tp_us;
              pdata->len_echos[pdata->n_echos] = tn_us;
              d_pre = (int)MicrosecondsToMillimeters(t_pre); // (int)(t_pre * ((float)_const_c/(2.0*1000)));
              pdata->distances[pdata->n_echos] = d_pre - 0; // pdata->d_offset_mm; // Distance in mm. T is in us
              pdata->n_echos++;
              i = _rx_buf_size;
              break; // all done !
          }
      }
  } // switch  
 #endif // Different SENSOR_TYPES

  return pdata->n_echos; // No pulse found
}



//-------------------------------------------
// Sensor Measurement Function
// Digital Send-Receive Generic Function
//------------------------------------------- 
long TDK_USSM::WaveSendReceive(int deviceMask, int masterMask, int cmdId, eWaveStruct_t *rxWave, int rxWaveSize, int skipTx, int timeDelayUs)
{
  uint32_t t0, ti, timeout;
  bool  looped, done;
  uint16_t   val, val_z;
  
  int deviceBitMask = ~deviceMask;
  int masterBitMask = ~masterMask;
 
  //-- Default Selections 
  eWaveStruct_t *_rx_buf      = &this->waveBuf[0];
  eWaveStruct_t *_tx_buf      = _rx_buf;
  int            _rx_buf_size = this->waveBufSize;
  int            timeDelay    = this->TimeAdjustUs;
  int icmd                    = cmdId;
    
  //-- Alternative Selections
  if(NULL != rxWave) 
  {
    _rx_buf = rxWave;
    _tx_buf = rxWave;
    _rx_buf_size = rxWaveSize;
  }
  if(-1 != timeDelayUs) 
  {
    timeDelay = timeDelayUs;
  } 
  
  //-- Init
  _rx_buf_ptr = 0;  
  _tx_buf_ptr = 0;  

  //-- Merge commands
  if(!skipTx) 
  {
      switch(cmdId){
        case CMD_SEND_C :
        case CMD_RECEIVE_C :
        case CMD_SEND_RECEIVE_C :
            icmd = CMD_SEND_RECEIVE_C;
            break;
        case CMD_SEND_B :
        case CMD_RECEIVE_B :
        case CMD_SEND_RECEIVE_B :
            icmd = CMD_SEND_RECEIVE_B;   
            break;
        case CMD_SEND_A :
        case CMD_RECEIVE_A :
        case CMD_SEND_RECEIVE_A :
            icmd = CMD_SEND_RECEIVE_A;   
            break;
        case CMD_ENV_SEND_A :
        case CMD_ENV_RECEIVE_A :
        case CMD_ENV_SEND_RECEIVE :
            icmd = CMD_ENV_SEND_RECEIVE;   
            break;
        case CMD_CAL_PULSE :
            icmd = CMD_CAL_PULSE;   
            break;
        default :
            return 0; // No commands no echos
      }

      //-- Prepare Tx_wave 
      _tx_buf_ptr = 0;  
      switch(icmd)
      {
           
        case CMD_SEND_RECEIVE_A:
              _tx_buf[_tx_buf_ptr].val = masterMask | deviceBitMask;      _tx_buf[_tx_buf_ptr++].time_us  = (TREC-TSND); 
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;                   _tx_buf[_tx_buf_ptr++].time_us  = TSND - timeDelay; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;                          _tx_buf[_tx_buf_ptr++].time_us  = TD - timeDelay; 
            break;
            
        case CMD_SEND_RECEIVE_B:
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;                   _tx_buf[_tx_buf_ptr++].time_us  = TMEAS - timeDelay; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;                          _tx_buf[_tx_buf_ptr++].time_us  = TD - timeDelay; 
              
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;                   _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = masterMask | deviceBitMask;      _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;                          _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE; 
              
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;                   _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;                   _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;                          _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE;
            break;
            
        case CMD_SEND_RECEIVE_C:
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;                   _tx_buf[_tx_buf_ptr++].time_us  = TMEAS - timeDelay; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;                          _tx_buf[_tx_buf_ptr++].time_us  = TD - timeDelay ; 
              
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;                   _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = masterMask | deviceBitMask;      _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;                          _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE;
              
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;                   _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;                          _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;                          _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE;
            break;
            
        case CMD_CAL_PULSE:
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;     _tx_buf[_tx_buf_ptr++].time_us  = TCMD - timeDelay; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;            _tx_buf[_tx_buf_ptr++].time_us  = TD - timeDelay; 
              // Bit0 = 0
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;     _tx_buf[_tx_buf_ptr++].time_us  = 2*TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;            _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE;
              // Bit1 = 0
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;     _tx_buf[_tx_buf_ptr++].time_us  = 2*TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;            _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE;
              // Bit2 = 0
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;     _tx_buf[_tx_buf_ptr++].time_us  = 2*TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;            _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE;
              // Bit3 = 0
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;     _tx_buf[_tx_buf_ptr++].time_us  = 2*TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;            _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE;
              // Bit4 = 1
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;     _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;            _tx_buf[_tx_buf_ptr++].time_us  = 2*TBIT_PHASE; 
            break;
            
        case CMD_ENV_SEND_RECEIVE:  //-- Analog Envelop over IO
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;                 _tx_buf[_tx_buf_ptr++].time_us  = TCMD - timeDelay; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;                        _tx_buf[_tx_buf_ptr++].time_us  = TD - timeDelay; 
              // Bit0 = 1
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;                 _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;                        _tx_buf[_tx_buf_ptr++].time_us  = 2*TBIT_PHASE;
              // Bit1 = 0
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;                 _tx_buf[_tx_buf_ptr++].time_us  = 2*TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;                        _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE;
              // Bit2 = 0
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;                 _tx_buf[_tx_buf_ptr++].time_us  = 2*TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;                        _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE;
              // Bit3 = 0 => SND = 0, REC = 1
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;                 _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = masterBitMask | deviceBitMask;    _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;                        _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE;
              // Bit4 = 0 => SND = 1, REC = 0
              _tx_buf[_tx_buf_ptr].val = deviceBitMask;                 _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = masterMask | deviceBitMask;     _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE; 
              _tx_buf[_tx_buf_ptr].val = 0xffff;                        _tx_buf[_tx_buf_ptr++].time_us  = TBIT_PHASE;
            break;    

        default  :
            return 0;
            break;
      }
      
      //-- In Inverted Drive then invert bits
      if(TdkUssm_LOW != IO_DRIVE_LOW)
      {
        for(int i=0; i<_tx_buf_ptr; i++) _tx_buf[i].val = ~_tx_buf[i].val; 
      }
      
      //-- Send Tx_wave
      HalTxInit(deviceMask, 0xffff);
      for(int i=0; i<_tx_buf_ptr; i++)
      {
        HalTxBusWrite(txPins, _tx_buf[i].val, nSensors);
        TdkUssm_delayMicroseconds(_tx_buf[i].time_us);
      }
  }
  
  //-- Capture Rx Wave 
  HalRxInit(deviceMask); // Init Rx Pins  
  _rx_buf_ptr = 0;  // Reset _rx_buf_ptr
  switch(icmd) 
  {
        case CMD_ENV_SEND_RECEIVE:
                // Nothing todo, will be handeled by envelopping function
            break; 
            
        default: 
            t0          = TdkUssm_micros();
            timeout     = t0 + TIMEOUT; 
            looped      = (t0 > timeout) ? true : false;
            done        = false;  
            val         = HalRxRead(deviceMask);
            val_z       = ~val;  // force first data sampling

            while (! done) 
            {
              ti =  TdkUssm_micros();
              val = HalRxRead(deviceMask) ; //| bit_mask;
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
            } // end while(!done)
            break;
  }

  //-- Print Waveform for Debug
/*  
  char str[64];
  sprintf(str,"RxWave[0x%x][%d] = { ", deviceMask, _rx_buf_ptr); Serial.println(str);
  t0 = _rx_buf[0].time_us;
  for(int i=0; i<_rx_buf_ptr; i++) {
      ti = _rx_buf[i].time_us - t0;
      sprintf(str,"%5d %02x", ti, _rx_buf[i].val);
      Serial.println(str);
  }
*/
  //-- return number of vectors received
  return _rx_buf_ptr;
}



//-------------------------------------------
// Send/Receive Functions Shortcut
//------------------------------------------- 
long TDK_USSM::CalPulse(int deviceMask) { return this->WaveSendReceive(deviceMask, deviceMask, CMD_CAL_PULSE, &this->waveBuf[0], this->waveBufSize);}

long TDK_USSM::SendA(int deviceMask) { return this->WaveSendReceive(deviceMask, deviceMask, CMD_SEND_A, &this->waveBuf[0], this->waveBufSize);}
long TDK_USSM::SendB(int deviceMask) { return this->WaveSendReceive(deviceMask, deviceMask, CMD_SEND_B, &this->waveBuf[0], this->waveBufSize);} 
long TDK_USSM::SendC(int deviceMask) { return this->WaveSendReceive(deviceMask, deviceMask, CMD_SEND_C, &this->waveBuf[0], this->waveBufSize);}  

long TDK_USSM::ReceiveA(int deviceMask) { return this->WaveSendReceive(deviceMask, 0, CMD_RECEIVE_A, &this->waveBuf[0], this->waveBufSize);}  
long TDK_USSM::ReceiveB(int deviceMask) { return this->WaveSendReceive(deviceMask, 0, CMD_RECEIVE_B, &this->waveBuf[0], this->waveBufSize);}  
long TDK_USSM::ReceiveC(int deviceMask) { return this->WaveSendReceive(deviceMask, 0, CMD_RECEIVE_C, &this->waveBuf[0], this->waveBufSize);} 

long TDK_USSM::SendReceiveA(int deviceMask, int masterMask) { return this->WaveSendReceive(deviceMask, masterMask, CMD_SEND_RECEIVE_A, &this->waveBuf[0], this->waveBufSize);} 
long TDK_USSM::SendReceiveB(int deviceMask, int masterMask) { return this->WaveSendReceive(deviceMask, masterMask, CMD_SEND_RECEIVE_B, &this->waveBuf[0], this->waveBufSize);}  
long TDK_USSM::SendReceiveC(int deviceMask, int masterMask) { return this->WaveSendReceive(deviceMask, masterMask, CMD_SEND_RECEIVE_C, &this->waveBuf[0], this->waveBufSize);}  

long TDK_USSM::EnvelopSendReceiveA(int deviceMask, int masterMask, uint8_t *pBuf, int bufSize, int nSample, int sampleTimeUs, int rightShift) 
{
    //--1. This will just send the command wvaform
    this->WaveSendReceive(deviceMask, masterMask, CMD_ENV_SEND_RECEIVE, &this->waveBuf[0], this->waveBufSize);
    //--2. Capture envelopping
    return HalRxAnalogEnvelopRead(deviceMask, pBuf, bufSize, nSample, sampleTimeUs, rightShift);
 }
 
long TDK_USSM::EnvelopSendA(int deviceMask, uint8_t *pBuf, int bufSize, int nSample, int sampleTimeUs, int rightShift)
{
    return this->EnvelopSendReceiveA(deviceMask, deviceMask, pBuf, bufSize, nSample, sampleTimeUs, rightShift) ;
}
  
long TDK_USSM::EnvelopReceiveA(int deviceMask, uint8_t *pBuf, int bufSize, int nSample, int sampleTimeUs, int rightShift) 
{
   return this->EnvelopSendReceiveA(deviceMask, 0, pBuf, bufSize, nSample, sampleTimeUs, rightShift);
}  

long TDK_USSM::JtagSendA(int deviceMask, eWaveStruct_t *rxWave, int rxWaveSize, uint8_t *pBuf, int bufSize, int nSample, int sampleTimeUs)  {return this->WaveGenCapture(rxWave, rxWaveSize, 0, WAV_JTAG_SEND_A, pBuf, bufSize, 0, 0, NULL, 0, deviceMask, deviceMask);}  
long TDK_USSM::JtagReceiveA(int deviceMask, eWaveStruct_t *rxWave, int rxWaveSize, uint8_t *pBuf, int bufSize, int nSample, int sampleTimeUs)  {return this->WaveGenCapture(rxWave, rxWaveSize, 0, WAV_JTAG_RECEIVE_A, pBuf, bufSize, 0, 0, NULL, 0, deviceMask, deviceMask);}  
long TDK_USSM::JtagSendReceiveA(int deviceMask, int masterMask, eWaveStruct_t *rxWave, int rxWaveSize,  uint8_t *pBuf, int bufSize, int nSample, int sampleTimeUs)  {return this->WaveGenCapture(rxWave, rxWaveSize, 0, WAV_JTAG_SEND_RECEIVE_A, pBuf, bufSize, 0, 0, NULL, 0, deviceMask, masterMask);}  
long TDK_USSM::JtagEnvelopThreshold(int deviceMask, int masterMask, eWaveStruct_t *rxWave, int rxWaveSize, uint8_t *pBuf, int bufSize, int nSample, int sampleTimeUs)  {return this->WaveGenCapture(rxWave, rxWaveSize, 0, WAV_JTAG_SEND_RECEIVE_A, pBuf, bufSize, 0, 0, NULL, 0, deviceMask, masterMask);}  
long TDK_USSM::JtagThreshold(int deviceMask, eWaveStruct_t *rxWave, int rxWaveSize, uint8_t *pBuf, int bufSize, int nSample, int sampleTimeUs)  {return this->WaveGenCapture(rxWave, rxWaveSize, 0, WAV_JTAG_SEND_RECEIVE_A, pBuf, bufSize, 0, 0, NULL, 0, deviceMask, deviceMask);}  


//-------------------------------------------
// Sensor Registers Read/Write
//-------------------------------------------
long TDK_USSM::RegisterReadWrite (const int txPin, const int rxPin, const int cmdId, uint8_t *rxBytes, int nRxBits,uint8_t *txBytes, int nTxBits) 
{
  int bitCntr = 0;
  int bitVal  = 0;
  int bitMask = 0;
  int bitId   = 0;
  int byteId  = 0;
  int bitDrive = IO_RELEASE;
  uint8_t partity_val  = 0;
  int n_parity_bits = 0;
  
  int val=1, val_z=1;
  uint32_t timeout=0, t0, ti, dt; 
  bool done = false;
  uint8_t   *bitBuf = (uint8_t *)&this->waveBuf[0]; // reuse of waveBuf in simple Byte mode
  int        bitBufSize = TDK_USSM_WAVE_BUF_SIZE * sizeof(eWaveStruct_t);
  
  const long COMMAND_TIMEOUT = TBIT*10; // Timeout after 4 bits Silence

  //-- Eval Parity If Needed
  partity_val = 0;
  n_parity_bits=0;
  if(txBytes != NULL) {
      switch(cmdId) {
        case CMD_MEAS_WRITE :
          n_parity_bits = 6;
          partity_val |= (this->GetEvenParity(txBytes,  0, 11) << 5);
          partity_val |= (this->GetEvenParity(txBytes, 12, 23) << 4);
          partity_val |= (this->GetEvenParity(txBytes, 24, 35) << 3);
          partity_val |= (this->GetEvenParity(txBytes, 36, 47) << 2);
          partity_val |= (this->GetEvenParity(txBytes, 48, 59) << 1);
          partity_val |= (this->GetEvenParity(txBytes, 60, 72) << 0);
          break;
        case CMD_THRES_A_WRITE :
        case CMD_THRES_B_WRITE :
          n_parity_bits = 6;
          partity_val |= (this->GetEvenParity(txBytes,  0, 12) << 5);
          partity_val |= (this->GetEvenParity(txBytes, 13, 25) << 4);
          partity_val |= (this->GetEvenParity(txBytes, 26, 38) << 3);
          partity_val |= (this->GetEvenParity(txBytes, 39, 51) << 2);
          partity_val |= (this->GetEvenParity(txBytes, 52, 65) << 1);
          partity_val |= (this->GetEvenParity(txBytes, 66, 79) << 0);
          break;
        default : // No partiy
          n_parity_bits=0;
          break;
      } // end switch
  }
  
  //-- Command Header
  TdkUssm_pinMode(txPin, TdkUssm_OUTPUT);
  TdkUssm_digitalWrite(txPin, IO_DRIVE_LOW);
  TdkUssm_delayMicroseconds(TCMD);  // TCMD
  TdkUssm_digitalWrite(txPin, IO_RELEASE);
  TdkUssm_delayMicroseconds(TD); // TD

  //-- Send Command Id
  bitMask = 1<<(5-1); // Command has 5 bits
  while(bitMask != 0){
      bitVal  = cmdId & bitMask;
      bitDrive = (bitVal == 0) ?  IO_DRIVE_LOW : IO_RELEASE;
      TdkUssm_digitalWrite(txPin, IO_DRIVE_LOW);
      TdkUssm_delayMicroseconds(40);  // Bit phase 0
      TdkUssm_digitalWrite(txPin, bitDrive);
      TdkUssm_delayMicroseconds(50);  // Bit phase 1
      TdkUssm_digitalWrite(txPin, IO_RELEASE);
      TdkUssm_delayMicroseconds(50);  // Bit phase 2 ==> Consider 10us for processing pre & post
      bitMask = bitMask >> 1; 
  }
  
  //-- Send Tx-bits if Any
  if((txBytes != NULL)&&(nTxBits>0)) {
      bitCntr=nTxBits;
      while(bitCntr > 0){
          bitCntr--;
          bitVal  = GetBit(txBytes, bitCntr);
          bitDrive = (bitVal == 0) ?  IO_DRIVE_LOW : IO_RELEASE;
          TdkUssm_digitalWrite(txPin, IO_DRIVE_LOW);
          TdkUssm_delayMicroseconds(40);  // Bit phase 0
          TdkUssm_digitalWrite(txPin, bitDrive);
          TdkUssm_delayMicroseconds(50);  // Bit phase 1
          TdkUssm_digitalWrite(txPin, IO_RELEASE);
          TdkUssm_delayMicroseconds(50);  // Bit phase 2 ==> Consider 10us for processing pre & post
      }
  }

  //-- Send Parity Bits if Any
  if(n_parity_bits>0) {
    bitMask = 1<<(n_parity_bits-1); // Command has 5 bits
    while(bitMask != 0){
        bitVal  = partity_val & bitMask;
        bitDrive = (bitVal == 0) ?  IO_DRIVE_LOW : IO_RELEASE;
        TdkUssm_digitalWrite(txPin, IO_DRIVE_LOW);
        TdkUssm_delayMicroseconds(50);  // Bit phase 0
        TdkUssm_digitalWrite(txPin, bitDrive);
        TdkUssm_delayMicroseconds(50);  // Bit phase 1
        TdkUssm_digitalWrite(txPin, IO_RELEASE);
        TdkUssm_delayMicroseconds(40);  // Bit phase 2 ==> Consider 10us for processing pre & post
        bitMask = bitMask >> 1; 
    }
  } else { // no parity
    bitMask = 0;
  }
  

  //-- Rx-bits read 
  if((nRxBits<=0) || (rxBytes == NULL)) {
    return nTxBits + n_parity_bits; // Wave elements
  } else { 
    //--------------------------------
    //-- Rx Bits if specified
    //-- Receive Rx-wave
    TdkUssm_pinMode(rxPin, IO_IN_MODE);
    val     = TdkUssm_digitalRead(rxPin);
    val_z   = val;
    t0      = TdkUssm_micros();
    done    = false;
    _rx_buf_ptr = 0;
    while (! done) {
         ti =  TdkUssm_micros();
         if(TdkUssm_HIGH == TdkUssm_digitalRead(rxPin)){
            if(TdkUssm_LOW == val_z) { // Posedge
                val_z = TdkUssm_HIGH;
                bitBuf[_rx_buf_ptr++] = (uint8_t)(ti-t0);
            } else {
                if((ti - t0) >= COMMAND_TIMEOUT) done=true;
                else if(bitCntr >= bitBufSize) done = true; // buffer max size reached                
            }
         } else { // Low
            if(TdkUssm_HIGH == val_z) { // negedge
                t0 = ti;
            }
            val_z = TdkUssm_LOW;
         }
    } // while progressing waveform
       
    //-- Clear Rx byte buffer
    byteId = 0;
    for(int i=0; i<nRxBits; i+=8){
      rxBytes[byteId++] = 0; // Clear
    }
    
    //-- Extract bits into bytes
    bitCntr=0;
    for(int i=_rx_buf_ptr-1; ((i>=0) && (bitCntr<nRxBits)); i--){
        if(bitBuf[i] >= (uint8_t)RX_TBIT_LOW_US) {
          bitVal  = 0;
        } else {
          bitVal  = 1;
        }
        bitId   = bitCntr%8;
        byteId  = bitCntr/8;
        if(bitVal) {
          rxBytes[byteId] |= 1<<bitId;
        }
        bitCntr++;
    } // for _rx_buf  
  } // end of Rx Bits
 
  //-- Return n Rx Bits        
  return (bitCntr); 
}


//-------------------------------------------
// Cascaded Register Functions / shortcuts
//-------------------------------------------
//-- Generic Registers Read/Write
long TDK_USSM::ReadRegister  (const int cmdId, uint8_t *rxBytes, int nRxBits, const int n, eSensorRegisters *sensorRegisters)
{ 
  long ret;
  ret = this->RegisterReadWrite(this->txPins[n], this->rxPins[n], cmdId, rxBytes, nRxBits, NULL, 0); // this->pWaveBuf, this->waveBufSize);
  if(NULL != sensorRegisters) 
  {
    this->SaveRegisterData(sensorRegisters,cmdId, rxBytes);
  }
  return ret;
}

long TDK_USSM::WriteRegister (const int cmdId, uint8_t *txBytes, int nTxBits, const int n, eSensorRegisters *sensorRegisters)
{
  long ret;
  ret = this->RegisterReadWrite(this->txPins[n], this->rxPins[n], cmdId, NULL, 0, txBytes, nTxBits); // this->pWaveBuf, this->waveBufSize);
  if(NULL != sensorRegisters) 
  {
    this->SaveRegisterData(sensorRegisters,cmdId, txBytes);
  }
  return ret;
}

//-- Single Register Read/Write functions
long TDK_USSM::CalibWrite     (uint8_t *txBytes,const int n)  {return this->WriteRegister(CMD_CALIB_WRITE  , txBytes, 74, n); }
long TDK_USSM::MeasWrite      (uint8_t *txBytes,const int n)  {return this->WriteRegister(CMD_MEAS_WRITE   , txBytes, 73, n); }
long TDK_USSM::ThresAWrite    (uint8_t *txBytes,const int n)  {return this->WriteRegister(CMD_THRES_A_WRITE, txBytes, 80, n); }
long TDK_USSM::ThresBWrite    (uint8_t *txBytes,const int n)  {return this->WriteRegister(CMD_THRES_B_WRITE, txBytes, 80, n); }
                                                             
long TDK_USSM::CalibRead      (uint8_t *rxBytes,const int n)  {return this->ReadRegister(CMD_CALIB_READ    , rxBytes, 74, n); }
long TDK_USSM::MeasRead       (uint8_t *rxBytes,const int n)  {return this->ReadRegister(CMD_MEAS_READ     , rxBytes, 73, n); }
long TDK_USSM::ThresARead     (uint8_t *rxBytes,const int n)  {return this->ReadRegister(CMD_THRES_A_READ  , rxBytes, 80, n); }
long TDK_USSM::ThresBRead     (uint8_t *rxBytes,const int n)  {return this->ReadRegister(CMD_THRES_B_READ  , rxBytes, 80, n); }
                                                             
long TDK_USSM::ReadId         (uint8_t *rxBytes,const int n)  {return this->ReadRegister(CMD_READ_ID       , rxBytes, 24, n); }    
long TDK_USSM::ReadStatus     (uint8_t *rxBytes,const int n)  {return this->ReadRegister(CMD_READ_STATUS   , rxBytes, 17, n); }  
long TDK_USSM::ReadTemp       (uint8_t *rxBytes,const int n)  {return this->ReadRegister(CMD_READ_TEMP     , rxBytes, 10, n); }
long TDK_USSM::ReadNfdStatus  (uint8_t *rxBytes,const int n)  {return this->ReadRegister(CMD_READ_NFD_STATUS,rxBytes, 24, n); }
long TDK_USSM::ReadEeprom     (uint8_t *rxBytes,const int n)  {return this->ReadRegister(CMD_EE_READ       , rxBytes,225, n);}

long TDK_USSM::Standby        (const int n)   {return this->WriteRegister(CMD_WAKE_UP  , NULL, 0, n); }
long TDK_USSM::Wakeup         (const int n)   {return this->WriteRegister(CMD_STANDBY  , NULL, 0, n); }

//-------------------------------------------
// EEPROM Programming
//-------------------------------------------
long TDK_USSM::EepromProg     (const int progPin, const int n, const int progON, const int progOFF)
{
    this->WriteRegister(CMD_EE_COPY  , NULL, 0, n);
      //-- Apply VPP (Programming Voltage)
    TdkUssm_pinMode(progPin, TdkUssm_OUTPUT);
    TdkUssm_digitalWrite(progPin, progON);
    TdkUssm_delayMicroseconds(TVPROG + TPROG);  // TPROG
    TdkUssm_digitalWrite(progPin, progOFF);
    TdkUssm_delayMicroseconds(TVPROG); 
    
    return 1;
}


//-------------------------------------------
// Helper Functions : Find Register Field parameters
//------------------------------------------- 
int TDK_USSM::parseFieldId (int fieldId, int* rid, int* eid)
{
  // Step 1 => decode Register Id 
  *rid = fieldId >> 8;
  *eid = fieldId & 0xff;
  
  // Check for Ids validity
  if(*rid == NULL_TABLE) return 0;
  if(*rid >= __N_ParamTableID) return 0;
  if(*eid >= C_PARAM_TABLES[*rid].size) return 0;
  return 1;
}

//-------------------------------------------
// Helper Functions : Register Field Get Real Value
//------------------------------------------- 
float TDK_USSM::RegisterFieldGetRealValue (int fieldId, int n)
{
    int rid, eid;
    // split fieldId in rid and eid, check for inconsistencies
    if (parseFieldId(fieldId, &rid, &eid) == 0) return 0;

    float result;
    GetRealValOfVal(C_PARAM_TABLES[rid].ptable[eid].lut, &result, RegisterFieldGet(fieldId, n), C_PARAM_TABLES[rid].ptable[eid].name);

    return result;
} 

//-------------------------------------------
// Helper Functions : Register Field Get
//------------------------------------------- 
long TDK_USSM::RegisterFieldGet (int fieldId, int n)  
{
  int rid, eid;
  uint32_t val;
  uint8_t  reg[30];
  
  // split fieldId in rid and eid, check for inconsistencies
  if (parseFieldId(fieldId, &rid, &eid) == 0) return 0;

  // Read register  
  ReadRegister(C_PARAM_TABLES[rid].readCmdId, reg, C_PARAM_TABLES[rid].regNBit, n);
  val =  GetBitsU32(reg, C_PARAM_TABLES[rid].ptable[eid].lsb, C_PARAM_TABLES[rid].ptable[eid].nbit);
  return val;
}

//-------------------------------------------
// Helper Functions : Register Field Set using Real value
//------------------------------------------- 
long TDK_USSM::RegisterFieldSetRealValue (int fieldId, int n, float newVal) 
{
  int rid, eid;

  // split fieldId in rid and eid, check for inconsistencies
  if (parseFieldId(fieldId, &rid, &eid) == 0) return 0;

  uint32_t result;
  GetValOfRealVal(C_PARAM_TABLES[rid].ptable[eid].lut, &result, newVal);
  return RegisterFieldSet(fieldId, n, result);
}

//-------------------------------------------
// Helper Functions : Register Field Set
//------------------------------------------- 
long TDK_USSM::RegisterFieldSet (int fieldId, int n, uint32_t newVal)  
{
  int rid, eid;
  uint8_t  reg[30];

  // split fieldId in rid and eid, check for inconsistencies
  if (parseFieldId(fieldId, &rid, &eid) == 0) return 0;

  // Read register  
  ReadRegister(C_PARAM_TABLES[rid].readCmdId, reg, C_PARAM_TABLES[rid].regNBit, n);
  SetBitsU32(reg, C_PARAM_TABLES[rid].ptable[eid].lsb, C_PARAM_TABLES[rid].ptable[eid].nbit, newVal);
  return WriteRegister(C_PARAM_TABLES[rid].writeCmdId, reg, C_PARAM_TABLES[rid].regNBit, n);
}


//-------------------------------------------
// Helper Functions : Get Register field Value from Real Value
//------------------------------------------- 
bool  TDK_USSM::GetValOfRealVal(const eLUTStruct *pLut, uint32_t *pVal, float real_val) const
{
  bool      valid = false;
  int       lut_size=0;
  float     gain;

   
  if(pLut == C_LUT_NULL) {
    return false;
  } else {
    lut_size=0;
    while(pLut[lut_size++].val != LUT_END); // Search Lut size
    for(int i=0; i<lut_size; i++){
      if(real_val == pLut[i].real_val) { // LUT
        *pVal = (uint32_t)pLut[i].val;
        return true;
      } else if( (i>0) && ((pLut[i-1].val+1) == (pLut[i].val)) 
                && (  ((real_val<pLut[i].real_val)&&(real_val>pLut[i-1].real_val))
                   || ((real_val>pLut[i].real_val)&&(real_val<pLut[i-1].real_val))
                   ) 
               ) { // Adjascent LUT entries
        if(abs(real_val - pLut[i-1].real_val) <= abs(real_val - pLut[i].real_val)) { // Select nearest value
          *pVal = (uint32_t)pLut[i-1].val;
        } else {
          *pVal = (uint32_t)pLut[i].val;
        }
        return true;
      } else if( (i>0) && // LIT: LINEARISATION TABLE
        (  ((real_val<pLut[i].real_val)&&(real_val>pLut[i-1].real_val))
        || ((real_val>pLut[i].real_val)&&(real_val<pLut[i-1].real_val))
        )) { // process in between
        if(pLut[i].real_val == pLut[i-1].real_val) { // avoid divsion by 0
          return false;
        } else {
          gain   = (pLut[i].val -  pLut[i-1].val) / (pLut[i].real_val -  pLut[i-1].real_val);
          *pVal  = (uint32_t)((gain * (real_val - pLut[i-1].real_val)) + pLut[i-1].val);
          return true;
        }
      } // end processing
    }
  }
  return false; // Not found or our of boundaries
}

//-------------------------------------------
// Helper Functions : Get Real Value of Register field value
//------------------------------------------- 
bool   TDK_USSM::GetRealValOfVal(const eLUTStruct *pLut, float *pRealVal, uint32_t val, const char *pDesc) const
{
  char fstr[16];
  bool                valid = false;
  int                 lut_size=0;
  float               gain;

  if(pLut == C_LUT_NULL) {
    return false;
  } else {
    lut_size=0; 
    while(pLut[lut_size++].val != LUT_END);
    for(int i=0; i<lut_size; i++){
      if(val == pLut[i].val) {  // LUT
        *pRealVal = pLut[i].real_val;
        // strcpy(pDesc, pLut[i].desc);
        return true;
      } else if( (i>0) && ((val<pLut[i].val)&&(val>pLut[i-1].val)) ) { // LIT: LINEARISATION TABLE
        gain   = (pLut[i].real_val -  pLut[i-1].real_val) / (pLut[i].val -  pLut[i-1].val);
        *pRealVal  = (gain * (val-pLut[i-1].val)) + pLut[i-1].real_val;
        // strcpy(pDesc, pLut[i].desc);
        return true;
      } // end processing
    }
  }
  return false; // Not found or our of boundaries
}

//-------------------------------------------
// Helper Functions : get bit value
//-------------------------------------------    
inline int   TDK_USSM::GetBit(uint8_t *buf, uint32_t bit_id) const
{
  uint32_t i, j;
  i = (bit_id >> 3); // div by 8
  j = (bit_id %  8); // reminder
  return ((buf[i] >> j) & (int)1);
}

//-------------------------------------------
// Helper Functions : set bit value
//------------------------------------------- 
inline int   TDK_USSM::SetBit(uint8_t *buf, uint32_t bit_id, int val) const 
{
  uint32_t i, j;
  i = (bit_id >> 3); // div by 8
  j  = (bit_id %  8); // reminder
  if(val) {
    buf[i] |= ((uint8_t)1<<j);
  } else {
    buf[i] &= ~((uint8_t)1<<j);
  }
  return ( buf[i] );
}


//-------------------------------------------
// Helper Functions : get bits as a uint32 value
//------------------------------------------- 
uint32_t  TDK_USSM::GetBitsU32(uint8_t *src, uint32_t src_lsb, uint32_t nbit) const 
{
  uint32_t val32=0ul;
  for(int i=0; i<nbit; i++) {
    val32 |= GetBit(src, src_lsb++) ? (1ul<<i): 0ul;
  }
  return val32;
}


//-------------------------------------------
// Helper Functions : set bit using a uint32 value
//------------------------------------------- 
uint32_t  TDK_USSM::SetBitsU32(uint8_t *dest, uint32_t dest_lsb, uint32_t nbit, uint32_t val32) const 
{
  for(int i=0; i<nbit; i++) {
    SetBit(dest, dest_lsb++, ((val32>>i)&0x1ul));
  }
  return (val32 & ((1<<nbit)-1)); // only valid bits returned
}


//-------------------------------------------
// Helper Functions : get bit range
//-------------------------------------------     
int   TDK_USSM::GetBitRange(uint8_t *dest, uint32_t dest_lsb, uint8_t *src, uint32_t src_lsb, uint32_t nbit) const
{
  for(int i=0; i<nbit; i++) {
    SetBit(dest, dest_lsb++, GetBit(src, src_lsb++));
  }
  return nbit;
}


//-------------------------------------------
// Helper Functions : get number of bytes for n bits
//------------------------------------------- 
uint32_t    TDK_USSM::GetBitsNbyte(uint32_t nbits) const
{
  uint32_t nbyte=0;
  nbyte = nbits>>3; // div 8
  if((nbyte * 8) < nbits) nbyte++;
  return nbyte;
}


//-------------------------------------------
// Helper Functions : get even parity
//------------------------------------------- 
int  TDK_USSM::GetEvenParity(uint8_t *pbuf, uint32_t lsb, uint32_t msb) const
{
  int  acc =0, val;

  for(int b=lsb; b<=msb; b++) {
    val = GetBit(pbuf, b);
    acc ^= val;
  }
  return acc;
}


//-------------------------------------------
// Helper Functions : Get IO State/Edge
//------------------------------------------- 
eIOState  TDK_USSM::GetIoState(eWaveStruct_t *pwave, uint32_t pos, uint8_t bit_id) const
{
  uint32_t bit_mask = 1<<bit_id;
  uint8_t io_st = 0;
  
  if(pos > 0ul) {
    io_st  = (pwave[pos-1].val & bit_mask) ? 1 : 0;
  } else { // Default state is high
    io_st  = 1;
  }
  io_st |= (pwave[pos  ].val & bit_mask) ? 2 : 0;
  
  switch(io_st) {
    case 0b00 : return IO_IN_0;
    case 0b01 : return IO_IN_NEGEDGE;
    case 0b10 : return IO_IN_POSEDGE;
    case 0b11 : return IO_IN_1;
    default   : return  IO_IN_VAL;
  }
}

//-------------------------------------------
// Helper Functions : IsSendReceiveA
//------------------------------------------- 
bool  TDK_USSM::IsSendReceiveA(int cmd_id) const
{
  if ( (cmd_id==CMD_SEND_A) || (cmd_id==CMD_RECEIVE_A) || (cmd_id==CMD_SEND_RECEIVE_A) ) return true;
  else return false;
}


//-------------------------------------------
// Helper Functions : IsSendReceiveA
//------------------------------------------- 
bool  TDK_USSM::IsSendReceiveCommand(int cmd_id) const
{
  if (  ( (cmd_id==CMD_SEND_A) || (cmd_id==CMD_RECEIVE_A) || (cmd_id==CMD_SEND_RECEIVE_A) ) 
     || ( (cmd_id==CMD_SEND_B) || (cmd_id==CMD_RECEIVE_B) || (cmd_id==CMD_SEND_RECEIVE_B) )
     || ( (cmd_id==CMD_SEND_C) || (cmd_id==CMD_RECEIVE_C) || (cmd_id==CMD_SEND_RECEIVE_C) )
     ) {
    return true;
  }
  return false;
}

//-------------------------------------------
// Helper Functions : IsValidCommand
//------------------------------------------- 
bool  TDK_USSM::IsValidCommand(int cmd_id) const
{
  for(int i=0; i<TDK_USSM_N_CMD; i++) 
  {
    if(cmd_id == (int)C_TDK_USSM_CMD[i].id) return true;
  }
  return false;
}


//-------------------------------------------
// Time-of-Flight to Distance Conversion
//-------------------------------------------
long TDK_USSM::MicrosecondsToMillimeters(long microseconds) const{
   return (microseconds*5) / 29; // us * 10 /29/2 => us *5 /29
}

long TDK_USSM::MicrosecondsToInches(long microseconds) const{
   return microseconds / 74 / 2;
}

long TDK_USSM::MicrosecondsToCentimeters(long microseconds) const{
   return microseconds/29/2; 
}


//-------------------------------------------
// HAL Functions : TxInit
//-------------------------------------------    
void inline TDK_USSM::HalTxInit(int val, int mask) 
{
  for(int d=0; d<nSensors; d++)
  { 
      if(mask & 0x1) // Enabled
      { 
        if((1<<d) & val) TdkUssm_digitalWrite(txPins[d], IO_RELEASE);
        else TdkUssm_digitalWrite(txPins[d], IO_DRIVE_LOW);
        TdkUssm_pinMode(txPins[d], TdkUssm_OUTPUT);
      }
      mask = mask>>1;
  }
}

//-------------------------------------------
// HAL Functions : TxWrite
//-------------------------------------------    
void inline TDK_USSM::HalTxWrite(int val, int mask) 
{
    for(int d=0; d<nSensors; d++)
    { 
      if(mask & 0x1) // Enabled
      { 
        if((1<<d) & val) TdkUssm_digitalWrite(txPins[d], IO_RELEASE);
        else TdkUssm_digitalWrite(txPins[d], IO_DRIVE_LOW);
      }
      mask = mask>>1;
    }
}


//-------------------------------------------
// HAL Functions : TxWrite
//-------------------------------------------  
void inline TDK_USSM::HalTxBusWrite(const int txPin[], int val, int nBit)
{
    for(int d=0; d<nBit; d++)
    { 
      TdkUssm_digitalWrite(txPins[d], (val&0x1));
      val = val>>1;
    }
}


//-------------------------------------------
// HAL Functions : RxInit
//-------------------------------------------  
void  inline TDK_USSM::HalRxInit(int mask) 
{
  for(int d=0; d<nSensors; d++)
  { 
    if(mask & (1<<d)) // Enabled
    {
     TdkUssm_pinMode(rxPins[d], IO_IN_MODE);
    }    
  }
}


//-------------------------------------------
// HAL Functions : RxRead
//-------------------------------------------  
int  inline TDK_USSM::HalRxRead(int mask,int defaultVal) 
{
  int val=0, bitMask=1; 
  for(int d=0; d<nSensors; d++)
  {
    if(TdkUssm_HIGH == TdkUssm_digitalRead(rxPins[d])) val |= bitMask;
    bitMask = (bitMask<<1); 
  }
  return val;
}

//-------------------------------------------
// HAL Functions : RxAnaRead Single
//-------------------------------------------  
int  inline TDK_USSM::HalRxAnaRead(int n, int rightShift) 
{
  #ifdef TDK_USSM_ADC_DMA_BUF
    return  TDK_USSM_ADC_DMA_BUF[n];
  #else
    return (TdkUssm_analogRead(anaPins[n]) >> rightShift);
  #endif
}


//-------------------------------------------
// HAL Functions : RxAnaRead Multiple
//-------------------------------------------  
int  inline TDK_USSM::HalRxAnalogEnvelopRead(int mask=WAVEGEN_BUS_MASK, uint8_t *pbuf, int bufSize, int nSample, int sampleTimUs, int rightShift) 
{
  int devAnaPin[TDK_USSM_MAX_DEVICES], dev_cnt, b;
  
  //-- Check parameters
  if((pbuf==NULL) || (bufSize<=0) || (nSample==0)) return 0; // No data to read

  //-- Check Buffer size
  if(bufSize < (nSample*dev_cnt)){
      nSample   = bufSize/dev_cnt;
  }
  
  //-- Get Active Sensor List
  dev_cnt=0;
  for(int d=0; d<nSensors; d++)
  { 
    if(mask & 0x1) // Enabled
    {   
      devAnaPin[dev_cnt]=anaPins[d];
      dev_cnt++;
    }
    mask = mask>>1;
  }
  
  
  //-- Sample 
  b=0; // bytes counter
  for(int s=0; s<nSample; s++)
  { 
    for(int d=0; d<dev_cnt; d++)
    { 
        pbuf[b++] = HalRxAnaRead(d, rightShift); // (TdkUssm_analogRead((devAnaPin[d])>> rightShift);
    }
    TdkUssm_delayMicroseconds(sampleTimUs);
  }
  return (nSample * dev_cnt);
}


//-------------------------------------------
// HAL Functions : HalDmaStart_CallBack
//------------------------------------------- 
int  TDK_USSM::HalDmaStart_CallBack(int mode, int devices, uint8_t *anaBuf, int anaBufSize, int n_sample, int sample_time_us) 
{
  // Must be overriden if used
  return 0; // No DMA Used
}


//-------------------------------------------
// HAL Functions : HalDmaStop_CallBack
//------------------------------------------- 
int  TDK_USSM::HalDmaStop_CallBack(void) 
{
  // Must be overriden if used
  return 0; // No DMA Used
}


//-------------------------------------------
// HAL Functions : HalSetTimerFreqHz
//------------------------------------------- 
void  TDK_USSM::HalSetTimerFreqHz  (uint32_t val)
{
  _hal_timer_freq_hz = val;
}
