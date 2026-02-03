/*************************************************
 *  (C) TDK Electronics GmbH & Co
 *  All rights reerved
 *  
 *  Title     : TDK_USSM_Ultrasonic_Sensor
 *  Release   : V1.0
 *  Version   : 2023-08-07 
 *  Descrption: 
 *    TDK Ultrasonic Sensor(s) Class Definition
 *
 *  -------- ABSOLUTE NO WARRENTY OF ANY KIND -------
 *
 *   THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, 
 *   EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
 *   OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE. MICROSOFT CORPORATION
 *   SHALL NOT BE LIABLE FOR ANY TECHNICAL OR EDITORIAL ERRORS OR OMISSIONS CONTAINED HEREIN. 
 * 
 *************************************************/


#include "tdk_ussm_hal.h"
#include "tdk_ussm_defs.h"
class TDK_USSM
{
    public:
/**
 * \brief TDK_USSM constructor.
 * For a single sensor.
 * \param ioPin => Single Sensor IO Pin used as both Tx and Rx at same time.
 * \return None
 */
    TDK_USSM(int ioPin);                       //Constructor for single sensor with single pin operated in IO mode. 
   
/**
 * \brief TDK_USSM constructor.
 * For a single sensor.
 * \param txPin/rxPin => Single Sensor IO Pins, for transmission (txPin) and receiving (rxPin).
 * \return None
 */    
    TDK_USSM(int txPin,int rxPin);             //Constructor for single sensor mode (tx pin , rx pin). 
    
/**
 * \brief TDK_USSM constructor.
 * For multiple sensors.
 * \param ioPins[] => Array of Single Sensor IO Pins used for both transmission and receiving, one per sensor (ioPins).
 * \param n => Target device
 * \param txLow => LOW TX switch marker
 * \param txHigh => HIGH TX switch marker
 * \param rxPinMode => Pin Mode for RX pin
 * \param rxLowUs => TO DO (IO Delay ?)
 * \return None
 */
    TDK_USSM(const int ioPins[],int n, int txLow=TdkUssm_LOW, int txHigh=TdkUssm_HIGH, int rxPinMode=TdkUssm_INPUT_PULLUP, int rxLowUs=TBIT0_CMP);   //Constructor for multiple sensors full config

/**
 * \brief TDK_USSM constructor.
 * For multiple sensors.
 * \param txPin[]/rxPin[] => Single Sensor IO Pins arrays, two per sensor, for transmission (txPin) and receiving (rxPin).
 * \param n => Target device
 * \param anaPin[] => Analog pins, NULL by default
 * \param txLow => LOW TX switch marker
 * \param txHigh => HIGH TX switch marker
 * \param rxPinMode => Pin Mode for RX pin
 * \param rxLowUs => TO DO (IO Delay ?)
 * \return None
 */  
    TDK_USSM(const int txPin[],const int rxPin[],int n, const int anaPin[]=NULL, int txLow=TdkUssm_LOW, int txHigh=TdkUssm_HIGH, int rxPinMode=TdkUssm_INPUT_PULLUP, int rxLowUs=TBIT0_CMP);   //Constructor for multiple sensors full config
   
/**
 * \brief TDK_USSM destructor.
 * Compatible with any constructor.
 * \param None.
 * \return None
 */    
    ~TDK_USSM();                              


//-- SENSORS INIT FUNCTIONS
/**
 * \brief Sensor Initialisation Method.
 * Initialises sensor for communication. Can be used by constructor or runtime.
 * \param txPin[] => Array of pins used for data transmisssion
 * \param rxPin[] => Array of pins used for data receiving
 * \param n => Target device
 * \param anaPin[] => Analog pins, NULL by default
 * \param txLow => LOW TX switch marker
 * \param txHigh => HIGH TX switch marker
 * \param rxPinMode => Pin Mode for RX pin
 * \param rxLowUs => TO DO (IO Delay ?)
 * \return None
 */  
    void Init(const int txPin[],const int rxPin[], int n, const int anaPin[]=NULL, int txLow=TdkUssm_LOW, int txHigh=TdkUssm_HIGH, int rxPinMode=TdkUssm_INPUT, int rxLowUs=TBIT0_CMP); //for constructor or on Runtime

/**
 * \brief Sensor Initialisation Driving Method.
 * Enables sensor power-up. Can be used by constructor or runtime.
 * \param txLow => LOW TX switch marker
 * \param txHigh => HIGH TX switch marker
 * \param rxPinMode => Mode for RX pin
 * \param rxLowUs => TO DO (IO Delay ?)
 * \return None
 */  
    void InitDrive(int txLow=TdkUssm_LOW, int txHigh=TdkUssm_HIGH, int rxPinMode=TdkUssm_INPUT, int rxLowUs=TBIT0_CMP); //for constructor or on Runtime

//-- HARDWARE ABSTRACTION LAYER (HAL) IO FUNCTIONS
/**
 * \brief HAL Transmission Initialisation
 * TO DO
 * \param val => Value for initialisation ?
 * \param mask => Sensor mask
 * \return None.
 */ 
    void inline HalTxInit(int val, int mask=WAVEGEN_BUS_MASK) ;
    void inline HalTxWrite(int val, int mask=WAVEGEN_BUS_MASK) ;
    void inline HalTxBusWrite(const int txPin[], int val, int nbit) ;
    void (*pHalTxBusWrite)(const int txPin[], int val, int nbit);
    void inline HalRxInit(int mask=WAVEGEN_BUS_MASK) ; 
    int  inline HalRxRead(int mask=WAVEGEN_BUS_MASK, int defaultVal=1) ;
    int  inline HalRxAnaRead(int n, int rightShift=0) ;
    int  inline HalRxAnalogEnvelopRead(int mask, uint8_t *pbuf=NULL, int bufSize=0, int nSample=0, int sampleTimUs=10, int rightShift=2);
    int         HalDmaStart_CallBack(int mode, int devices, uint8_t *anaBuf=NULL, int anaBufSize=0, int n_sample=0, int sample_time_us=0) ;
    int         HalDmaStop_CallBack(void) ;
    void        HalSetTimerFreqHz(uint32_t val);



//-- BASIC TIME OF FLIGHT MEASUREMENT METHODS
/**
 * \brief Time of Flight, simple measurement, sensor 0.
 * Getter method that returns the ToF of sensor 0.
 * \param None
 * \return Time of Flight in us
 */ 
    long GetTimeOfFlight()      const; // returns time-of-flight in us of sensor 0
    
/**
 * \brief Time of Flight, simple measurement for a given sensor.
 * Getter method that returns the ToF of the specified sensor.
 * \param n => Target sensor
 * \return Time of Flight in us
 */ 
    long GetTimeOfFlight(int n) const; // returns time-of-flight in us of sensor n

/**
 * \brief Time of Flight, simple measurement, Tx/Rx connection.
 * Getter method that returns the ToF of sensor connected to a Tx/Rx pin pair.
 * \param txPin/rxPin => Pin pair for data transmission/receive
 * \return Time of Flight in us
 */ 
    long GetTimeOfFlight(int txPin, int rxPin) const; //returns time-of-flight in us of sensor connected to txPin / rxPin

//-- BASIC DISTANCE MEASUREMENT METHODS
/**
 * \brief Distance measurement in mm, sensor 0.
 * Getter method that returns the distance measurement in mm for sensor 0.
 * \param None
 * \return Measured distance (mm)
 */ 
    long GetDistanceMm()        const; // returns distance in mm of sensor 0

/**
 * \brief Distance measurement in mm, for a given sensor.
 * Getter method that returns the distance measurement in mm for the specified sensor
 * \param Target sensor
 * \return Measured distance (mm)
 */ 
    long GetDistanceMm(int n)   const; // returns distance in mm of sensor n

/**
 * \brief Distance measurement in mm, Tx/Rx connection.
 * Getter method that returns the distance measurement in mm for a sensor connected to a Tx/Rx pin pair
 * \param txPin/rxPin => Pin pair for data transmission/receive
 * \return Measured distance (mm)
 */ 
    long GetDistanceMm(int txPin, int rxPin)   const; // returns distance in mm of sensor n
    
/**
 * \brief Distance measurement in cm, sensor 0.
 * Getter method that returns the distance measurement in cm for sensor 0
 * \param None
 * \return Measured distance (cm)
 */     
    long GetDistanceCm()        const; // returns distance in cm of sensor 0

/**
 * \brief Distance measurement in cm, for a given sensor.
 * Getter method that returns the distance measurement in cm for the specified sensor
 * \param Target sensor
 * \return Measured distance (cm)
 */ 
    long GetDistanceCm(int n)   const; // returns distance in cm of sensor n

/**
 * \brief Distance measurement in cm, Tx/Rx connection.
 * Getter method that returns the distance measurement in cm for a sensor connected to a Tx/Rx pin pair
 * \param txPin/rxPin => Pin pair for data transmission/receive
 * \return Measured distance (cm)
 */ 
    long GetDistanceCm(int txPin, int rxPin)   const; // returns distance in mm of sensor n
    
/**
 * \brief Distance measurement in inches, sensor 0.
 * Getter method that returns the distance measurement in inches for sensor 0
 * \param None
 * \return Measured distance (inch)
 */     
    long GetDistanceInch()      const; // returns distance in inch of sensor 0

/**
 * \brief Distance measurement in inches, for a given sensor.
 * Getter method that returns the distance measurement in inches for the specified sensor
 * \param n => Target sensor
 * \return Measured distance (inch)
 */ 
    long GetDistanceInch(int n) const; // returns distance in inch of sensor n

/**
 * \brief Distance measurement in inches, Tx/Rx connection.
 * Getter method that returns the distance measurement in inches for a sensor connected to a Tx/Rx pin pair
 * \param txPin/rxPin => Pin pair for data transmission/receive
 * \return Measured distance (inch).
 */ 
    long GetDistanceInch(int txPin, int rxPin) const; // returns distance in inch of sensor n



//-- WAVE MANAGEMENT FUNCTIONS
    long WaveEncode(const int cmdId, eWaveStruct_t *rxWave, int rxWaveSize, int deviceMask=1, int masterMask=WAVEGEN_BUS_MASK);
    long WaveSendReceive(int deviceMask, int masterMask=WAVEGEN_BUS_MASK, int cmdId=CMD_SEND_RECEIVE_A, eWaveStruct_t *rxWave=NULL, int rxWaveSize=0, int skipTx=0, int timeDelayUs=-1);
    long WaveGenCapture(eWaveStruct_t *rxWave=NULL, int rxWaveSize=0, int txrxWaveAppend=1, const eWaveGenCmd *inCmdWave=NULL, uint8_t *anaBuf=NULL, int anaBufize=0, int n_sample=0, int sample_time_us=0, uint8_t *txBits=NULL, int nTxBits=0, int devices=0x1, int masters=0x1);
    long WaveDecodeData(int n, const int cmdId=CMD_SEND_RECEIVE_A, const int statusCfg=2, eWaveStruct_t *rxWave=NULL, int rxWaveSize=0, eSensorData *rxData=NULL, int rxByteSize=TDK_USSM_DATA_SIZE, int maxEchos=TDK_USSM_DEFAULT_MAX_ECHOS, uint32_t dmaMask=0ul);
    long WaveDecodeBytes(int n, eWaveStruct_t *rxWave=NULL, int rxWaveSize=0, int stopPos=0, eSensorData *rxData=NULL, int rxByteSize=TDK_USSM_DATA_SIZE);


//-- ADVANCED TOF MEASUREMENT 
    long CalPulse(int deviceMask) ;

    long SendA(int deviceMask) ;
    long SendB(int deviceMask) ;
    long SendC(int deviceMask) ;

    long ReceiveA(int deviceMask) ;
    long ReceiveB(int deviceMask) ;
    long ReceiveC(int deviceMask) ;

    long SendReceiveA(int deviceMask, int masterMask) ;
    long SendReceiveB(int deviceMask, int masterMask) ;
    long SendReceiveC(int deviceMask, int masterMask) ;

    long EnvelopSendA(int deviceMask, uint8_t *pBuf=NULL, int bufSize=0, int nSample=512, int sampleTimeUs=10, int rightShift=0);
    long EnvelopReceiveA(int deviceMask, uint8_t *pBuf=NULL, int bufSize=0, int nSample=512, int sampleTimeUs=10, int rightShift=0);
    long EnvelopSendReceiveA(int deviceMask, int masterMask, uint8_t *pBuf=NULL, int bufSize=0, int nSample=512, int sampleTimeUs=10, int rightShift=0);

    long JtagSendA(int deviceMask, eWaveStruct_t *rxWave=NULL, int rxWaveSize=0, uint8_t *pBuf=NULL, int bufSize=0, int nSample=512, int sampleTimeUs=20)  ;
    long JtagReceiveA(int deviceMask, eWaveStruct_t *rxWave=NULL, int rxWaveSize=0, uint8_t *pBuf=NULL, int bufSize=0, int nSample=512, int sampleTimeUs=20)  ;
    long JtagSendReceiveA(int deviceMask, int masterMask, eWaveStruct_t *rxWave=NULL, int rxWaveSize=0,  uint8_t *pBuf=NULL, int bufSize=0, int nSample=512, int sampleTimeUs=20)  ;
    long JtagEnvelopThreshold(int deviceMask, int masterMask, eWaveStruct_t *rxWave=NULL, int rxWaveSize=0, uint8_t *pBuf=NULL, int bufSize=0, int nSample=512, int sampleTimeUs=20)  ;
    long JtagThreshold(int deviceMask, eWaveStruct_t *rxWave=NULL, int rxWaveSize=0, uint8_t *pBuf=NULL, int bufSize=0, int nSample=512, int sampleTimeUs=20)  ;



//-- CONVERSION FUNCTIONS TOF TO DISTANCE
/**
 * \brief TOF us conversion into distance in mm.
 * Conversion helper function to convert time of flight in us to distance in mm
 * \param microseconds => ToF in us
 * \return Converted distance in mm
 */ 
    long MicrosecondsToMillimeters(long microseconds) const; // Time of Flight to Millimeters 

/**
 * \brief TOF us conversion into distance in inches.
 * Conversion helper function to convert time of flight in us to distance in inches
 * \param microseconds => ToF in us
 * \return Converted distance in inch
 */ 
    long MicrosecondsToInches(long microseconds) const; // Time of Flight to Inches 

/**
 * \brief TOF us conversion into distance in cm.
 * Conversion helper function to convert time of flight in us to distance in cm
 * \param microseconds => ToF in us
 * \return Converted distance in cm
 */ 
    long MicrosecondsToCentimeters(long microseconds) const; // Time of Flight to Centimeters 



//-- REGISTERS READ/WRITE
/**
 * \brief Register Read/Write function
 * Method for reading/writing register data
 * \param TO DO
 * \return TO DO
 */ 
    long RegisterReadWrite (const int txPin, const int rxPin, const int cmdId, uint8_t *rxBytes, int nRxBits,uint8_t *txBytes=NULL, int nTxBits=0);   

/**
 * \brief Register Read function
 * Method for reading register data
 * \param TO DO
 * \return Total number of read bits from sensor n
 */ 
    long ReadRegister   (const int cmdId, uint8_t *rxBytes, int nRxBits, const int n=0, eSensorRegisters *sensorRegisters=NULL) ; // returns total number of read bits of sensor n
    
/**
 * \brief Register Write function
 * Method for writing register data
 * \param  TO DO
 * \return Total number of written bits to sensor n
 */ 
    long WriteRegister  (const int cmdId, uint8_t *txBytes, int nTxBits, const int n=0, eSensorRegisters *sensorRegisters=NULL) ; // returns total number of written bits to sensor n
    int  SaveRegisterData(eSensorRegisters *pRegs, const int cmdId, uint8_t *pBytes) ;


/**
 * \brief Register field unique ID check
 * Parses and checks unique ID fields, splits ID into relevant parts and writes using passed pointers to variables
 * \param fieldId => Unique field ID
 * \param rid => Pointer to placeholder for Register ID
 * \param eid => Pointer to placeholder for Parameter ID
 * \return 0 or 1, depending whether the check was successful
 */ 
    int parseFieldId (int fieldId, int* rid, int* eid);

/**
 * \brief Get register field values based on their unique IDs
 * Getter method which returns the set value of a given register field
 * \param fieldId => Register field unique ID
 * \param n => Target sensor
 * \return Lookup table index value for set field register value
 */ 
    long RegisterFieldGet (int fieldId, int n);  

/**
 * \brief Set register field values based on their unique IDs
 * Setter method which sets the value of a given register field
 * \param fieldId => Register field unique ID
 * \param n => Target sensor
 * \param newVal => New value to be set (according to relevant lookup table)
 * \return Number of written bits
 */ 
    long RegisterFieldSet (int fieldId, int n, uint32_t newVal);

/**
 * \brief Get register field real values based on their unique IDs
 * Getter method which returns the REAL set value of a given register field
 * \param fieldId => Register field unique ID
 * \param n => Target sensor
 * \return Real value of target register field
 */ 
    float RegisterFieldGetRealValue (int fieldId, int n);   

/**
 * \brief Set register field real values based on their unique IDs
 * Setter method which sets the value of a given register field using a real value
 * \param fieldId => Register field unique ID
 * \param n => Target sensor
 * \param newVal => New real value to be set
 * \return Number of written bits
 */ 
    long RegisterFieldSetRealValue (int fieldId, int n, float newVal);

/**
 * \brief Write Calibration Register
 * Setter method which writes a new calibration register
 * \param txBytes => Register bytes to be written (pointer)
 * \param n => Target sensor
 * \return Number of written bits
 */
    long CalibWrite     (uint8_t *txBytes,const int n=0) ;

/**
 * \brief Read Calibration Register
 * Getter method which reads the current bytes of the calibration register for the target sensor
 * \param rxBytes => Placeholder variable to be filled with read register bytes (pointer)
 * \param n => Target sensor
 * \return Number of read bits
 */ 
    long CalibRead      (uint8_t *rxBytes,const int n=0) ;

/**
 * \brief Write Measurement Register
 * Setter method which writes a new measurement register
 * \param txBytes => Register bytes to be written (pointer)
 * \param n => Target sensor
 * \return Number of written bits
 */ 
    long MeasWrite      (uint8_t *txBytes,const int n=0) ;

/**
 * \brief Read Measurement Register
 * Getter method which reads the current bytes of the measurement register for the target sensor
 * \param rxBytes => Placeholder variable to be filled with read register bytes (pointer)
 * \param n => Target sensor
 * \return Number of read bits
 */ 
    long MeasRead       (uint8_t *rxBytes,const int n=0) ;


/**
 * \brief Write THRES_A Register
 * Setter method which writes a new THRES_A register
 * \param txBytes => Register bytes to be written (pointer)
 * \param n => Target sensor
 * \return Number of written bits
 */ 
    long ThresAWrite    (uint8_t *txBytes,const int n=0) ;

/**
 * \brief Read THRES_A Register
 * Getter method which reads the current bytes of the THRES_A register for the target sensor
 * \param rxBytes => Placeholder variable to be filled with read register bytes (pointer)
 * \param n => Target sensor
 * \return Number of read bits
 */ 
    long ThresARead     (uint8_t *rxBytes,const int n=0) ;


/**
 * \brief Write THRES_B Register
 * Setter method which writes a new THRES_B register
 * \param txBytes => Register bytes to be written (pointer)
 * \param n => Target sensor
 * \return Number of written bits
 */ 
    long ThresBWrite    (uint8_t *txBytes,const int n=0) ;

/**
 * \brief Read THRES_B Register
 * Getter method which reads the current bytes of the THRES_B register for the target sensor
 * \param rxBytes => Placeholder variable to be filled with read register bytes
 * \param n => Target sensor
 * \return Number of read bits
 */ 
    long ThresBRead     (uint8_t *rxBytes,const int n=0) ;  
    
    long ReadId         (uint8_t *rxBytes,const int n=0) ;      
    long ReadStatus     (uint8_t *rxBytes,const int n=0) ;    
    long ReadTemp       (uint8_t *rxBytes,const int n=0) ;  
    long ReadNfdStatus  (uint8_t *rxBytes,const int n=0) ;  
    long ReadEeprom     (uint8_t *rxBytes,const int n=0) ; 

 /**
 * \brief Put sensor in standby mode
 * Triggers the standby mode for a given sensor
 * \param n => Target device
 * \return TO DO
 */
    long Standby        (const int n=0) ;

 /**
 * \brief Wake up sensor
 * Triggers a wakeup call for a given sensor
 * \param n => Target device
 * \return TO DO
 */
    long Wakeup         (const int n=0) ; 

    long EepromProg     (const int progPin, const int n=0, const int progON=TdkUssm_HIGH, const int prgOFF=TdkUssm_LOW) ; 
     
     

//-- HELPER FUNCTIONS
    eIOState  GetIoState(eWaveStruct_t *pwave, uint32_t pos, uint8_t bit_id) const;

 /**
 * \brief Get Bits
 * Getter method acting as a helper function for bit acquisition
 * \param buf => Placeholder variable for storing the queried value
 * \param bitId => bit ID
 * \return TO DO
 */
    int       GetBit(uint8_t *buf, uint32_t bit_id) const;
    int       SetBit(uint8_t *buf, uint32_t bit_id, int val) const;
    uint32_t  GetBitsU32(uint8_t *src, uint32_t src_lsb, uint32_t nbit) const;
    uint32_t  SetBitsU32(uint8_t *dest, uint32_t dest_lsb, uint32_t nbit, uint32_t val32) const;
    int       GetBitRange(uint8_t *dest, uint32_t dest_lsb, uint8_t *src, uint32_t src_lsb, uint32_t nbit) const;
    uint32_t  GetBitsNbyte(uint32_t nbits) const;
    int       GetEvenParity(uint8_t *pBuf, uint32_t lsb, uint32_t msb) const;
 
 /**
 * \brief Determine lookup table value of a real value
 * Getter (helper) method which returns the lookup table index value of a real value for a specific register field, based on the passed lookup table
 * \param pLut => Lookup table custom structure (pointer)
 * \param pVal => Lookup table index value (pointer)
 * \param real_val => Queried real value
 * \return Boolean which shows whether the real value exists in lookup table
 */ 
    bool      GetValOfRealVal(const eLUTStruct *pLut, uint32_t *pVal, float real_val) const;

 /**
 * \brief Determine real value of an index lookup table value
 * Getter (helper) method which returns the real value of an index value for a specific register field, based on the passed lookup table
 * \param pLut => Lookup table custom structure (pointer)
 * \param pRealVal => Real value to be converted (pointer)
 * \param val => Placeholder variable to be filled with the lookup table index value
 * \return Boolean which shows whether the real value exists in lookup table
 */
    bool      GetRealValOfVal(const eLUTStruct *pLut, float *pRealVal, uint32_t val, const char *pDesc) const;

    bool      IsSendReceiveCommand(const int cmd_id) const;
    bool      IsSendReceiveA(const int cmd_id) const;
    bool      IsValidCommand(int cmd_id) const;

 /**
 * \brief Device-to-Device Mask conversion
 * Assigns a device mask to the inputted device
 * \param n => Target device
 * \return TO DO
 */
    inline int DeviceToMask(int n); // Easy conversion from single device to Device Mask


//-- PUBLIC RUNTIME VARIABLES
    eSimpleCmdStruct    cmd;
    eWaveStruct_t       waveBuf[TDK_USSM_WAVE_BUF_SIZE];
    const int           waveBufSize = TDK_USSM_WAVE_BUF_SIZE;
    eSensorData         dataBuf;
    int                 _rx_buf_ptr;
    int                 _tx_buf_ptr;
    
    int                 TimeAdjustUs; 
 
    /************************
     * Private Section
     * */
    private:
    
    //-- Sensors Pins and count
    int *txPins;                    // txPin list
    int *rxPins;                    // rxPin list
    int *anaPins;                   // anaPins list
    int nSensors;                   // Number of sensors
    
    //-- Signalling Mode
    int  IO_DRIVE_LOW;
    int  IO_RELEASE;
    int  RX_TBIT_LOW_US;
    int  IO_IN_MODE;
    
 private:
    //-- HAL Variables
    uint32_t            _hal_timer_freq_hz;
};
