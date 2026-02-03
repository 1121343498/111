/*************************************************
 *  (C) TDK Electronics GmbH & Co
 *  All rights reerved
 *  
 *  Title     : TDK_USSM_Ultrasonic_Sensor
 *  Release   : V1.0
 *  Version   : 2023-08-07 
 *  Descrption: 
 *    TDK Ultrasonic Sensor Defines
 *
 *  -------- ABSOLUTE NO WARRENTY OF ANY KIND -------
 *
 *   THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, 
 *   EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
 *   OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE. MICROSOFT CORPORATION
 *   SHALL NOT BE LIABLE FOR ANY TECHNICAL OR EDITORIAL ERRORS OR OMISSIONS CONTAINED HEREIN. 
 * 
 *************************************************/

#ifndef _TDK_USSM_DEFS_H
#define _TDK_USSM_DEFS_H

#include "stdint.h"


/* ----------------------- Defines ------------------------------------------*/

/*! \ingroup tdk_ussm
 * \brief Default buffers config
 */

//-- Class Buffer configuration
#if (defined ARDUINO_AVR_NANO) || (defined ARDUINO_AVR_UNO)    
    #define TDK_USSM_WAVE_BUF_SIZE  32  // For Small SRAM MCUS this will offer reduced functionality
#else
    #define TDK_USSM_WAVE_BUF_SIZE  256 /*512 Recommended !*/ // For MCUS with enough SRAM just use as much as needed
#endif      


//-- Other static configurations
#define WAVEGEN_SND_OFFSET              16
#define WAVEGEN_RCV_OFFSET              0
#define WAVEGEN_BUS_MASK                0xfffful

#define TDK_USSM_DEVICE_DEFAULT         524 // E524 Family
#define TDK_USSM_DATA_SIZE              32  // 32 bytes is the max data size for register (eerporm 28 bytes)

#ifndef TDK_USSM_MAX_DEVICES
#define TDK_USSM_MAX_DEVICES            16 // 8
#endif

//-- Advanced Configuration for MCUS with Cortex-M MCUs, DMA, SPI ...etc.
#define TDK_USSM_BUFFER_SIZE            256 * TDK_USSM_MAX_DEVICES
#define TDK_USSM_DMA_BUF_MAX_SIZE       512                                                        // 0.5 K of 32Bits = 2K per channel
#define TDK_USSM_ADC_BUF_MAX_SIZE       (TDK_USSM_DMA_BUF_MAX_SIZE * 4)                            // For ADC we use 8 bit
#define TDK_USSM_ADC_BUF_TOTAL_SIZE     ((TDK_USSM_MAX_DEVICES * TDK_USSM_ADC_BUF_MAX_SIZE) / 2) // We use Half of it for ADC as the buffer is shared
#define TDK_USSM_SEQ_MAX_LENGTH         32
#define TDK_USSM_JTAG_BUF_MAX_SIZE      1024    // 16bit words = 2KByte per JTAG port

#define TDK_USSM_DEFAULT_MAX_ECHOS      4       // Default Max Echos

//-- CLI Configuration
#define DEFAULT_REQUEST_TERMINATOR      ";"
#define DEFAULT_RESPONSE_TERMINATOR     "#\r\n"

#define TDK_USSM_N_CMD                  55


/* ----------------------- Type definitions ---------------------------------*/

/*! \ingroup tdk_ussm
 * \brief tdk_ussm Protocol Times in uS.
 *
 * tdk_ussm Protocol Times defines all applicable Times Txx for tdk_ussm protocol in us unit . 
 */
typedef enum
{
    TNONE       = 0ul,     /*!< No pulse. */
    TDEB        = 20ul,    /*!< Input debouncer (max value) . */
    TD          = 40ul,    // 50ul,                 /*!< IO high phase after TCMD. */
    TSND        = 100ul,   /*!< IO low phase for send request. */
    TREC        = 150ul,   /*!< IO low phase for receive request. */
    TMEAS       = 200ul,   /*!< IO low phase beginning of measurement command. */
    TCMD        = 250ul,   /*!< IO low phase to enter command mode. */
    TVPROG      = 5000ul,  /*!< Setup time for VPROG. */
    TPROG       = 25000ul, /*!< Programming time. */
    TBIT        = 140ul,   //150ul,                 /*!< Bit length. */
    TBIT0_CMP   = 80ul,    /* Threshold used to detect Bit = 0*/
    TBIT_MAX    = 180ul,   /*!< Bit length. */
    TBIT0       = 1000ul,  // TODO 100ul              /*!< IO low phase for a logical '0'. */
    TBIT1       = 50ul,    // TODO 50ul              /*!<IO low phase for a logical '1', . */
    TBIT0_LO    = 100ul,   // TODO 100ul,                 /*!< IO low phase for a logical '0'. */
    TBIT1_LO    = 50ul,    // 50ul,                 /*!<IO low phase for a logical '1', . */
    TBIT0_HI    = 40ul,    // 50ul,                 /*!< IO low phase for a logical '0'. */
    TBIT1_HI    = 90ul,   // 100ul
    TBIT_PHASE  = 40ul,     // TODO 50ul              /*!<IO low phase for a logical '1', . */
    TECHO_MIN   = 1000ul,   // TODO: Double check this. Minimum echo distance = 2mm ???
    TIMEOUT     = 18000ul
} eTimeDly;

typedef enum
{
    //-- Low-level direct ASIC Commands
    CMD_STANDBY             = 0b011101,
    CMD_WAKE_UP             = 0b011110,
    CMD_READ_ID             = 0b001110,
    CMD_EE_READ             = 0b011011,
    CMD_EE_COPY             = 0b011000,
    CMD_CALIB_READ          = 0b010111,
    CMD_CALIB_WRITE         = 0b010100,
    CMD_ENV_SEND_A          = 0b010001,
    CMD_ENV_RECEIVE_A       = 0b010010,
    CMD_READ_NFD_STATUS     = 0b010101,
    CMD_READ_TEMP           = 0b001111,
    CMD_CAL_PULSE           = 0b000001,
    CMD_READ_STATUS         = 0b001100,
    CMD_MEAS_READ           = 0b001010,
    CMD_MEAS_WRITE          = 0b001001,
    CMD_THRES_B_READ        = 0b000110,
    CMD_THRES_B_WRITE       = 0b000101,
    CMD_THRES_A_READ        = 0b000011,
    CMD_THRES_A_WRITE       = 0b000000,

    //-- High-level Software commands: 0b1xx_xxxx
    CMD_SEND_A              = 0b100000,
    CMD_RECEIVE_A           = 0b100001,
    CMD_SEND_B              = 0b110010,
    CMD_RECEIVE_B           = 0b110000,
    CMD_SEND_C              = 0b110011,
    CMD_RECEIVE_C           = 0b110001,


    CMD_SEND_RECEIVE_A      = 0b1000000,
    CMD_SEND_RECEIVE_B      = 0b1000001,
    CMD_SEND_RECEIVE_C      = 0b1000010,
    CMD_ENV_SEND_RECEIVE    = 0b1000011,
    
    CMD_READ_ALL            = 0b1000100,
    CMD_POLL                = 0b1000101,
    CMD_STREAMOUT           = 0b1000110,
    CMD_END                 = 0b1000111,

    CMD_JTAG_SEND_A         = 0b1001100,
    CMD_JTAG_RECEIVE_A      = 0b1001101,
    CMD_JTAG_SEND_RECEIVE   = 0b1001110,
    CMD_JTAG_THRESH_READ    = 0b1001111,
    CMD_JTAG_ENVTHRES_READ  = 0b1010000,
    CMD_SEND_RECEIVE_AH     = 0b1010001,    

    CMD_SET                 = 0b1100000,
    CMD_GET                 = 0b1100001,
    CMD_RX_WAVE             = 0b1100010,

    CMD_CAL_OSCILLATOR      = 0b1100011,
    CMD_CAL_FDRV            = 0b1100100,
    CMD_CAL_GAIN            = 0b1100101,

    CMD_COM_BAUDRATE        = 0b1101000,
    CMD_RESET               = 0b1101001,    
    CMD_UPDATE_FIRMWARE     = 0b1101010,
    CMD_HARDWARE_ID         = 0b1101011,
    CMD_DIAGNOSIS           = 0b1101100,

    CMD_HELP                = 0b1110000,
    CMD_DEBUG               = 0b1110001,
    CMD_SOFT_PARAM_READ     = 0b1110010,
    CMD_SOFT_PARAM_WRITE    = 0b1110011,
    CMD_NOP                 = 0b1111111,

    //-- Invalid command
    CMD_INVALID             = 0b11111111 // xff

} eCmdId;


typedef enum
{
    // Arduino General Purpose Us Sensors
    HC_SR04         = 04,
    JSN_SR04T_V3    = 05,
    // ELMOS ASIC EL524.33 - IO Sensors
    EL524_IO        = 33,
    // ELMOS ASIC EL524.36 LIN/SPI Sensors
    EL524_LIN       = 36
} eTdkUssmType;

typedef enum
{
    ELM_REQ_NONE = 0,      // Default = Idle
    ELM_REQ_CMD = 1,       // Single Command Request
    ELM_REQ_POLL = 2,      // Poll on single command
    ELM_REQ_SEQUENCE = 3,  // Sequence of commands and functions
    ELM_REQ_RESPONSE = 4,  // Sending response
    ELM_REQ_STREAMOUT = 5, // Streamout mode
} eTdkUssmReqMode;

typedef enum
{
    ELM_PROC_NONE = 0,  // Default = Idle
    ELM_PROC_SET_PARAM, // Single Command Request
    ELM_PROC_GET_PARAM, // Poll on single command
    ELM_PROC_CAL_FDRV,  // Sequence of commands and functions
    ELM_PROC_CAL_VDRV,  // Sequence of commands and functions
    ELM_PROC_CAL_GAIN,  // Sequence of commands and functions
    ELM_PROC_CAL_AUTO   // Auto Calibration FDRV, VDRV, GAIN
} eTdkUssmProcFunction;

typedef enum
{
    ELM_JTAG_OFF = 0x0000000,
    ELM_JTAG_READ_ENV = 0xD0,     // Read Envelop data
    ELM_JTAG_READ_THRES = 0xD1,   // Read Threshold data
    ELM_JTAG_READ_ENVTHRES = 0xD2 // Read Envelop and Threshold  data
} eTdkUssmJTAGMode;

const uint16_t ELM_JTAG_TMS_RESET = (uint16_t)0b0001111100000000;
const uint16_t ELM_JTAG_TMS_IR_SET = (uint16_t)0b0110000000001100;
const uint16_t ELM_JTAG_TMS_SHIFT_DR10 = (uint16_t)0b0100000000000110;
const uint16_t ELM_JTAG_TMS_SHIFT_DR20_MSB = (uint16_t)0b0000000100000000;
const uint16_t ELM_JTAG_TMS_SHIFT_DR20_LSB = (uint16_t)0b0000000000000110;

const uint16_t ELM_JTAG_TDO_DEFAULT = (uint16_t)0x0000;
const uint16_t ELM_JTAG_TDO_IR_ENV = (uint16_t)0b0000000001011000;
const uint16_t ELM_JTAG_TDO_IR_THRES = (uint16_t)0b0000010001011000;
const uint16_t ELM_JTAG_TDO_IR_ENVTHRES = (uint16_t)0b0000001001011000;

/*! \ingroup tdk_ussm
 * \brief Errorcodes used by all function in the protocol stack.
 */
typedef enum
{
    ELM_MODE_POLLING = 0, // Default
    ELM_MODE_EVENTS       // Event based
} eTdkUssmMode;

/*! \ingroup tdk_ussm
 * \brief Errorcodes used by all function in the protocol stack.
 */
typedef enum
{
    DBG_NONE = 0,
    DBG_TEXT_OFF,
    DBG_ECHO_CMD,
    DBG_ERROR
} eDebugIds;


/*! \ingroup tdk_ussm
 * \brief Errorcodes used by all function in the protocol stack.
 */
typedef enum
{
    ELM_OK,
    ELM_DONE,
    ELM_EINVAL,
    ELM_BUSY,
    ELM_OVERFLOW,
    ELM_EPORTERR,
    ELM_ENORESP,
    ELM_EIO,
    ELM_INACTIVE,
    ELM_EILLSTATE,
    ELM_ETIMEDOUT,
    ELM_EPARITY,
    ELM_INVALID_CMD,
    ELM_FEW_CMD_DATA,
    ELM_INVALID_PARAM,
    ELM_ERROR
} eErrorCode;

/*! \ingroup tdk_ussm
 * \brief Errorcodes used by all function in the protocol stack.
 */
typedef enum
{
    IO_VAL_0 = 0,
    IO_VAL_1 = 1,
    IO_OUT_0,
    IO_OUT_1,
    IO_OUT_BIT,
    IO_OUT_BUS,
    IO_OUT_PULSE,
    IO_OUT_VAL,
    IO_OUT_PROG,
    IO_OUT_ANA,
    IO_OUT_OD,
    IO_OUT,
    IO_WRITE,
    IO_IN_PU,
    IO_IN_PD,
    IO_IN_0,
    IO_IN_1,
    IO_IN_BIT,
    IO_IN_VAL,
    IO_IN_NEGEDGE,
    IO_IN_POSEDGE,
    IO_IN_BUS_EDGE,
    IO_IN_EDGE,
    IO_IN_ANA,
    IO_IN_TIME,
    IO_IN,
    IO_READ,
    IO_GET_BITS,
    IO_IDLE,
    IO_INIT,
    IO_NOP,
    IO_WAIT,
    IO_DONE,
    IO_READY,
    IO_ERROR
} eIOState;


typedef enum
{
    PARITY_EVEN = 0,
    PARITY_ODD = 1,
    PARITY_NONE = 2
} eParityType;

typedef enum
{
    STATUS_CFG_NONE  = 0,
    STATUS_CFG_6BIT  = 1,
    STATUS_CFG_12BIT = 2,
    STATUS_CFG_18BIT = 3
} eStatusCfgType;  


/*! \ingroup tdk_ussm
 * \brief Errorcodes used by all function in the protocol stack.
 */
typedef struct
{
    eIOState cmd; // IO Sta   id;te
    uint32_t val; // Bus val
    uint32_t arg; // delay_us, bit_val, n_bit ...etc
} eWaveGenCmd;


typedef struct {
  uint16_t  val;
  uint32_t  time_us;
} eWaveStruct_t;

typedef struct
{
    char cmd[4];
    eTimeDly pulse_us;
    unsigned char id;
    unsigned char id_bits;
    unsigned char rx_bits;
    unsigned char tx_bits;
    unsigned char analog;
    const eWaveGenCmd *pwave; // Wave for SND/REC
    const char *name;
    const char *usage;
} eTdkUssmCmdStruct;


typedef struct 
{
    int             id;
    uint32_t        devices;
    uint32_t        masters;
} eSimpleCmdStruct;


typedef struct 
{
    eTdkUssmReqMode     state;
    uint8_t             args[64]; // Command and its args
    char                cmd_str[64];
    int                 n_arg;
    uint32_t            devices;
    uint32_t            masters;
    uint32_t            dma_flags;  // When set to 1 means DMA was used during measurements
    int                 n_sample;
    uint32_t            sample_time_us;
    int                 iterations;
    int                 cntr;
    uint32_t            delay_ms;
    uint32_t            seq_devices;
    const char              **pseq;
    const eTdkUssmCmdStruct *pdef;
} eTdkUssmCmdExecStruct;


typedef struct
{
    char     cmd_str[128]; // Command and its args
    uint32_t val;
    int      iterations;
    uint32_t delay_ms;
    uint32_t cntr;
} eTdkUssmPolStruct;

typedef struct
{
    uint32_t cntr;                                            // step counter
    uint32_t n_step;                                          // Number of Steps
    char cmd_id[TDK_USSM_SEQ_MAX_LENGTH];                     // Command and its args
    eTdkUssmProcFunction proc_func[TDK_USSM_SEQ_MAX_LENGTH]; // Function pointer
} eTdkUssmSeqStruct;


typedef struct
{
    uint32_t buf[256];  // Data Buffer
    int      len;
    int      bits; // number of valid bits in each data
} eTdkUssmDataStruct;


/***************************
* Sensor Data Structures
*/

/*! \ingroup tdk_ussm
 * \brief Threshold Element.
 */
#define  LUT_END   -1  // Marker for LUT end
 
typedef struct
{
    int   val;  // Entry val
    float real_val;        // Real Value
    const char *desc; // Delta time to initial threshold position in addition to 1ms offset
} eLUTStruct;


/*! \ingroup tdk_ussm
 * \brief Measurement Data Structure.
 */
typedef struct
{
    uint8_t id;
    uint8_t lsb;
    uint8_t nbit;
    uint32_t val;
    const char *name;
    const char *description;
    const eLUTStruct *lut;
} eParameterElement;


/*! \ingroup tdk_ussm
 * \brief Set Get Struct .
 */
typedef struct
{
    int id ;            // eParameterTableID id;
    const eParameterElement *ptable;
    const char *pname;
    uint32_t size;
    unsigned char readCmdId;
    unsigned char writeCmdId;
    int regNBit;
} eParamaterTable;

/***************************
* Main Sensor Strucure
*/
typedef struct
{
    uint8_t     id[4];          // 24 bits
    uint8_t     thres_a[11];    // 86 bits // Including 6 bit parity
    uint8_t     thres_b[11];    // 86 bits // Including 6 bit parity
    uint8_t     meas[10];       // 80 bits
    uint8_t     status[3];      // 17 bits
    uint8_t     temp[4];        // 16 bits signed
    float       temp_val;         // Temperature value
    uint8_t     nfd_status[4];  // 24 bits
    uint16_t    envelop[32];   // analog values
    uint8_t     calib[10];      // 74 bits + 6 bits CRC
    uint8_t     ee_data[10];    // 74 bits
    uint8_t     standby;        //  1 bit for standby mode

    //--- Set/Get Temporary values -----
    uint32_t    param_val;
    float       param_real_val;

} eSensorRegisters;


typedef struct
{
    //-- Raw Register bytes& Status bits buffer
    uint8_t     rx_data[TDK_USSM_DATA_SIZE];
    uint32_t    rx_data_len;
    uint8_t     rx_data_nbyte;
    
    //--- Cal Pulses
    uint32_t    cal_pulse[4];   //  4 edges
    uint32_t    tcal_us;        // CAL_PULSE TCal in us
    uint32_t    freq_hz;        // CAL_PULSE Measured Frequency in Hz,
    
    //-- Distance and Time of flight
    uint8_t     echo_status[3];     // Measurement status bits
    int         t_offset_us;    // time delay offset in us
    int         d_offset_mm;    // distance physical offset in mm
    uint8_t     n_echos;        // Number of Valid Echos
    uint8_t     n_echo_heights;
    int         tn_echos[TDK_USSM_DEFAULT_MAX_ECHOS];       // T to echos (negedge to negedge)
    int         len_echos[TDK_USSM_DEFAULT_MAX_ECHOS];      // Length of echos in us (low duration)
    uint8_t     hight_echos[TDK_USSM_DEFAULT_MAX_ECHOS];    // Echo hight
    int         distances[TDK_USSM_DEFAULT_MAX_ECHOS];      // Distance in mm D = T*c/2
} eSensorData;

#endif //_TDK_USSM_DEFS_H

/************************ (C) COPYRIGHT TDK Electronics *****END OF FILE****/
