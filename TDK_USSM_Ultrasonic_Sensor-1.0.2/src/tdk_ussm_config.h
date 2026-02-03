/*************************************************
 *  (C) TDK Electronics GmbH & Co
 *  All rights reerved
 *  
 *  Title     : TDK_USSM_Ultrasonic_Sensor
 *  Release   : V1.0
 *  Version   : 2023-08-07 
 *  Descrption: 
 *    TDK Ultrasonic Sensor Driver defines
 *
 *  -------- ABSOLUTE NO WARRENTY OF ANY KIND -------
 *
 *   THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, 
 *   EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
 *   OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE. MICROSOFT CORPORATION
 *   SHALL NOT BE LIABLE FOR ANY TECHNICAL OR EDITORIAL ERRORS OR OMISSIONS CONTAINED HEREIN. 
 * 
 *************************************************/

#ifndef _TDK_USSM_CONFIG_H
#define _TDK_USSM_CONFIG_H


#include "stdint.h"
#include "tdk_ussm_defs.h"


/* ----------------------- Defines ------------------------------------------*/

/* ----------------------- Type definitions ---------------------------------*/



/* ----------------------- Constants -----------------------------------------*/
 
/****/
/*! \ingroup tdk_ussm
 * \brief Commands constants
 */

/****/
const eWaveGenCmd  WAV_SEND_A[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TSND      },
    { IO_OUT_1    , 0xffff , 0         }, // IO Release
    { IO_INIT     , IO_IN_PU, 0        }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff , 40000ul   }, // 40 ms
    { IO_DONE     , 0xffff , 0         }
};

const eWaveGenCmd  WAV_RECEIVE_A[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TREC      },
    { IO_OUT_1    , 0xffff , 0         }, // IO Release
    { IO_INIT     , IO_IN_PU, 0        }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff , 40000ul   }, // 40 ms
    { IO_DONE     , 0xffff , 0         }
};

//-- SND on bits [31:16], REC on bits [15:0]
const eWaveGenCmd  WAV_SEND_RECEIVE_A[] PROGMEM = {
    { IO_OUT_BUS  , 0xffff0000  ,(TREC-TSND)    }, // Send (Master) is bit 0, all others are slaves
    { IO_OUT_BUS  , 0x00000000  , TSND          },
    { IO_OUT_1    , 0xffff      , 0             }, // IO Release
    { IO_INIT     , IO_IN_PU    , 0             }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff      ,40000ul        }, // 40 ms
    { IO_DONE     , 0xffff      ,0              }
};


//-- SEND_RECEIVE_AHybrid: SND on bits [31:16], REC on bits [15:0]
const eWaveGenCmd  WAV_SEND_RECEIVE_AH[] PROGMEM = {
    { IO_OUT_BUS  , 0xffff0000  ,(TREC-TSND)    }, // Send (Master) is bit 0, all others are slaves
    { IO_OUT_BUS  , 0x00000000  ,(TSND-10)      }, // JSN_SR04T_V3 : requires High pulse for 20us
    { IO_OUT_BUS  , 0x00000300  ,10             }, // HC_SR04 : requires High pulse for 10 us
    { IO_OUT_BUS  , 0x00ff00ff  ,0              }, // Release all Sensors
    { IO_INIT     , IO_IN_PU    , 0             }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff      ,40000ul        }, // 40 ms
    { IO_DONE     , 0xffff      ,0              }
};


/****/
const eWaveGenCmd  WAV_SEND_B[] PROGMEM = {
    { IO_OUT_0   , 0x0000 , TMEAS       },
    { IO_OUT_1   , 0xffff , TD          },
    { IO_OUT_BIT , 0xffff , 1           },
    { IO_OUT_BIT , 0x0000 , 0           },
    { IO_OUT_1   , 0xffff , 0           }, // IO Release
    { IO_INIT    , IO_IN_PU , 0         }, // Change IO Mode to Input
    { IO_IN_EDGE , 0xffff , 40000ul     }, // 40 ms
    { IO_DONE    , 0xffff , 0           }
};

const eWaveGenCmd  WAV_RECEIVE_B[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TMEAS       },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_1    , 0xffff , 0           }, // IO Release
    { IO_INIT     , IO_IN_PU, 0          }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff , 40000ul     }, // 40 ms
    { IO_DONE     , 0xffff , 0           }
};

//-- SND on bits [31:16], REC on bits [15:0]
const eWaveGenCmd  WAV_SEND_RECEIVE_B[] PROGMEM = {
    { IO_OUT_BUS  , 0x00000000 , TMEAS       },
    { IO_OUT_BUS  , 0xffffffff , TD          },
    { IO_OUT_BUS  , 0x00000000 , TBIT_PHASE  }, // start BIT0
    { IO_OUT_BUS  , 0xffff0000 , TBIT_PHASE  }, // SND = 1, REC = 0
    { IO_OUT_BUS  , 0xffffffff , TBIT_PHASE  }, // end BIT 0
    { IO_OUT_BUS  , 0x00000000 , TBIT_PHASE  }, // start BIT1
    { IO_OUT_BUS  , 0x00000000 , TBIT_PHASE  }, // SND = 0, REC = 0
    { IO_OUT_BUS  , 0xffffffff , TBIT_PHASE  }, // end BIT 1
    { IO_OUT_1    , 0xffff     , 0           }, // IO Release
    { IO_INIT     , IO_IN_PU   , 0           }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff     , 40000       }, // 40 ms
    { IO_DONE     , 0xffff     , 0           }
};

/****/
const eWaveGenCmd  WAV_SEND_C[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TMEAS       },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_1    , 0xffff , 0           }, // IO Release
    { IO_INIT     , IO_IN_PU , 0         }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff , 40000ul     },
    { IO_DONE     , 0xffff , 0           }
};

const eWaveGenCmd  WAV_RECEIVE_C[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TMEAS       },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_1    , 0xffff , 0           }, // IO Release
    { IO_INIT     , IO_IN_PU , 0         }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff , 40000ul     },
    { IO_DONE     , 0xffff , 0           }
};

//-- SND on bits [31:16], REC on bits [15:0]
const eWaveGenCmd  WAV_SEND_RECEIVE_C[] PROGMEM = {
    { IO_OUT_BUS  , 0x00000000 , TMEAS       },
    { IO_OUT_BUS  , 0xffffffff , TD          },
    { IO_OUT_BUS  , 0x00000000 , TBIT_PHASE  }, // start BIT0
    { IO_OUT_BUS  , 0xffff0000 , TBIT_PHASE  }, // SND = 1, REC = 0
    { IO_OUT_BUS  , 0xffffffff , TBIT_PHASE  }, // end BIT 0
    { IO_OUT_BUS  , 0x00000000 , TBIT_PHASE  }, // start BIT1
    { IO_OUT_BUS  , 0xffffffff , TBIT_PHASE  }, // SND = 1, REC = 1
    { IO_OUT_BUS  , 0xffffffff , TBIT_PHASE  }, // end BIT 1
    { IO_OUT_1    , 0xffff     , 0           }, // IO Release
    { IO_INIT     , IO_IN_PU   , 0           }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff     , 40000ul     }, // 40 ms
    { IO_DONE     , 0xffff     , 0           }
};

/****/
const eWaveGenCmd  WAV_THRES_A_WRITE[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_VAL  , 0xffff , 86          },  // 86 bits includiny parity bits
    { IO_OUT_1    , 0xffff , TD          },
    { IO_DONE     , 0xffff , 0           }
};

const eWaveGenCmd  WAV_THRES_A_READ[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_INIT     , IO_IN_PU , 0         }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff , (100*TBIT)  },  // 86 bits includiny parity bits
    { IO_DONE     , 0xffff , 0           }
};


/****/
const eWaveGenCmd  WAV_THRES_B_WRITE[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_VAL  , 0xffff , 86          },  // 86 bits includiny parity bits
    { IO_OUT_1    , 0xffff , TD          },
    { IO_DONE     , 0xffff , 0           }
};

const eWaveGenCmd  WAV_THRES_B_READ[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_INIT     , IO_IN_PU , 0         }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff , (90*TBIT)   },  // 86 bits includiny parity bits
    { IO_DONE     , 0xffff , 0           }
};

/****/
const eWaveGenCmd  WAV_MEAS_WRITE[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_VAL  , 0xffff , 79          },  // 78 bits includiny parity bits
    { IO_OUT_1    , 0xffff , TD          },
    { IO_DONE     , 0xffff , 0           }
};

const eWaveGenCmd  WAV_MEAS_READ[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_INIT     , IO_IN_PU , 0         }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff , (80*TBIT)   },  // 78 bits includiny parity bits
    { IO_DONE     , 0xffff , 0           }
};

/****/
const eWaveGenCmd  WAV_READ_STATUS[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_INIT     , IO_IN_PU , 0         }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff , (20*TBIT)   },  // 16 bits includiny parity bits
    { IO_DONE     , 0xffff , 0           }
};

/****/
const eWaveGenCmd  WAV_CAL_PULSE[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_INIT     , IO_IN_PU , 0         }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff , 20000ul     },  // 16758 is the max value.
    { IO_DONE     , 0xffff , 0           }
};


/****/
const eWaveGenCmd  WAV_READ_TEMP[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_INIT     , IO_IN_PU , 0         }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff , (20*TBIT)   },  // 16 bit ?
    { IO_DONE     , 0xffff , 0           }
};


/****/
const eWaveGenCmd  WAV_READ_NFD_STATUS[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_INIT     , IO_IN_PU , 0         }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff , (20*TBIT)   },  // 11    bit
    { IO_DONE     , 0xffff , 0           }
};


/****/
const eWaveGenCmd  WAV_ENV_SEND_A[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_INIT     , IO_IN_ANA , 0        }, // Change IO Mode to Input
    { IO_IN_ANA   , 0xffff , 40000ul     },  // Analog In ==> Use ADC for sampling
    { IO_DONE     , 0xffff , 0           }
};


/****/
const eWaveGenCmd  WAV_ENV_RECEIVE_A[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_INIT     , IO_IN_ANA , 0        }, // Change IO Mode to Input
    { IO_IN_ANA   , 0xffff , 40000ul     },  // Analog In ==> Use ADC for sampling
    { IO_DONE     , 0xffff , 0           }
};


//-- SND on bits [31:16], REC on bits [15:0]
const eWaveGenCmd  WAV_ENV_SEND_RECEIVE[] PROGMEM = {
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


//-- JTAG Waveforms: SND on bits [31:16], REC on bits [15:0]
const eWaveGenCmd  WAV_JTAG_SEND_A[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TSND      },
    { IO_INIT     , IO_IN_PU, 0        }, // Change IO Mode to Input
    { IO_DONE     , 0xffff , 0         }
};

const eWaveGenCmd  WAV_JTAG_RECEIVE_A[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TREC      },
    { IO_INIT     , IO_IN_PU, 0        }, // Change IO Mode to Input
    { IO_DONE     , 0xffff , 0         }
};

const eWaveGenCmd  WAV_JTAG_SEND_RECEIVE_A[] PROGMEM = {
    { IO_OUT_BUS  , 0xffff0000  ,(TREC-TSND)    }, // Send (Master) is bit 0, all others are slaves
    { IO_OUT_BUS  , 0x00000000  , TSND          },
    { IO_INIT     , IO_IN_PU    , 0             }, // Change IO Mode to Input
    { IO_DONE     , 0xffff      ,0              }
};

//-- 
const eWaveGenCmd  WAV_JTAG_ENV_SEND_RECEIVE[] PROGMEM = {
    { IO_OUT_BUS  , 0x00000000 , TCMD        },
    { IO_OUT_BUS  , 0xffffffff , TD          },
    { IO_OUT_BIT  , 0xffffffff , 1           },
    { IO_OUT_BIT  , 0x00000000 , 0           },
    { IO_OUT_BIT  , 0x00000000 , 0           },
    { IO_OUT_BUS  , 0x00000000 , TBIT_PHASE  }, // start BIT0
    { IO_OUT_BUS  , 0x0000ffff , TBIT_PHASE  }, // SND = 0, REC = 1
    { IO_OUT_BUS  , 0xffffffff , TBIT_PHASE  }, // end BIT 0
    { IO_OUT_BUS  , 0x00000000 , TBIT_PHASE  }, // start BIT1
    { IO_OUT_BUS  , 0xffff0000 , TBIT_PHASE  }, // SND = 1, REC = 0
    { IO_OUT_BUS  , 0xffffffff , TBIT_PHASE  }, // end BIT 1
    { IO_INIT     , IO_IN_ANA  , 0           }, // Change IO Mode to Input
    { IO_DONE     , 0xffffffff , 0           }
};




/****/
const eWaveGenCmd  WAV_CALIB_WRITE[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_VAL  , 0xffff , 74          },  // 74 bits 
    { IO_OUT_1    , 0xffff , TD          },
    { IO_DONE     , 0xffff , 0           }
};


const eWaveGenCmd  WAV_CALIB_READ[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_INIT     , IO_IN_PU , 0         }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff , (100*TBIT)  },  // 74    bit
    { IO_DONE     , 0xffff , 0           }
};

const eWaveGenCmd  WAV_EE_COPY[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_INIT     , IO_IN_PU , 0         }, // Change IO Mode to Input
    { IO_OUT_PROG , 0xffff , TVPROG      },  // VPROG ON
    { IO_OUT_PROG , 0xffff , TPROG       },  // VPROG ON
    { IO_OUT_PROG , 0x0000 , TVPROG      },  // VPROG OFF
    { IO_DONE     , 0xffff , 0           }
};                   

const eWaveGenCmd  WAV_EE_READ[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_INIT     , IO_IN_PU , 0         }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff , (300*TBIT)  },  // 3 * (74+1)    bit (repeated 3 times)
    { IO_DONE     , 0xffff , 0           }
};

const eWaveGenCmd  WAV_WAKE_UP[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_DONE     , 0xffff , 0           }
};

const eWaveGenCmd  WAV_STANDBY[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_DONE     , 0xffff , 0           }
};

const eWaveGenCmd WAV_READ_ID[] PROGMEM = {
    { IO_OUT_0    , 0x0000 , TCMD        },
    { IO_OUT_1    , 0xffff , TD          },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0xffff , 1           },
    { IO_OUT_BIT  , 0x0000 , 0           },
    { IO_INIT     , IO_IN_PU , 0         }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff , (30*TBIT)   }, // 24 bit    bit (repeated 3 times)
    { IO_DONE     , 0xffff , 0           }
};


const eWaveGenCmd  WAV_NO_CMD[] PROGMEM = {
    { IO_DONE     , 0xffff , 0         }
};

const eWaveGenCmd  WAV_TEST_CMD[] PROGMEM = {
    { IO_INIT     , IO_IN_PU , 0          }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff , (8*30*TBIT)  },  // 30 byte
    { IO_DONE     , 0xffff , 0            }
};


/*******************
* Non tdk_ussm Commands
********************/
//-- DHT11 Temperature and Humidity Sensor
const eWaveGenCmd WAV_DHT11_READ[] PROGMEM = {
    { IO_OUT_0    , 18000ul             }, // 18 ms
    { IO_OUT_1    , 30ul                },
    { IO_INIT     , IO_IN_PU , 0        }, // Change IO Mode to Input
    { IO_IN_EDGE  , ((70+50)*40ul)      },  // 40 bit assuming all 1
    { IO_DONE     , 0                   }
};


//-- HC_SR04 Ultrasonic Sensor
const eWaveGenCmd WAV_HC_SR04[] PROGMEM = {
    { IO_OUT_0    , 5ul                 }, // 5 us
    { IO_OUT_1    , 10ul                },
    { IO_OUT_0    , 0ul                 }, // Start measurement
    { IO_INIT     , IO_IN_PU , 0        }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff   , 40000ul  }, // 40 ms Time of Flight maximum
    { IO_DONE     , 0xffff   , 0        }
};

//-- JSN_SR04T_V3.0 Ultrasonic Sensor
const eWaveGenCmd WAV_JSN_SR04T_V3[] PROGMEM = {
    { IO_OUT_0    , 5ul                 }, // 10 us
    { IO_OUT_1    , 20ul                },
    { IO_OUT_0    , 0ul                 }, // Start measurement
    { IO_INIT     , IO_IN_PU , 0        }, // Change IO Mode to Input
    { IO_IN_EDGE  , 0xffff   , 40000ul  }, // 40 ms Time of Flight maximum
    { IO_DONE     , 0xffff   , 0        }
};

/******************************************/
/*! \ingroup tdk_ussm
 * \brief Commands Arrays
 */
const eTdkUssmCmdStruct C_TDK_USSM_CMD[TDK_USSM_N_CMD]  PROGMEM = {
      //Cmd , Pulse ,   Id              ,   id_bits , rx_bits   , tx_bits   , io mode , Wave, name   , Usage
    {"sna", TSND  ,CMD_SEND_A           , 0  ,  0 ,  0, 0 , WAV_SEND_A            , "SEND_A"                    , "sna"},
    {"rca", TREC  ,CMD_RECEIVE_A        , 0  ,  0 ,  0, 0 , WAV_RECEIVE_A         , "RECEIVE_A"                 , "rca"},   
                                                                                                                                                                                        
    {"snb", TMEAS ,CMD_SEND_B           , 2  ,  0 ,  0, 0 , WAV_SEND_B            , "SEND_B"                    , "snb"},
    {"rcb", TMEAS ,CMD_RECEIVE_B        , 2  ,  0 ,  0, 0 , WAV_RECEIVE_B         , "RECEIVE_B"                 , "rcb"},     
                                                                                                                                                                                        
    {"snc", TMEAS ,CMD_SEND_C           , 2  ,  0 ,  0, 0 , WAV_SEND_C            , "SEND_C"                    , "snc"},
    {"rcc", TMEAS ,CMD_RECEIVE_C        , 2  ,  0 ,  0, 0 , WAV_RECEIVE_C         , "RECEIVE_C"                 , "rcc"}, 
                                                                                                                         
    {"sra", TNONE ,CMD_SEND_RECEIVE_A   , 5  ,  0 ,  0, 0 , WAV_SEND_RECEIVE_A    , "SEND_RECEIVE_A"            , "sra <master_num>"}, 
    {"srb", TNONE ,CMD_SEND_RECEIVE_B   , 5  ,  0 ,  0, 0 , WAV_SEND_RECEIVE_B    , "SEND_RECEIVE_B"            , "srb <master_num>"}, 
    {"src", TNONE ,CMD_SEND_RECEIVE_C   , 5  ,  0 ,  0, 0 , WAV_SEND_RECEIVE_C    , "SEND_RECEIVE_C"            , "src <master_num>"}, 

    {"taw", TCMD ,CMD_THRES_A_WRITE     , 5  ,  0 , 80, 0 , WAV_THRES_A_WRITE     , "THRES_A_WRITE"             , "taw [11 hex bytes]"},
    {"tar", TCMD ,CMD_THRES_A_READ      , 5  , 80 ,  0, 0 , WAV_THRES_A_READ      , "THRES_A_READ"              , "tar [details_hex_mask]"}, 
                                                                                                                                                                                                  
    {"tbw", TCMD ,CMD_THRES_B_WRITE     , 5  ,  0 , 80, 0 , WAV_THRES_B_WRITE     , "THRES_B_WRITE"             , "tbw [11 hex bytes]"},
    {"tbr", TCMD ,CMD_THRES_B_READ      , 5  , 80 ,  0, 0 , WAV_THRES_B_READ      , "THRES_B_READ"              , "tbr [details_hex_mask]"},  
                                                                                                                                                                                                  
    {"mwr", TCMD ,CMD_MEAS_WRITE        , 5  ,  0 , 73, 0 , WAV_MEAS_WRITE        , "MEAS_WRITE"                , "mwr [10 hex bytes]"},
    {"mrd", TCMD ,CMD_MEAS_READ         , 5  , 73 ,  0, 0 , WAV_MEAS_READ         , "MEAS_READ"                 , "mrd [details_hex_mask]"},  
                                                                                                                                                                                                  
    {"rds", TCMD ,CMD_READ_STATUS       , 5  , 17 ,  0, 0 , WAV_READ_STATUS       , "READ_STATUS"               , "rds [details_hex_mask]"},
    {"clp", TCMD ,CMD_CAL_PULSE         , 5  ,  2 ,  0, 0 , WAV_CAL_PULSE         , "CAL_PULSE"                 , "clp"},  
    {"rdt", TCMD ,CMD_READ_TEMP         , 5  , 10 ,  0, 0 , WAV_READ_TEMP         , "READ_TEMP"                 , "rdt [details_hex_mask]"},  
    {"rid", TCMD ,CMD_READ_ID           , 5  , 24 ,  0, 0 , WAV_READ_ID           , "READ_ID"                   , "rid"},  
                                                                                                                                                                                                  
    {"rns", TCMD ,CMD_READ_NFD_STATUS   , 5  , 24 ,  0, 0 , WAV_READ_NFD_STATUS   , "READ_NFD_STATUS"           , "rns [details_hex_mask]"}, 
                                                                                                                                    
    {"esa", TCMD ,CMD_ENV_SEND_A        , 5  ,  0 ,  0, 0 , WAV_ENV_SEND_A        , "ENV_SEND_A"                , "esa [<n_sample> <ts_us> ]"}, 
    {"era", TCMD ,CMD_ENV_RECEIVE_A     , 5  ,  0 ,  0, 0 , WAV_ENV_RECEIVE_A     , "ENV_RECEIVE_A"             , "era [<n_sample> <ts_us> ]"}, 
    {"esr", TNONE,CMD_ENV_SEND_RECEIVE  , 5  ,  0 ,  0, 0 , WAV_ENV_SEND_RECEIVE  , "ENV_SEND_RECEIVE"          , "esr [<n_sample> <ts_us> <master_num>]"},
                                                                                                                                                  
    {"jsa", TCMD ,CMD_JTAG_SEND_A       , 0  ,  0 ,  0, 0 , WAV_JTAG_SEND_A          , "JTAG_SEND_A"            , "jsa [<n_sample> <ts_us> ]"}, 
    {"jra", TCMD ,CMD_JTAG_RECEIVE_A    , 0  ,  0 ,  0, 0 , WAV_JTAG_RECEIVE_A       , "JTAG_RECEIVE_A"         , "jra [<n_sample> <ts_us> ]"}, 
    {"jsr", TNONE,CMD_JTAG_SEND_RECEIVE , 5  ,  0 ,  0, 0 , WAV_JTAG_SEND_RECEIVE_A  , "JTAG_SEND_RECEIVE"      , "jsr [<n_sample> <ts_us> <master_num>]"},
    {"jth", TNONE,CMD_JTAG_THRESH_READ  , 5  ,  0 ,  0, 0 , WAV_JTAG_SEND_RECEIVE_A  , "JTAG_THRESH_READ"       , "jth [<n_sample> <ts_us> <master_num>]"},
    {"jet", TNONE,CMD_JTAG_ENVTHRES_READ, 5  ,  0 ,  0, 0 , WAV_JTAG_SEND_RECEIVE_A  , "JTAG_ENVTHRES_READ"     , "jet [<n_sample> <ts_us> <master_num>]"},

    {"srh", TNONE,CMD_SEND_RECEIVE_AH   , 0 ,  0 ,   0, 0 , WAV_SEND_RECEIVE_AH    , "SEND_RECEIVE_AH"           , "srh <master_num>"}, 
    
    {"cwr", TCMD ,CMD_CALIB_WRITE       , 5  ,  0 , 74, 0 , WAV_CALIB_WRITE       , "CALIB_WRITE"               , "cwr [10 hex bytes]"},
    {"crd", TCMD ,CMD_CALIB_READ        , 0  , 74 ,  0, 0 , WAV_CALIB_READ        , "CALIB_READ"                , "crd [details_hex_mask]"},  
    {"eec", TCMD ,CMD_EE_COPY           , 5  ,  0 ,  0, 0 , WAV_EE_COPY           , "EE_COPY"                   , "eec"}, 
    {"eer", TCMD ,CMD_EE_READ           , 5  ,225 ,  0, 0 , WAV_EE_READ           , "EE_READ"                   , "eer [details_hex_mask]"},     
                                                                                                                                                                                        
    {"wkp", TCMD ,CMD_WAKE_UP           , 5  ,  0 ,  0, 0 , WAV_WAKE_UP           , "WAKE_UP"                   , "wkp"}, 
    {"std", TCMD ,CMD_STANDBY           , 5  ,  0 ,  0, 0 , WAV_STANDBY           , "STANDBY"                   , "std"},  
                
    {"set", TNONE ,CMD_SET              , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "SET"                       , "set <PARAM_NAME> <Value>|[<RalValue> -r]"},               
    {"get", TNONE ,CMD_GET              , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "GET"                       , "get <PARAM_NAME> [-rl]"},
    {"ral", TNONE ,CMD_READ_ALL         , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "READ_ALL"                  , "ral"},        
    {"sto", TNONE ,CMD_STREAMOUT        , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "STREAMOUT"                 , "sto [<max_iterations> <delay_ms> <mode>]"},  
    {"pol", TNONE ,CMD_POLL             , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "POLL"                      , "pol <max_iterations> <delay_ms> <cmd>"},  
    {"end", TNONE ,CMD_END              , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "END"                       , "end"},
 
    {"caf", TNONE ,CMD_CAL_FDRV         , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "CAL_FDRV"                  , "caf [<max_iterations> <delay_ms>]"},    
    {"cao", TNONE ,CMD_CAL_OSCILLATOR   , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "CAL_OSCILLATOR"            , "cao [<max_iterations> <delay_ms>]"},
    {"cag", TNONE ,CMD_CAL_GAIN         , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "CAL_GAIN"                  , "cag [<max_iterations> <delay_ms>]"},

    {"rxw", TNONE ,CMD_RX_WAVE          , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "RX_WAVE"                   , "rxw [<offset>]"},

    {"spw", TNONE ,CMD_SOFT_PARAM_WRITE , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "SOFT_PARAM_WRITE"          , "spw [40 hex bytes]"},
    {"spr", TNONE ,CMD_SOFT_PARAM_READ  , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "SOFT_PARAM_READ"           , "spr [<details_hex_mask>]"},

    {"hid", TNONE ,CMD_HARDWARE_ID      , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "HARDWARE_ID"               , "hid"},        
    {"com", TNONE ,CMD_COM_BAUDRATE     , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "COM_BAUDRATE"              , "com <new_baudrate>"},
    {"rst", TNONE ,CMD_RESET            , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "RESET_DEVICE"              , "rst"},
    {"dia", TNONE ,CMD_DIAGNOSIS        , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "DIAGNOSIS"                 , "diag"},        
    {"upd", TNONE ,CMD_UPDATE_FIRMWARE  , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "UPDATE_FIRMWARE"           , "upd <passcode>"},
                
    {"hlp", TNONE ,CMD_HELP             , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "HELP"                      , "hlp"},
    {"nop", TNONE ,CMD_NOP              , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "NOP"                       , "nop"},
    {"   ", TNONE ,CMD_INVALID          , 0  ,  0 ,  0, 0 , WAV_NO_CMD            , "INVALID_CMD"               , "   "},        
};



//----------------------------------------------------
/******************************************/
/*! \ingroup tdk_ussm
 * \brief Read All Registers Sequence
 */
const char C_SEQ_READ_ALL[][5] PROGMEM = {
  "rid",
  "eer",
  "crd",
  "mrd",
  "tar",
  "tbr",
  "rdt",
  "clp",
  "sna",
  "rds",
  "rns",
  "end"  // Sequences
};

/******************************************/
/*! \ingroup tdk_ussm
 * \brief Sensor Calibration Sequences
 */
const char C_SEQ_CALIBRATE_OSC[][20] PROGMEM = {
  "set F_DRV 40000 -r ", // 40KHz 
  "clp",
  "seq OSC_CHK", // Trimm oscialltor and check condition
  "end"  
};

const char C_SEQ_CALIBRATE_FDRV[][20] PROGMEM = {
  "crd",
  "sna",
  "rds",
  "seq FDRV_CHK", // Adjust frequency and check condition
  "end"  
};

const char C_SEQ_CALIBRATE_SDAMP[][20] PROGMEM = {
  "crd",
  "sna",
  "seq SDAMP_CHK", // Check for minimum value of ringdown
  "end"  
};

const char C_SEQ_CALIBRATE_VDRV[][20] PROGMEM = {
  "crd",
  "sra[0x3] 0 ", // Sensor0 Sending, Sensor1=Ref_Sensor Receiving
  "rds",
  "seq VDRV_CHK", // Adjust VDRIVE and check exit condition
  "end"  
};

const char C_SEQ_CALIBRATE_GAIN[][20] PROGMEM = {
  "crd",
  "sna",
  "rds",
  "seq GAIN_CHK", // Adjust GAIN and check exit condition
  "end"  
};

const char C_SEQ_CALIBRATE_NFD[][20] PROGMEM = {
  "crd",
  "sna",
  "rds",
  "rns",
  "seq NFD_CHK", // Adjust NFD_WIND, NFD_THRES, NFD_TOFF for optimum NFD
  "end"  
};

const char C_SEQ_CALIBRATE_THRES_A[][20] PROGMEM = {
  "crd",
  "tar",
  "sna",
  "rds",
  "seq THRES_A_CHK", // Adjust NFD_WIND, NFD_THRES, NFD_TOFF for optimum NFD
  "end"  
};

const char C_SEQ_CALIBRATE_THRES_B[][20] PROGMEM = {
  "crd",
  "tbr",
  "snb",
  "rds",
  "seq THRES_B_CHK", // Adjust NFD_WIND, NFD_THRES, NFD_TOFF for optimum NFD
  "end"  
};

//----------------------------------------------------
/******************************************/
/*! \ingroup TDK_USSM
 * \brief Frequency deviation LINEAR/LUT
 */
const eLUTStruct C_LUT_FREQ_DEV[] PROGMEM ={
    {0       ,   0     , "%"},
    {1       ,   0.78  , "%"},
    {15      ,  11.7   , "%"},
    {16      , -12.48  , "%"},
    {31      ,  -0.78  , "%"},
    {LUT_END ,   0     , "%"}
};


/******************************************/
/*! \ingroup TDK_USSM
 * \brief Frequency deviation LINEAR/LUT
 */
const eLUTStruct C_LUT_FDEV[] PROGMEM ={
    {0   , 3.12   , "%>|f_dev|"},
    {1   , 6.24   , "%>|f_dev|"},
    {2   , 9.36   , "%>|f_dev|"},
    {3   , 9.36   , "%<|f_dev|"},
    {LUT_END ,   0, ""}
};

/******************************************/
/*! \ingroup TDK_USSM
 * \brief Temperature deviation LINEAR/LUT
 */
const eLUTStruct C_LUT_TEMP[] PROGMEM ={
    {315  , -40 , "C" },
    {588  , 100 , "C" },
    {LUT_END , 0, ""  }
};


/******************************************/
/*! \ingroup TDK_USSM
 * \brief Threshold Values LUT
 */
const eLUTStruct C_LUT_THVAL[] PROGMEM ={
     {0  ,0    , "lsb"},
     {1  ,5    , "lsb"},
     {2  ,6    , "lsb"},
     {3  ,8    , "lsb"},
     {4  ,9    , "lsb"},
     {5  ,11   , "lsb"},
     {6  ,13   , "lsb"},
     {7  ,15   , "lsb"},
     {8  ,17   , "lsb"},
     {9  ,19   , "lsb"},
     {10 ,22   , "lsb"},
     {11 ,25   , "lsb"},
     {12 ,28   , "lsb"},
     {13 ,31   , "lsb"},
     {14 ,34   , "lsb"},
     {15 ,38   , "lsb"},
     {16 ,42   , "lsb"},
     {17 ,46   , "lsb"},
     {18 ,51   , "lsb"},
     {19 ,56   , "lsb"},
     {20 ,61   , "lsb"},
     {21 ,67   , "lsb"},
     {22 ,73   , "lsb"},
     {23 ,79   , "lsb"},
     {24 ,86   , "lsb"},
     {25 ,94   , "lsb"},
     {26 ,102  , "lsb"},
     {27 ,111  , "lsb"},
     {28 ,121  , "lsb"},
     {29 ,131  , "lsb"},
     {30 ,142  , "lsb"},
     {31 ,155  , "lsb"},
     {LUT_END, 0, "" }
};


/******************************************/
/*! \ingroup TDK_USSM
 * \brief Threshold Position LUT
 */
const eLUTStruct C_LUT_THPOS[] PROGMEM ={
     {0 , 128   , "us"},
     {1 , 256   , "us"},
     {2 , 512   , "us"},
     {3 , 1024  , "us"},
     {4 , 2048  , "us"},
     {5 , 4096  , "us"},
     {6 , 8192  , "us"},
     {7 , 8192  , "us"},
     {LUT_END, 0, "" }
};



/******************************************/
/*! \ingroup TDK_USSM
 * \brief Threshold Position LUT
 */
const eLUTStruct C_LUT_TSENS_TRIM[] PROGMEM ={
    {0 , -12   , "lsb"},
    {1 , -9    , "lsb"},
    {2 , -6    , "lsb"},
    {3 , -3    , "lsb"},
    {4 ,  0    , "lsb"},
    {5 , +3    , "lsb"},
    {6 , +6    , "lsb"},
    {7 , +12   , "lsb"},
    {LUT_END, 0, ""   }
};

const eLUTStruct C_LUT_OSC_TRIM[] PROGMEM ={
    {0  , 0    , "%" },
    {1  , 0.5  , "%" },
    {7  , 3.5  , "%" },
    {8  , -4.0 , "%" },
    {15 , -0.5 , "%" },
    {LUT_END, 0, ""  }
};

const eLUTStruct C_LUT_NFD_WIN[] PROGMEM ={
    {0  , 12 , "Samples"},
    {1  , 16 , "Samples"},
    {2  , 20 , "Samples"},
    {3  , 24 , "Samples"},
     {LUT_END, 0, ""     }
};

const eLUTStruct C_LUT_NFD_THRES[] PROGMEM ={
    {0  , 24  , "* (1+NFD_WIN/3) "},
    {1  , 36  , "* (1+NFD_WIN/3) "},
    {2  , 48  , "* (1+NFD_WIN/3) "},
    {3  , 60  , "* (1+NFD_WIN/3) "},
    {4  , 84  , "* (1+NFD_WIN/3) "},
    {5  , 108 , "* (1+NFD_WIN/3) "},
    {6  , 150 , "* (1+NFD_WIN/3) "},
    {7  , 192 , "* (1+NFD_WIN/3) "},
    {LUT_END, 0, "" }
};

const eLUTStruct C_LUT_NFD_TOFF[] PROGMEM ={
    {0  , 100 , "us"},
    {1  , 200 , "us"},
    {2  , 300 , "us"},
    {3  , 400 , "us"},
    {4  , 500 , "us"},
    {5  , 600 , "us"},
    {6  , 700 , "us"},
    {7  , 800 , "us"},
    {LUT_END, 0, "" }
};

const eLUTStruct C_LUT_G_DIG[] PROGMEM ={
    {0   , 0     , "dB"},
    {1   , 0.38  , "dB"},
    {127 , -0.5  , "dB"},
    {LUT_END, 0, "" }
};

const eLUTStruct C_LUT_G_ANA[] PROGMEM ={
    {0  , 39.2 , "dB"},
    {1  , 41.6 , "dB"},
    {2  , 44.0 , "dB"},
    {3  , 46.4 , "dB"},
    {4  , 48.8 , "dB"},
    {5  , 51.2 , "dB"},
    {6  , 53.6 , "dB"},
    {7  , 56.0 , "dB"},
    {LUT_END, 0, "" }
};

const eLUTStruct C_LUT_V_DRV[] PROGMEM ={
    {0  , 0    , ":Off"},
    {1  , 15.7 , "V"},
    {2  , 15.7 , "V"},
    {3  , 16.6 , "V"},
    {4  , 17.6 , "V"},
    {5  , 18.6 , "V"},
    {6  , 19.7 , "V"},
    {7  , 20.8 , "V"},
    {8  , 22.1 , "V"},
    {9  , 23.4 , "V"},
    {10 , 24.8 , "V"},
    {11 , 26.3 , "V"},
    {12 , 27.9 , "V"},
    {13 , 29.6 , "V"},
    {14 , 31.3 , "V"},
    {15 , 31.3 , "V"},
    {LUT_END, 0, "" }
};

/******************************************/
/*! \ingroup TDK_USSM
 * \brief Linear F_DRV LUT
 */
const eLUTStruct C_LUT_F_DRV[] PROGMEM ={
    {0   , 83333  , "Hz"},
    {8   , 78947  , "Hz"},
    {12  , 76923  , "Hz"},
    {16  , 75000  , "Hz"},
    {20  , 73171  , "Hz"},
    {25  , 71006  , "Hz"},
    {56  , 60000  , "Hz"},
    {96  , 50000  , "Hz"},
    {156 , 40000  , "Hz"},
    {255 , 30075  , "Hz"},
    {LUT_END, 0, "" }
};

#define  Freq_to_F_DRV(f) ((12000ul/(f/1000))-144 )  // F_DRV   = (f_osc/freq_Hz) - 144
#define  F_DRV_to_Freq(N) (1000*12000ul/(N+144))  // freq_Hz = 1000*(f_osc/(F_DRV + 144))


/******************************************/
/*! \ingroup TDK_USSM
 * \brief Number of burst pulses LUT
 */
const eLUTStruct C_LUT_NPULSES[] PROGMEM ={ 
    { 0, 4    , "pulses"},
    { 1, 8    , "pulses"},
    { 2, 12   , "pulses"},
    { 3, 16   , "pulses"},
    { 4, 20   , "pulses"},
    { 5, 24   , "pulses"},
    { 6, 28   , "pulses"},
    { 7, 32   , "pulses"},
    {LUT_END, 0, "" }
    };

    
/******************************************/
/*! \ingroup TDK_USSM
 * \brief Measurement time in us LUT
 */
const eLUTStruct C_LUT_TMEAS[] PROGMEM ={ 
    { 0,  8.75 , "ms"},
    { 1, 11.66 , "ms"},
    { 2, 14.58 , "ms"},
    { 3, 17.49 , "ms"},
    { 4, 20.41 , "ms"},
    { 5, 23.32 , "ms"},
    { 6, 29.15 , "ms"},
    { 7, 34.98 , "ms"},
    {LUT_END, 0, "" }
};
    
/******************************************/
/*! \ingroup TDK_USSM
 * \brief Threshold Selection LUT (Th_A, Th_B)
 */
const eLUTStruct C_LUT_TSEL[] PROGMEM ={ 
    { 0, 0 , ":Threshold A"},
    { 1, 1 , ":Threshold B"},
    {LUT_END, 0, "" }
};

/******************************************/
/*! \ingroup TDK_USSM
 * \brief ON/OFF LUT
 */
const eLUTStruct C_LUT_ON_OFF[] PROGMEM ={ 
    { 0, 0 , ":OFF"},
    { 1, 1 , ":ON "},
    {LUT_END, 0, "" }
};

/******************************************/
/*! \ingroup TDK_USSM
 * \brief Filter Config LUT
 */
const eLUTStruct C_LUT_FILTER_CFG[] PROGMEM ={ 
    { 0 , 0 , ":automatic"},
    { 1 , 1 , ":wide     "},
    { 2 , 2 , ":medium   "},
    { 3 , 3 , ":narrow   "},
    {LUT_END, 0, "" }
};


/******************************************/
/*! \ingroup TDK_USSM
 * \brief Sensitivity Time Control (STC) Gain LUT
 */
const eLUTStruct C_LUT_STC_GAIN[] PROGMEM ={ 
  { 0,   0, ": 0    "},
  { 1,  -8, "* DGDig"},
  { 2, -12, "* DGDig"},
  { 3, -16, "* DGDig"},
  { 4,   8, "* DGDig"},
  { 5,  12, "* DGDig"},
  { 6,  16, "* DGDig"},
  { 7,  20, "* DGDig"},
  { 8,  24, "* DGDig"},
  { 9,  28, "* DGDig"},
  {10,  32, "* DGDig"},
  {11,  36, "* DGDig"},
  {12,  40, "* DGDig"},
  {13,  44, "* DGDig"},
  {14,  48, "* DGDig"},
  {15,  52, "* DGDig"},
  {LUT_END, 0, "" }
};



/******************************************/
/*! \ingroup TDK_USSM
 * \brief Sensitivity Time Control (STC) Position LUT
 */
const eLUTStruct C_LUT_STC_POS[] PROGMEM ={ 
  { 0,  128 , "us"},
  { 1,  256 , "us"},
  { 2,  512 , "us"},
  { 3, 1024 , "us"},
  { 4, 2048 , "us"},
  { 5, 4096 , "us"},
  { 6, 8192 , "us"},
  { 7, 8192 , "us"},
  {LUT_END, 0, "" }
};


/******************************************/
/*! \ingroup TDK_USSM
 * \brief Time constant of ATG LUT
 */
const eLUTStruct C_LUT_ATG_TAU[] PROGMEM ={ 
  { 0 , 0 , ": slow  "},
  { 1 , 1 , ": medium"},
  { 2 , 2 , ": fast 1"},
  { 3 , 3 , ": fast 2"},
  {LUT_END, 0, "" } 
};


/******************************************/
/*! \ingroup TDK_USSM
 * \brief Automatic threshold configuration
 */
const eLUTStruct C_LUT_ATG_CFG[] PROGMEM ={ 
  { 0 , 0.0 , ": ATG off"   },
  { 1 , 2.5 , "ms to ATG On"},
  { 2 , 6.5 , "ms to ATG On"},
  { 3 ,10.5 , "ms to ATG On"},
  {LUT_END, 0, "" } 
};


/******************************************/
/*! \ingroup TDK_USSM
 * \brief Multiplier of ATG LUT
 */
const eLUTStruct C_LUT_ATG_ALPHA[] PROGMEM ={ 
  { 0 , 0 ,": sensitive"     },
  { 1 , 1 ,": less sensitive"},
  {LUT_END, 0, "" }
};

/******************************************/
/*! \ingroup TDK_USSM
 * \brief Noise suppression configuration (%) LUT
 */
const eLUTStruct C_LUT_NSUPP_CFG[] PROGMEM ={ 
  { 0 , 0   , "% noise reduction"},
  { 1 , 50  , "% noise reduction"},
  { 2 , 75  , "% noise reduction"},
  { 3 , 100 , "% noise reduction"},
  {LUT_END, 0, "" } 
};


/******************************************/
/*! \ingroup TDK_USSM
 * \brief Noise threshold for noise measurement LUT
 */
const eLUTStruct C_LUT_NOISE_CFG[] PROGMEM ={ 
  { 0 , 16  , "lsb"},
  { 1 , 32  , "lsb"},
  { 2 , 64  , "lsb"},
  { 3 ,128  , "lsb"},
  {LUT_END, 0, "" } 
};


/******************************************/
/*! \ingroup TDK_USSM
 * \brief Down scaling in receive mode for Threshold A and B LUT
 */
const eLUTStruct C_LUT_THRESSCALE_REC[] PROGMEM ={ 
  { 0 , 0.5    , "full scale"},
  { 1 , 0.625  , "full scale"},
  { 2 , 0.8125 , "full scale"},
  { 3 , 1      , "full scale"},
  {LUT_END, 0, "" } 
};


/******************************************/
/*! \ingroup TDK_USSM
 * \brief Configuration of wrap up status information LUT
 */
const eLUTStruct C_LUT_STATUS_CFG[] PROGMEM ={ 
  { 0 ,  2 , "bit: No status bits"},
  { 1 ,  8 , "bit: 6 diagnosis bits"},
  { 2 , 14 , "bit: diag + 1st echo" },
  { 3 , 20 , "bit: diag + 2 echos h"},
  {LUT_END, 0, "" } 
};


/******************************************/
/*! \ingroup TDK_USSM
 * \brief Null LUT
 */
const eLUTStruct C_LUT_NULL[] PROGMEM ={
  {LUT_END, 0, "" }
};



/******************************************/
/*! \ingroup TDK_USSM
 * \brief Measurement Array
 */
typedef enum
{
    NPULSE_A = (0x0200u + 0u),
    TMEAS_A,
    TSEL_A,
    NPULSE_B,
    TMEAS_B,
    TSEL_B,
    NPULSE_C,
    TMEAS_C,
    TSEL_C,
    ECHO_DEB,
    RT_CFG,
    NFTG,
    FTC,
    EPD,
    APD,
    FILTER_CFG,
    STC_GAIN4,
    STC_POS4,
    STC_GAIN3,
    STC_POS3,
    STC_GAIN2,
    STC_POS2,
    STC_GAIN1,
    STC_POS1,
    STC_POS0,
    ATG_CFG,
    ATG_TAU,
    ATG_ALPA,
    NSUPP_CFG,
    NOISE_CFG,
    THRESSCALE_REC,
    STATUS_CFG
} eMeasurementID;

#define  __N_MeasurementID  ((STATUS_CFG - NPULSE_A)+1)

const eParameterElement C_MEASUREMENT[__N_MeasurementID] PROGMEM = {
    { NPULSE_A         , 70 , 3, 3, "NPULSE_A"     , "Number of burst pulses - Profile A" , C_LUT_NPULSES      }, 
    { TMEAS_A          , 67 , 3, 2, "TMEAS_A"      , "Measurement time - Profile A     " , C_LUT_TMEAS         }, 
    { TSEL_A           , 66 , 1, 0, "TSEL_A"       , "Threshold selection - Profile A    " , C_LUT_TSEL        }, 

    { NPULSE_B         , 63 , 3, 1, "NPULSE_B"     , "Number of burst pulses - Profile B" , C_LUT_NPULSES      }, 
    { TMEAS_B          , 60 , 3, 0, "TMEAS_B"      , "Measurement time - Profile B      " , C_LUT_TMEAS        }, 
    { TSEL_B           , 59 , 1, 0, "TSEL_B"       , "Threshold selection - Profile B   " , C_LUT_TSEL         }, 

    { NPULSE_C         , 56 , 3, 5, "NPULSE_C"     , "Number of burst pulses - Profile C" , C_LUT_NPULSES      }, 
    { TMEAS_C          , 53 , 3, 6, "TMEAS_C"      , "Measurement time - Profile C      " , C_LUT_TMEAS        }, 
    { TSEL_C           , 52 , 1, 0, "TSEL_C"       , "Threshold selection - Profile C   " , C_LUT_TSEL         }, 

    { ECHO_DEB         , 51 , 1, 0, "ECHO_DEB"     , "Echo debouncing                   " , C_LUT_ON_OFF       }, 
    { RT_CFG           , 50 , 1, 0, "RT_CFG"       , "Ringing time configuration        " , C_LUT_ON_OFF       }, 
    { NFTG             , 49 , 1, 1, "NFTG"         , "Near field threshold generation   " , C_LUT_ON_OFF       }, 
    { FTC              , 48 , 1, 0, "FTC"          , "Fast Time Constant method         " , C_LUT_ON_OFF       }, 
    { EPD              , 47 , 1, 1, "EPD"          , "Echo Peak Detection               " , C_LUT_ON_OFF       }, 
    { APD              , 46 , 1, 0, "APD"          , "All-Peak Detection                " , C_LUT_ON_OFF       }, 
    { FILTER_CFG       , 44 , 2, 0, "FILTER_CFG"   , "Digital Filter config receive mode" , C_LUT_FILTER_CFG   }, 

    { STC_GAIN4        , 40 , 4, 7, "STC_GAIN4"    , "STC Gain 4                        " , C_LUT_STC_GAIN     }, 
    { STC_POS4         , 37 , 3, 5, "STC_POS4 "    , "STC Position 4 (Delta) +1ms offset" , C_LUT_STC_POS      }, 
    { STC_GAIN3        , 33 , 4, 8, "STC_GAIN3"    , "STC Gain 3                        " , C_LUT_STC_GAIN     }, 
    { STC_POS3         , 30 , 3, 4, "STC_POS3"     , "STC Position 3 (Delta) +1ms offset" , C_LUT_STC_POS      }, 
    { STC_GAIN2        , 26 , 4, 8, "STC_GAIN2"    , "STC Gain 2                        " , C_LUT_STC_GAIN     }, 
    { STC_POS2         , 23 , 3, 3, "STC_POS2"     , "STC Position 2 (Delta) +1ms offset" , C_LUT_STC_POS      }, 
    { STC_GAIN1        , 19 , 4, 8, "STC_GAIN1"    , "STC Gain 1                        " , C_LUT_STC_GAIN     }, 
    { STC_POS1         , 16 , 3, 2, "STC_POS1"     , "STC Position 1 (Delta) +1ms offset" , C_LUT_STC_POS      }, 
    { STC_POS0         , 13 , 3, 1, "STC_POS0"     , "STC Position 0 (Delta) +1ms offset" , C_LUT_STC_POS      }, 

    { ATG_CFG          , 11 , 2, 3, "ATG_CFG"      , "Automatic threshold configuration " , C_LUT_ATG_CFG      }, 
    { ATG_TAU          ,  9 , 1, 1, "ATG_TAU"      , "Time constant of ATG              " , C_LUT_ATG_TAU      }, 
    { ATG_ALPA         ,  8 , 1, 0, "ATG_ALPA"     , "Multiplier of ATG                 " , C_LUT_ATG_ALPHA    }, 

    { NSUPP_CFG        ,  6 , 2, 0, "NSUPP_CFG"    , "Noise suppression configuration   " , C_LUT_NSUPP_CFG    }, 
    { NOISE_CFG        ,  4 , 2, 2, "NOISE_CFG"    , "Threshold for noise measurement   " , C_LUT_NOISE_CFG    },

    { THRESSCALE_REC   ,  2 , 2, 1, "THRESSCALE_REC", "Down scaling receive mode ThA ThB" , C_LUT_THRESSCALE_REC }, 
    { STATUS_CFG       ,  0 , 2, 2, "STATUS_CFG"    , "Configuration of wrap up status  " , C_LUT_STATUS_CFG     } 
};



/******************************************/
/*! \ingroup TDK_USSM
 * \brief Calibration and Eeprom Array
 */
typedef enum
{
    TSENS_TRIM = (0x0300u +  0u),
    OSC_TRIM,
    reserved,
    NFD_WIN,
    NFD_THRES,
    NFD_TOFF,
    G_DIG,
    G_ANA,
    V_DRV,
    F_DRV,
    S_DAMP,
    CUSTOMER_BITS
} eCalibrationID;

#define  __N_CalibrationID ((CUSTOMER_BITS - TSENS_TRIM)+1)

const eParameterElement C_CALIBRATION[__N_CalibrationID] PROGMEM = {
    { TSENS_TRIM        , 71,  3, 0, "TSENS_TRIM"    ,"Temperature sensor trimming"          , C_LUT_TSENS_TRIM}, 
    { OSC_TRIM          , 67,  4, 0, "OSC_TRIM"      ,"Oscillator trimming"                  , C_LUT_OSC_TRIM  },     
    { reserved          , 66,  1, 0, "reserved"      ,"reserved"                             , C_LUT_NULL      },                                                                    
    { NFD_WIN           , 64,  2, 0, "NFD_WIN"       ,"NFD energy accumulation window"       , C_LUT_NFD_WIN   }, 
    { NFD_THRES         , 61,  3, 0, "NFD_THRES"     ,"Near-field detection energy threshold", C_LUT_NFD_THRES },     
    { NFD_TOFF          , 58,  3, 0, "NFD_TOFF"      ,"Near-field detection off time"        , C_LUT_NFD_TOFF  },     
    { G_DIG             , 51,  7, 0, "G_DIG"         ,"Digital amplifier gain setting"       , C_LUT_G_DIG     }, 
    { G_ANA             , 48,  3, 0, "G_ANA"         ,"Analog amplifier gain setting"        , C_LUT_G_ANA     }, 
    { V_DRV             , 44,  4, 0, "V_DRV"         ,"Transducer driver voltage setting"    , C_LUT_V_DRV     }, 
    { F_DRV             , 36,  8, 0, "F_DRV"         ,"Driver frequency setting"             , C_LUT_F_DRV     }, 
    { S_DAMP            , 32,  4, 0, "S_DAMP"        ,"Smart damping setting"                , C_LUT_NULL      }, 
    { CUSTOMER_BITS     ,  0, 32, 0, "CUSTOMER_BITS" ,"CUSTOMER BITS Free configurable"      , C_LUT_NULL      }      
};


/******************************************/
/*! \ingroup TDK_USSM
 * \brief Threshold A Array
 */
typedef enum
{
    THVAL_A1 = (0x0100u +  0u),
    THPOS_A1,
    THVAL_A2,
    THPOS_A2,
    THVAL_A3,
    THPOS_A3,
    THVAL_A4,
    THPOS_A4,
    THVAL_A5,
    THPOS_A5,
    THVAL_A6,
    THPOS_A6,
    THVAL_A7,
    THPOS_A7,
    THVAL_A8,
    THPOS_A8,
    THVAL_A9,
    THPOS_A9,
    THVAL_A10,
    THPOS_A10
} eThresholdAID;

#define  __N_ThresholdAID  ((THPOS_A10 - THVAL_A1)+1)

const eParameterElement C_THRESHOLD_A[__N_ThresholdAID] PROGMEM = {
    { THVAL_A1  , 75 , 5, 31, "THVAL_A1"  , "THVAL_A1 " , C_LUT_THVAL}, 
    { THPOS_A1  , 72 , 3,  1, "THPOS_A1"  , "THPOS_A1 " , C_LUT_THPOS},  
    { THVAL_A2  , 67 , 5, 15, "THVAL_A2"  , "THVAL_A2 " , C_LUT_THVAL},
    { THPOS_A2  , 64 , 3,  2, "THPOS_A2"  , "THPOS_A2 " , C_LUT_THPOS}, 
    { THVAL_A3  , 59 , 5, 15, "THVAL_A3"  , "THVAL_A3 " , C_LUT_THVAL},  
    { THPOS_A3  , 56 , 3,  2, "THPOS_A3"  , "THPOS_A3 " , C_LUT_THPOS},  
    { THVAL_A4  , 51 , 5, 10, "THVAL_A4"  , "THVAL_A4 " , C_LUT_THVAL}, 
    { THPOS_A4  , 48 , 3,  2, "THPOS_A4"  , "THPOS_A4 " , C_LUT_THPOS}, 
    { THVAL_A5  , 43 , 5, 10, "THVAL_A5"  , "THVAL_A5 " , C_LUT_THVAL}, 
    { THPOS_A5  , 40 , 3,  4, "THPOS_A5"  , "THPOS_A5 " , C_LUT_THPOS}, 
    { THVAL_A6  , 35 , 5, 10, "THVAL_A6"  , "THVAL_A6 " , C_LUT_THVAL}, 
    { THPOS_A6  , 32 , 3,  4, "THPOS_A6"  , "THPOS_A6 " , C_LUT_THPOS}, 
    { THVAL_A7  , 27 , 5,  5, "THVAL_A7"  , "THVAL_A7 " , C_LUT_THVAL},
    { THPOS_A7  , 24 , 3,  4, "THPOS_A7"  , "THPOS_A7 " , C_LUT_THPOS},
    { THVAL_A8  , 19 , 5,  5, "THVAL_A8"  , "THVAL_A8 " , C_LUT_THVAL},
    { THPOS_A8  , 16 , 3,  5, "THPOS_A8"  , "THPOS_A8 " , C_LUT_THPOS},
    { THVAL_A9  , 11 , 5,  5, "THVAL_A9"  , "THVAL_A9 " , C_LUT_THVAL},
    { THPOS_A9  ,  8 , 3,  5, "THPOS_A9"  , "THPOS_A9 " , C_LUT_THPOS},
    { THVAL_A10 ,  3 , 5,  0, "THVAL_A10" , "THVAL_A10" , C_LUT_THVAL},
    { THPOS_A10 ,  0 , 3,  5, "THPOS_A10" , "THPOS_A10" , C_LUT_THPOS}
};


/******************************************/
/*! \ingroup TDK_USSM
 * \brief Threshold A Array
 */
typedef enum
{
    THVAL_B1 = (0x0400u + 0u),
    THPOS_B1,
    THVAL_B2,
    THPOS_B2,
    THVAL_B3,
    THPOS_B3,
    THVAL_B4,
    THPOS_B4,
    THVAL_B5,
    THPOS_B5,
    THVAL_B6,
    THPOS_B6,
    THVAL_B7,
    THPOS_B7,
    THVAL_B8,
    THPOS_B8,
    THVAL_B9,
    THPOS_B9,
    THVAL_B10,
    THPOS_B10
} eThresholdBID;

#define  __N_ThresholdBID  ((THPOS_B10 - THVAL_B1)+1)

const eParameterElement C_THRESHOLD_B[__N_ThresholdBID] PROGMEM = {
    { THVAL_B1  , 75 , 5, 31, "THVAL_B1"  , "THVAL_B1 " , C_LUT_THVAL}, 
    { THPOS_B1  , 72 , 3,  1, "THPOS_B1"  , "THPOS_B1 " , C_LUT_THPOS},  
    { THVAL_B2  , 67 , 5, 15, "THVAL_B2"  , "THVAL_B2 " , C_LUT_THVAL},
    { THPOS_B2  , 64 , 3,  2, "THPOS_B2"  , "THPOS_B2 " , C_LUT_THPOS}, 
    { THVAL_B3  , 59 , 5, 15, "THVAL_B3"  , "THVAL_B3 " , C_LUT_THVAL},  
    { THPOS_B3  , 56 , 3,  2, "THPOS_B3"  , "THPOS_B3 " , C_LUT_THPOS},  
    { THVAL_B4  , 51 , 5, 10, "THVAL_B4"  , "THVAL_B4 " , C_LUT_THVAL}, 
    { THPOS_B4  , 48 , 3,  2, "THPOS_B4"  , "THPOS_B4 " , C_LUT_THPOS}, 
    { THVAL_B5  , 43 , 5, 10, "THVAL_B5"  , "THVAL_B5 " , C_LUT_THVAL}, 
    { THPOS_B5  , 40 , 3,  4, "THPOS_B5"  , "THPOS_B5 " , C_LUT_THPOS}, 
    { THVAL_B6  , 35 , 5, 10, "THVAL_B6"  , "THVAL_B6 " , C_LUT_THVAL}, 
    { THPOS_B6  , 32 , 3,  4, "THPOS_B6"  , "THPOS_B6 " , C_LUT_THPOS}, 
    { THVAL_B7  , 27 , 5,  5, "THVAL_B7"  , "THVAL_B7 " , C_LUT_THVAL},
    { THPOS_B7  , 24 , 3,  4, "THPOS_B7"  , "THPOS_B7 " , C_LUT_THPOS},
    { THVAL_B8  , 19 , 5,  5, "THVAL_B8"  , "THVAL_B8 " , C_LUT_THVAL},
    { THPOS_B8  , 16 , 3,  5, "THPOS_B8"  , "THPOS_B8 " , C_LUT_THPOS},
    { THVAL_B9  , 11 , 5,  5, "THVAL_B9"  , "THVAL_B9 " , C_LUT_THVAL},
    { THPOS_B9  ,  8 , 3,  5, "THPOS_B9"  , "THPOS_B9 " , C_LUT_THPOS},
    { THVAL_B10 ,  3 , 5,  0, "THVAL_B10" , "THVAL_B10" , C_LUT_THVAL},
    { THPOS_B10 ,  0 , 3,  5, "THPOS_B10" , "THPOS_B10" , C_LUT_THPOS}
};

/******************************************/
/*! \ingroup TDK_USSM
 * \brief Near Field Status Array
 */
typedef enum
{
    ENERGY_LEVEL = (0x0900u + 0u) ,
    T_COMP
} eNfdStatusID;

#define __N_NfdStatusID ((T_COMP - ENERGY_LEVEL)+1)

const eParameterElement C_NFD_STATUS[__N_NfdStatusID] PROGMEM = {
    { ENERGY_LEVEL , 11 , 13 , 0 , "ENERGY_LEVEL" , "Measured Energy Level"  , C_LUT_NULL},
    { T_COMP       , 0  , 11 , 0 , "T_COMP"       , "Ringdown By comparator" , C_LUT_NULL}
};



/******************************************/
/*! \ingroup TDK_USSM
 * \brief Near Temperature Array
 */
typedef enum
{
    TEMP_RAW_VAL = (0x0800u + 0u)
} eTemperatureID;

#define __N_TemperatureID ((TEMP_RAW_VAL - TEMP_RAW_VAL)+1)

const eParameterElement C_TEMPERATURE[__N_TemperatureID] PROGMEM ={
   { TEMP_RAW_VAL  , 0  , 10 , 0 , "TEMP" , "Temperature = 0.513*(N-442)+25 C", C_LUT_TEMP } 
};


/******************************************/
/*! \ingroup TDK_USSM
 * \brief Near Field Temperature Array
 */
typedef enum
{
   VPROG_STATUS0 = (0x0700u + 0u),
   CALIB0        ,
   VPROG_STATUS1 ,
   CALIB1        ,
   VPROG_STATUS2 ,
   CALIB2
} eEepromID;

#define __N_EepromID ((CALIB2 - VPROG_STATUS0) + 1)

const eParameterElement C_EEPROM[__N_EepromID] PROGMEM ={
   { VPROG_STATUS0 , 0   , 1  , 0 , "VPROG_STATUS0" , "VPROG Status sample 0"        , C_LUT_ON_OFF }, 
   { CALIB0        , 1   , 74 , 0 , "CALIB0"        , "EEPROM Calibration sample 0"  , C_LUT_NULL   }, 
   { VPROG_STATUS1 , 75  , 1  , 0 , "VPROG_STATUS1" , "VPROG Status sample 1"        , C_LUT_ON_OFF }, 
   { CALIB1        , 76  , 74 , 0 , "CALIB1"        , "EEPROM Calibration sample 1"  , C_LUT_NULL   }, 
   { VPROG_STATUS2 , 150 , 1  , 0 , "VPROG_STATUS2" , "VPROG Status sample 2"        , C_LUT_ON_OFF }, 
   { CALIB2        , 151 , 74 , 0 , "CALIB2"        , "EEPROM Calibration sample 3"  , C_LUT_NULL   }
};



/******************************************/
/*! \ingroup TDK_USSM
 * \brief Status Register Array
 */
typedef enum
{
   FREQ_DEVIATION = (0x0600u + 0u),
   FREQ_VALID        ,
   THRESHOLD_A_OK    ,
   THRESHOLD_B_OK    ,
   MEASUREMENT_OK    ,
   VSUP_UNDERVOLTAGE ,
   VDRV_UNDERVOLTAGE ,
   VDRV_OVERVOLTAGE  ,
   VTRANSDUCER_FAULT ,
   NOISE_FAULT       ,
   PRE_NOISE_FAULT   ,
   NEAR_FIELD_FLAG   ,
   STATUS_RESERVED
} eStatusID;

#define  __N_StatusID  ((STATUS_RESERVED - FREQ_DEVIATION)+1)

const eParameterElement C_STATUS[__N_StatusID] PROGMEM ={
   { FREQ_DEVIATION    , 0  , 5 , 0 , "FREQ_DEVIATION"    , "Signed Frequency deviation N*0.78%", C_LUT_FREQ_DEV }, 
   { FREQ_VALID        , 5  , 1 , 0 , "FREQ_VALID"        , "Frequency measurement valid"       , C_LUT_ON_OFF }, 
   { THRESHOLD_A_OK    , 6  , 1 , 0 , "THRESHOLD_A_OK"    , "Threshold A Configured"            , C_LUT_ON_OFF }, 
   { THRESHOLD_B_OK    , 7  , 1 , 0 , "THRESHOLD_B_OK"    , "Threshold B Configured"            , C_LUT_ON_OFF }, 
   { MEASUREMENT_OK    , 8  , 1 , 0 , "MEASUREMENT_OK"    , "Measurement Configured"            , C_LUT_ON_OFF }, 
   { VSUP_UNDERVOLTAGE , 9  , 1 , 0 , "VSUP_UNDERVOLTAGE" , "VSup Undervoltage"                 , C_LUT_ON_OFF }, 
   { VDRV_UNDERVOLTAGE , 10 , 1 , 0 , "VDRV_UNDERVOLTAGE" , "VDrv Undervoltage"                 , C_LUT_ON_OFF }, 
   { VDRV_OVERVOLTAGE  , 11 , 1 , 0 , "VDRV_OVERVOLTAGE"  , "VDrv Overvoltage"                  , C_LUT_ON_OFF }, 
   { VTRANSDUCER_FAULT , 12 , 1 , 0 , "VTRANSDUCER_FAULT" , "Transducer Voltage NOK"            , C_LUT_ON_OFF }, 
   { NOISE_FAULT       , 13 , 1 , 0 , "NOISE_FAULT"       , "Noise during measurement "         , C_LUT_ON_OFF }, 
   { PRE_NOISE_FAULT   , 14 , 1 , 0 , "PRE_NOISE_FAULT"   , "Noise before measurement "         , C_LUT_ON_OFF }, 
   { NEAR_FIELD_FLAG   , 15 , 1 , 0 , "NEAR_FIELD_FLAG"   , "Near Field Flag Set"               , C_LUT_ON_OFF },
   { STATUS_RESERVED   , 16 , 1 , 0 , "STATUS_RESERVED"   , "Reserved"                          , C_LUT_ON_OFF }
};



/******************************************/
/*! \ingroup TDK_USSM
 * \brief Measurement Status Array
 */
typedef enum
{
   ABS_FREQ_DEV = (0x0500u + 0u),
   MEAS_INVALID      ,
   NOISE_DETECTED    ,
   SR_RESERVED       ,
   NFD_FLAG          ,
   ECHO_HEIGHT1      ,
   ECHO_HEIGHT2
} eSndRecStatusID;

#define  __N_SndRecStatusID  ((ECHO_HEIGHT2 - ABS_FREQ_DEV)+1)

const eParameterElement C_SND_REC_STATUS[__N_SndRecStatusID] PROGMEM ={
   { ABS_FREQ_DEV      , 0  , 2 , 0 , "ABS_FREQ_DEV"      , "Absolute Freq deviation range"     , C_LUT_FDEV }, 
   { MEAS_INVALID      , 2  , 1 , 0 , "MEAS_INVALID"      , "Invalid measurement conditions"    , C_LUT_NULL }, 
   { NOISE_DETECTED    , 3  , 1 , 0 , "NOISE_DETECTED"    , "Noise detected before or during"   , C_LUT_NULL }, 
   { SR_RESERVED       , 4  , 1 , 0 , "SR_RESERVED"       , ""                                  , C_LUT_NULL }, 
   { NFD_FLAG          , 5  , 1 , 0 , "NFD_FLAG"          , "Near Field flag set"               , C_LUT_NULL }, 
   { ECHO_HEIGHT1      , 6  , 6 , 0 , "ECHO_HEIGHT1"      , "1st Echo Height"                   , C_LUT_NULL }, 
   { ECHO_HEIGHT2      , 12 , 6 , 0 , "ECHO_HEIGHT2"      , "2nd Echo Height"                   , C_LUT_NULL }
};



/******************************************/
/*! \ingroup TDK_USSM
 * \brief High-level parameters Array
 */
 
typedef enum
{
    DEVICE_EN = (0x0a00u + 0u),
    N_SAMPLE      ,
    SAMPLE_TIME   ,
    MASTER_MASK   ,
    RX_WAVE_OFFSET,
    MAX_ITERATION ,
    SEQ_ITERATION ,
    DELAY_MS      ,
    SOUND_C       ,
    BAUDRATE      ,
    DEBUG         
} eSoftwareParamID;

#define __N_SoftwareParamID ((DEBUG - DEVICE_EN)+1)
const eParameterElement C_SOFT_PARAM[__N_SoftwareParamID] PROGMEM = {  //Variable and could be edited
    { DEVICE_EN         , 0*(16), 16,       1 , "DEVICE_EN"       ,"Sensor Enable mask"         , C_LUT_NULL},
    { N_SAMPLE          , 1*(16), 16,     512 , "N_SAMPLE"        ,"Number of Samples per frame", C_LUT_NULL},
    { SAMPLE_TIME       , 2*(16), 16,      10 , "SAMPLE_TIME"     ,"Sample Time in (us)"        , C_LUT_NULL},
    { MASTER_MASK       , 3*(16), 16,       1 , "MASTER_MASK"     ,"Master Sensors mask"        , C_LUT_NULL},
    { RX_WAVE_OFFSET    , 4*(16), 16,       2 , "RX_WAVE_OFFSET"  ,"RX Wave offset"             , C_LUT_NULL},
    { MAX_ITERATION     , 5*(16), 16,0xffffffff,"MAX_ITERATION"   ,"Maximum Iterations"         , C_LUT_NULL},
    { SEQ_ITERATION     , 6*(16), 16,      32 ,"SEQ_ITERATION"   ,"Sequence Iterations"        , C_LUT_NULL},
    { DELAY_MS          , 7*(16), 16,     100 , "DELAY_MS"        ,"Delay inter samples in (ms)", C_LUT_NULL},
    { SOUND_C           , 8*(16), 16,     343 , "SOUND_C"         ,"Sound velocity constant C"  , C_LUT_NULL}, 
    { BAUDRATE          , 9*(16), 16,  115200 , "BAUDRATE"        ,"Serial Com Baudrate"        , C_LUT_NULL}, 
    { DEBUG             ,10*(16), 16,       0 , "DEBUG"           ,"Debug depth"                , C_LUT_NULL},
};


/**************************
* Parameters Table
*/
typedef enum
{
     // MSB: is the Register ID, LSB is the field ID
    NULL_TABLE      = 0x0000u,
    THRESHOLD_A     = 0x0100u,  // DONE
    MEASUREMENT     = 0x0200u,  // DONE
    CALIBRATION     = 0x0300u,  // DONE
    THRESHOLD_B     = 0x0400u,  // DONE
    SND_REC_STATUS  = 0x0500u,  // DONE
    STATUS          = 0x0600u,  // DONE
    EEPROM          = 0x0700u,  // DONE
    TEMPERATURE     = 0x0800u,  // DONE
    NFD_STATUS      = 0x0900u,  // DONE
    SOFT_PARAM      = 0x0a00u   // DONE
} eParameterTableID;

#define __N_ParamTableID  (SOFT_PARAM-NULL_TABLE+1)
const eParamaterTable C_PARAM_TABLES[__N_ParamTableID] PROGMEM = {
    /*
     eParameterTableID        id;
     const eParameterElement  *ptable;
     const char               *pname;
     uint32_t                 size;
     unsigned char            readCmdId;
     unsigned char            writeCmdId;
     int                      regNBit;  
  */
    {NULL_TABLE       , NULL               , ""                , 0                   , CMD_NOP             , CMD_NOP           ,   0 },
    {THRESHOLD_A      , C_THRESHOLD_A      , "THRESHOLD_A"     , __N_ThresholdAID    , CMD_THRES_A_READ    , CMD_THRES_A_WRITE ,  80 },
    {MEASUREMENT      , C_MEASUREMENT      , "MEASUREMENT"     , __N_MeasurementID   , CMD_MEAS_READ       , CMD_MEAS_WRITE    ,  73 },
    {CALIBRATION      , C_CALIBRATION      , "CALIBRATION"     , __N_CalibrationID   , CMD_CALIB_READ      , CMD_CALIB_WRITE   ,  74 },
    {THRESHOLD_B      , C_THRESHOLD_B      , "THRESHOLD_B"     , __N_ThresholdBID    , CMD_THRES_B_READ    , CMD_THRES_B_WRITE ,  80 },
    {STATUS           , C_STATUS           , "STATUS"          , __N_StatusID        , CMD_READ_STATUS     , CMD_NOP           ,  17 },
    {SND_REC_STATUS   , C_SND_REC_STATUS   , "SND_REC_STATUS"  , __N_SndRecStatusID  , CMD_SEND_A          , CMD_NOP           ,  19 },
    {EEPROM           , C_EEPROM           , "EEPROM"          , __N_EepromID        , CMD_EE_READ         , CMD_NOP           , 225 },
    {TEMPERATURE      , C_TEMPERATURE      , "TEMPERATURE"     , __N_TemperatureID   , CMD_READ_TEMP       , CMD_NOP           ,  10 },
    {NFD_STATUS       , C_NFD_STATUS       , "NFD_STATUS"      , __N_NfdStatusID     , CMD_READ_NFD_STATUS , CMD_NOP           ,  24 },
    {SOFT_PARAM       , C_SOFT_PARAM       , "SOFT_PARAM"      , __N_SoftwareParamID , CMD_NOP             , CMD_NOP           ,   0 }
};




#endif // _TDK_USSM_CONFIG_H

/************************ (C) COPYRIGHT TDK Electronics *****END OF FILE****/



