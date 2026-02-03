= TDK_USSM ultrasonic sensor Library for Arduino =
==========
TDK_USSM is an [Arduino](http://arduino.cc) library for TDK Ultrasonic Sensor Modules (USSM)


Documentation
-------------
Documentation for the library will be released soon on [TDK website](tdk.com)

HARDWARE SETUP:
  - TDK Ultrasonic Sensor has 3 pins:
    1- VS : to be connected to a power supply in the range of 7V to 18V. (Default 12V or 9V)
    2- IO : Is a bidirectional IO with signal range in 0 to VS
    3- GND: Ground signal shall be same as Arduino GND
    
  - Using Different Hardware:
    1- Driver which could be:
      - N-MOS : to operate as a Low-Side switch:
        + Gate  to Arduino Trigger Pin
        + Drain to IO Line.
        + Source to GND
        => Configuration: TdkUssm(TriggerPin, EchoPin, N_SENSORS, HIGH, LOW, INPUT_PULLUP)
      - LIN Transceiver :
        + Vsup same as Sensor VS
        + GND same as Sensor & Arduino GND
        + TX to Arduino Trigger Pin
        => Configuration: TdkUssm(TriggerPin, EchoPin, N_SENSORS, LOW, HOGH, INPUT_PULLUP) // Default
    2- Receiver which could be: 
      - Voltage adapater using Simple resistor-divider:
        + Upper-end to IO Line
        + Lower-end to GND
        + Mid-point to Arduino Echo Pin insuring that max voltage is within Arduino voltage range (5V or 3V3 depending of hardware version)  
        => Configuration: rxPinMode=INPUT 
      - Zenner diode + Resistor
        + Upper-end (Zenner Kathode) to IO Line
        + Lower-end (Resistor) to GND
        + Mid-point to Arduino Echo Pin insuring that max voltage is within Arduino voltage range (5V or 3V3 depending of hardware version)
        => Configuration: rxPinMode=INPUT 
      - LIN Transceiver :
        + RX to Arduino Echo Pin
        => Configuration: rxPinMode=INPUT_PULLUP // Default
            
 Note:    
  - IO Line requires a Pull-up to VS. Some of sensors have it inside, but some others variants do not have it.
  - If your sensor does not have an internal pull-up, you need to add an external 6.8KOhm pull-up resistor between IO and VS.


[basic example](examples/TDK_USSM_multiple_cm/TDK_USSM_multiple_cm.ino)

```Arduino
#include <TDK_USSM.h>

//-- Instanciate Single Sensor
TDK_USSM TdkUssm(3); //Initialize Sensor Pins (IoPin)

//-- Setup 
void setup()
{ 
  Serial.begin(115200); 
}

//-- Runtime.
void loop()
{ 
  while(1){
    Serial.println( TdkUssm.GetDistanceInch() );  // Prints Distance in inch.
    delay(200); 
  }
}
```
> The library supports multiple sensors. Here after is a simple example   

```Arduino
#include <TDK_USSM.h>

//-- Sensors Pinmap Sensors
#define N_SENSORS  2
const int TriggerPin[N_SENSORS] = {3, 4}; // Trigger Pin of Ultrasonic Sensor
const int EchoPin[N_SENSORS]    = {5, 6}; // Echo Pin of Ultrasonic Sensor. 
// Note: Echo Pins could be same as Trigger pin if a bidir level shifter is used.

TDK_USSM TdkUssm(TriggerPin,EchoPin); //Initialize Sensor Pins (TxPins , RxPins)

//-- Setup 
void setup()
{ 
  Serial.begin(115200); 
}

//-- Runtime.
void loop()
{ 
  while(1)
  {
    for(int i=0; i<N_SENSORS) 
    {
      Serial.print( TdkUssm.GetDistanceCm(i) );  // Prints Distance in cm of sensor i.
      Serial.print(" ");
    }
    Serial.println();  // Terminate single capture line
    delay(200); 
  }
}
```

Download
--------
Coming soon on [TDK](tdk.com) website.


Install
-------
The library can be installed using the [standard Arduino library install procedure](http://arduino.cc/en/Guide/Libraries)  

