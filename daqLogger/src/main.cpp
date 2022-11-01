#include <Arduino.h>

#include <RTClib.h>
#include "virtualTimer.h"
#include "can_interface.h"
//#include "virtualTimer.h"

// Goals:
//   Read HP CAN
//     Time, Motor temp, Inverter temp, Coolant temp, Remaining battery, 
//     Battery temp, Power output, Throttle, HV Battery voltage
//   Read LP CAN
//     Time (RTC ?), Wheel Speed (one for each wheel), Break temp (for each break),
//     Break pressure (for front/back breaks), LV Battery voltage
//   Store information to SD Card
//     Use CSV
//     Log every 10 ms:
//       RTC, Accelerometer
//     Log every 100 ms:
//       Wheel Speed, GPS, Suspension
//     Log every 1000 ms:
//       Ambient temp, Brake temp, LV Battery voltage, HV Battery voltage

// Initialize canbus based on teensy board
#if defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41)
#include "teensy_can.h"
// The bus number is a template argument for Teensy: TeensyCAN<bus_num>
TeensyCAN<1> can_bus{};
#endif

#ifdef ARDUINO_ARCH_ESP32
//#include "esp_can.h"
// The tx and rx pins are constructor arguments to ESPCan, which default to TX = 5, RX = 4
//ESPCAN can_bus{};
#endif
// How do I differentiate HP vs LP???


// Read from CAN:
// Use CANRXMessage for fr, fl, br, bl wheels
//    When to use DecodeSignals?




// Store to SD
//    Create dict of 10ms, 100ms, 1000ms 
//       {10:[RTC, Acelerometer], 100:[Wheel Speed, GPS], 1000:[Temps, Battery Voltage]}
//    How dict operates
//       timer %10 = 0, store info from dict[10]; 
//       timer %100 = 0, store dict[100]; 
//       timer %1000 = 0, store dict[1000];
//    
#include "SD.h"
#include "SPI.h"

unsigned long previousTime10Analog = millis();
unsigned long previousTime100Analog = millis();
unsigned long previousTime1000Analog = millis();
/*
// pulls all analog values and compiles into CSV string
void compileCurData(delta){
  analogSensors();
  // convert to CSV
  dataString = "";
  if (delta == 10){
    dataString = 0;
    
  }
  if (delta == 100){
    dataString = 1;
  }
  if (delta == 1000){
    dataString = 10;
  }
  //for (int i = 0; i < sizeof(allSensors)/sizeof(float); i = i + 1) {
  //  dataString = dataString + String(allSensors[i]) + ",";
  //}
}*/

File sensorData;
String fileName;

void setup(void){
  Serial.begin(9600);
  Serial.print("Initializing SD card...");

  fileName = "test";
  sensorData = SD.open(fileName, FILE_WRITE);

  if (sensorData){
    sensorData.println("FL_VSS,FR_VSS,BL_VSS,BR_VSS,FL_BRK_TMP,FR_BRK_TMP,BL_BRK_TMP,BR_BRK_TMP,FL_SUS_POT,FR_SUS_POT,BL_SUS_POT,BR_SUS_POT,F_BRK_PRES,B_BRK_PRES,STEER_ANG,TPS,OIL_PRES,OIL_TEMP,COOL_TEMP,MAP,MAT,NEUT,LAMBDA1,LAMBDA2,ACCELX,ACCELY,ACCELZ,STRAIN1,STRAIN2,STRAIN3,STRAIN4,PTUBE1,PTUBE2,PTUBE3,PTUBE4,PTUBE5,PTUBE6,PTUBE7,PTUBE8,PTUBE9,PTUBE10,PTUBE11,PTUBE12,GYROX,GYROY,GYROZ,MAGNETX,MAGNETY,MAGNETZ,DAQTEMP");
    sensorData.close();
  }
}

void loop() {

  //digitalWrite(10, LOW);

  /*
  unsigned long currentTime = millis();
  
  // run checks for digital sensors every single loop, check for reading of 0
  if (currentTime < 10000) {
    digitalSensor();

    // check for analog reading every 10 ms
    if (currentTime - previousTime10Analog > 10) {
      previousTime10Analog = currentTime;
      compileCurData();
      saveData();
    }
    // check for analog reading every 100 ms
    if (currentTime - previousTime100Analog > 100) {
      previousTime100Analog = currentTime;
      compileCurData();
      saveData();
    }
    // check for analog reading every 1000 ms
    if (currentTime - previousTime10Analog > 1000) {
      previousTime1000Analog = currentTime;
      compileCurData();
      saveData();
    }
  }*/
}



