
/* Telemetry testing code for 3/6/2021
 *  subroutines: analogSensors, telemetry
 *  unused subroutines: digitalSensors, saveAndCompile
 *  
 *  GOAL: record FL brake temp, FR brake temp, and front brake pressure
 */

//#include "SD.h"
//#include"SPI.h"
//#include <Wire.h>
//#include <Adafruit_ADS1015.h>
//#include <SoftwareSerial.h>

SoftwareSerial xbee(0,1);

//GLOBALS
//String dataString =""; // holds the data to be written to the SD card
//String fileName;
//File sensorData;


unsigned long previousTimeDigital = millis();
unsigned long previousTimeAnalog = millis();
unsigned long currentTime;

// SENSOR ARRAYS
float allSensors[3];

// SENSOR GLOBALS
int sensorVoltage = 0;
int systemVoltage = 5;
int resolution = 65535;

// Brake Temperature
int FL_BRK_TMP_PIN = 97;
int FR_BRK_TMP_PIN = 96;
float FL_BRK_TMP, FR_BRK_TMP;

// Brake Pressure
int F_BRK_PRES_PIN = 93;
float F_BRK_PRES = 0;

// OFFSETS
float F_BRK_PRES_CLB;
                

int convertSensor(int sensorValue, int calibration=0);
// sensor value from 0 to 2^16 and returns a voltage between 0 and 5 V
int convertSensor(int sensorValue, int calibration=0){
  sensorVoltage = (sensorValue * (systemVoltage/resolution)) - calibration;
  return sensorVoltage;
}


void setup() {
  // Open serial communications
  Serial.begin(9600);
  //  Serial.print("Initializing SD card...");
  xbee.begin(9600);

  F_BRK_PRES_CLB = convertSensor(analogRead(F_BRK_PRES_PIN));

}


void loop() {
  currentTime = millis();
  
//  run checks for digital sensors every single loop, check for reading of 0
//  digitalSensors();
//  check for analog reading every second
  if (currentTime - previousTimeAnalog > 1000){
    previousTimeAnalog = currentTime;
//    compileCurData();
//    saveData();
    // for now write to xbee every second, may shorten interval
    writeXbee();
  }
}