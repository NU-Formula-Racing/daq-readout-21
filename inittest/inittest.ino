
/*
Sensor Multitasking
- checking for multiple sensors at different time frames
- uses millis and is ugly asf bc of if statements but it works
- may b shitty but no 3rd party lieberry = simulator friendly: 
https://www.tinkercad.com/things/kvzXWspMo0g-stunning-snaget-duup/editel?sharecode=lyl-Zan2RpXMJFdnIAeN28VRvoHp2dthZG_x7TVTIyk
*/

/* TODO
 *  pull data from I2C bus
 *  pull data from MOTEC/CAN 
 *  write functions for manipulating data:
 *    Non testing:
 *      lambdas, wheel speed (DIG) , brake temp, shock potent, brake pressure, throttle pos
 *      intake manifold ap, intake manifold at, oil pressure, cam angle (DIG)
 *      coolant temp, oil temp, crank angle
 *    Testing: 
 *      steering angle, pitot tubes, strain gauges, accelerometer, gyroscope, thermocouple
 */
// test a: 1ms digital loop + 1000ms analog loop with all float type
//  test b: 1ms digital loop + 1000ms analog loop with all float type
//  test c: 1ms digital loop + 1000ms analog loop with all unsigned long type
//  test d: constant digital loop + 1000ms analog loop with all float type
//  test e: constant digital loop + 1000ms analog loop with all unsigned long type

#include "SD.h"
#include"SPI.h"

//GLOBALS
const int CSpin = 53;
String dataString =""; // holds the data to be written to the SD card
String fileName = "testA.csv";
File sensorData;
float digitalReading = 0;
int lastDigReadTime = 0;
float analogReading1 = 0;
float analogReading2 = 0;
unsigned long previousTimeDigital = millis();
unsigned long previousTimeAnalog = millis();
int diff = 0;
       
//FUNCTIONS

// fake digital sensor
void digitalSensor(){
  digitalReading = digitalReading + 1;
//  Serial.print("Digital Output:");
//  Serial.println(digitalReading);
}
// fake analog sensors
void analogSensor1(){
  analogReading1 = analogReading1 + 1;
//  Serial.print("Analog 1 Output:");
//  Serial.println(analogReading1);
}
void analogSensor2(){
  analogReading2 = (analogReading2+1)*2;
//  Serial.print("Analog 2 Output:");
//  Serial.println(analogReading2);
}


// saves to sd card
void saveData(){
  sensorData = SD.open(fileName, FILE_WRITE);
//  if (SD.exists(fileName)) { // check the card is still there
//  // now append new data file
    if (sensorData){
      sensorData.println(dataString);
      sensorData.close(); // close the file
      Serial.println(dataString);
    }
//  } else {
//    Serial.println("Error writing to file !");
//  }
}

// pulls all analog values and compiles into CSV string
void compileCurData(){
  analogSensor1();
  analogSensor2();
  // convert to CSV
  dataString = String(digitalReading) + "," + String(analogReading1) + "," + String(analogReading2);
  
}


void setup() {
  // Open serial communications
  Serial.begin(9600);
  Serial.print("Initializing SD card...");
  pinMode(CSpin, OUTPUT);
  //
  // see if the card is present and can be initialized
  if (!SD.begin(CSpin)) {
  Serial.println("Card failed/not found");
  // stop
  return;
  }
  Serial.println("card initialized.");
  sensorData = SD.open(fileName, FILE_WRITE);
  sensorData.println("Digital Reading, Analog Reading 1, Analog Reading 2");
  sensorData.close();
}


void loop() {
  unsigned long currentTime = millis();
  // this is kinda ugly but i think it will do the job lul 
  
  // run checks for digital sensors every single loop, check for reading of 0
  if (currentTime < 10000){
    
  if (currentTime - previousTimeDigital > 1){
    digitalSensor();
  }
  

  // check for analog reading every second
  if (currentTime - previousTimeAnalog > 1000){
    previousTimeAnalog = currentTime;
    compileCurData();
    saveData();
  }
  }
  
}
