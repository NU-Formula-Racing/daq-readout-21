#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

#include "RTClib.h"
#include "virtualTimer.h"
#include "can_interface.h"
#include "virtualTimer.h"


RTC_PCF8523 rtc;

// FAILURE MODES:
//  Is data all changing? If not, sensor could be broken
//  Is data not being sent from a specific sensor? If not, could be broken



// Goals:
//   Read HP CAN
//     Time, Motor temp, Inverter temp, Coolant temp, Remaining battery, 
//     Battery temp, Power output, Throttle, HV Battery voltage
//   Read LP CAN
//     Time (RTC), Wheel Speed (one for each wheel), Break temp (for each break),
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
#include "esp_can.h"
// The tx and rx pins are constructor arguments to ESPCan, which default to TX = 5, RX = 4
ESPCAN can_bus{GPIO_NUM_17, GPIO_NUM_16};
#endif

const int kFLCANFrameAddress = 0x400;
const int kFRCANFrameAddress = 0x401;
const int kBLCANFrameAddress = 0x402;
const int kBRCANFrameAddress = 0x403;

CANSignal<float, 0, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), false> wheel_speed_signal{}; 
CANSignal<float, 16, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), false> brake_temp_signal{};
CANRXMessage<2> rx_message_wheel{can_bus, kFLCANFrameAddress, wheel_speed_signal, brake_temp_signal};

const int kGPS_CAN = 0x430;
const int kACCEL_CAN = 0x431;
const int kGYRO_CAN = 0x432;
const int kGYRO_2_CAN = 0x432;

CANSignal<float, 0, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), false> accel_x{}; 
CANSignal<float, 16, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), false> accel_y{}; 
CANSignal<float, 32, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), false> accel_z{};
CANRXMessage<3> rx_message_accel{can_bus, kACCEL_CAN, accel_x, accel_y, accel_z};

CANSignal<float, 0, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), false> gyro_x{}; 
CANSignal<float, 16, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), false> gyro_y{}; 
CANSignal<float, 32, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), false> gyro_z{}; 
CANRXMessage<3> rx_message_gyro{can_bus, kGYRO_CAN, gyro_x, gyro_y, gyro_z};

CANSignal<float, 0, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), true> lon_signal{}; 
CANSignal<float, 16, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), true> lat_signal{}; 
CANRXMessage<2> rx_message_pos{can_bus, kGPS_CAN, lon_signal, lat_signal};

const int CSpin = 5;

File sensorData;
String fileName;
String dataString ="";

VirtualTimerGroup timer_group;

void saveData(){
  // reopen file
  sensorData = SD.open(fileName, FILE_WRITE);
  if (sensorData){
    // print line into csv
    sensorData.println(dataString);
    sensorData.close();
    Serial.println(dataString);
  } else {
    Serial.println("Error saving values to file !");
  }
}

void sensor10ms(){
  DateTime now = rtc.now();

  dataString = dataString + "\n" + now.timestamp() + ", , " + float(accel_x) + ", " + float(accel_y) + ", " + float(accel_z) + ", , , , , ";
  saveData();
}

void sensor100ms(){
  DateTime now = rtc.now();

  dataString = dataString + "\n" + now.timestamp() + float(wheel_speed_signal) + ", , " + float(accel_x) + ", " + float(accel_y) + ", " + float(accel_z) + ", " + float(gyro_x) + ", " + float(gyro_y) + ", " + float(gyro_z) + ", " + float(lon_signal) + ", " + float(lat_signal);
  saveData();
}

void sensor1000ms(){
  DateTime now = rtc.now();

  dataString = dataString + "\n" + now.timestamp() + float(wheel_speed_signal) + ", " + float(brake_temp_signal) + ", " + float(accel_x) + ", " + float(accel_y) + ", " + float(accel_z) + ", " + float(gyro_x) + ", " + float(gyro_y) + ", " + float(gyro_z) + ", " + float(lon_signal) + ", " + float(lat_signal);
  saveData();
}

void setup(void){
  Serial.begin(9600);

  Serial.print("Initializing CAN...");
  can_bus.Initialize(ICAN::BaudRate::kBaud1M);
  Serial.print("CAN Initialized");

  Serial.print("Initializing timers...");
  timer_group.AddTimer(10U, sensor10ms);
  timer_group.AddTimer(100U, sensor100ms);
  timer_group.AddTimer(1000U, sensor1000ms);
  Serial.print("Timers Initialized");

  Serial.print("Initializing RTC...");

  #ifndef ESP8266
    while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (! rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  rtc.start();
  DateTime now = rtc.now();

  float drift = 43; // seconds plus or minus over oservation period - set to 0 to cancel previous calibration.
  float period_sec = (7 * 86400);  // total obsevation period in seconds (86400 = seconds in 1 day:  7 days = (7 * 86400) seconds )
  float deviation_ppm = (drift / period_sec * 1000000); //  deviation in parts per million (μs)
  float drift_unit = 4.34; // use with offset mode PCF8523_TwoHours
  // float drift_unit = 4.069; //For corrections every min the drift_unit is 4.069 ppm (use with offset mode PCF8523_OneMinute)
  int offset = round(deviation_ppm / drift_unit);
  // rtc.calibrate(PCF8523_TwoHours, offset); // Un-comment to perform calibration once drift (seconds) and observation period (seconds) are correct
  // rtc.calibrate(PCF8523_TwoHours, 0); // Un-comment to cancel previous calibration

  Serial.print("Offset is "); Serial.println(offset); // Print to control offset

  Serial.print("Initializing SD card...");

  if (!SD.begin(CSpin)) {
    Serial.println("Card failed/not found.");
  } else{
    Serial.print("Card Initialized.");

    // New file name not working yet, does work with /test.txt
    fileName = "/test-" + now.timestamp() + ".csv";

    sensorData = SD.open(fileName, FILE_WRITE);

    if (sensorData){
      dataString = "Timestamp, Wheel_Speed, Brake_Temp, Accel_x, Accel_y, Accel_z, Gyro_x, Gyro_y, Gyro_z, Longitude, Latitude";
      saveData();
    }

    sensorData.close();
  }
}

void loop() {

  can_bus.Tick();
  timer_group.Tick(millis());

}



