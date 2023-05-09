#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <esp32-hal-gpio.h>

#include "RTClib.h"
#include "virtualTimer.h"
#include "can_interface.h"
#include "virtualTimer.h"
#include "inverter_driver.h"
#include <string>

RTC_PCF8523 rtc;
VirtualTimerGroup timer_group;

// Initialize canbus based on teensy board
#if defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41)
#include "teensy_can.h"
// The bus number is a template argument for Teensy: TeensyCAN<bus_num>
TeensyCAN<1> can_bus{};
#endif

#ifdef ARDUINO_ARCH_ESP32
#include "esp_can.h"
// The tx and rx pins are constructor arguments to ESPCan, which default to TX = 5, RX = 4; Currenlty tx=32, rx=27
ESPCAN can_bus{10, GPIO_NUM_32, GPIO_NUM_27};
#endif

const bool isLP = true;

#if (isLP)
//
//
// START LP CAN
//
//

//
// LP CAN Addresses
//

const int kFL_CAN = 0x400;
const int kFR_CAN = 0x401;
const int kBL_CAN = 0x402;
const int kBR_CAN = 0x403;
const int kBrake_CAN = 0x410;
const int kGPS_CAN = 0x430;
const int kACCEL_CAN = 0x431;
const int kGYRO_CAN = 0x432;

//
// LP CAN Messages
//

// Front left wheel speed and temp
CANSignal<float, 0, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), false> FL_wheel_speed_signal{};
CANSignal<float, 16, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), false> FL_brake_temp_signal{};
CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0.1), false> FL_suspension_position_signal{};
CANRXMessage<2> rx_message_FLwheel{can_bus, kFL_CAN, FL_wheel_speed_signal, FL_brake_temp_signal};
CANTXMessage<2> tx_message_FLwheel{can_bus, kFL_CAN, 4, 100, timer_group, FL_wheel_speed_signal, FL_brake_temp_signal};

// Front right wheel speed and temp
CANSignal<float, 0, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), false> FR_wheel_speed_signal{};
CANSignal<float, 16, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), false> FR_brake_temp_signal{};
CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0.1), false> FR_suspension_position_signal{};
CANRXMessage<3> rx_message_FRwheel{can_bus, kFR_CAN, FR_wheel_speed_signal, FR_brake_temp_signal, FR_suspension_position_signal};

// Back left wheel speed and temp
CANSignal<float, 0, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), false> BL_wheel_speed_signal{};
CANSignal<float, 16, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), false> BL_brake_temp_signal{};
CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0.1), false> BL_suspension_position_signal{};
CANRXMessage<3> rx_message_BLwheel{can_bus, kBL_CAN, BL_wheel_speed_signal, BL_brake_temp_signal, BL_suspension_position_signal};

// Back right wheel speed and temp
CANSignal<float, 0, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), false> BR_wheel_speed_signal{};
CANSignal<float, 16, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), false> BR_brake_temp_signal{};
CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0.1), false> BR_suspension_position_signal{};
CANRXMessage<3> rx_message_BRwheel{can_bus, kBR_CAN, BR_wheel_speed_signal, BR_brake_temp_signal, BR_suspension_position_signal};

// Brake Pressure
CANSignal<float, 0, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0.1), true> F_Brake_pressure{};
CANSignal<float, 16, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0.1), true> R_Brake_pressure{};
CANRXMessage<2> rx_message_brake{can_bus, kBrake_CAN, F_Brake_pressure, R_Brake_pressure};

// // Acceleration
CANSignal<float, 0, 16, CANTemplateConvertFloat(0.0005), CANTemplateConvertFloat(0), false> accel_x{};
CANSignal<float, 16, 16, CANTemplateConvertFloat(0.0005), CANTemplateConvertFloat(0), false> accel_y{};
CANSignal<float, 32, 16, CANTemplateConvertFloat(0.0005), CANTemplateConvertFloat(0), false> accel_z{};
CANRXMessage<3> rx_message_accel{can_bus, kACCEL_CAN, accel_x, accel_y, accel_z};

// Gyro
CANSignal<float, 0, 16, CANTemplateConvertFloat(0.0005), CANTemplateConvertFloat(0), false> gyro_x{};
CANSignal<float, 16, 16, CANTemplateConvertFloat(0.0005), CANTemplateConvertFloat(0), false> gyro_y{};
CANSignal<float, 32, 16, CANTemplateConvertFloat(0.0005), CANTemplateConvertFloat(0), false> gyro_z{};
CANRXMessage<3> rx_message_gyro{can_bus, kGYRO_CAN, gyro_x, gyro_y, gyro_z};

// GPS
CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true> lon_signal{};
CANSignal<float, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), true> lat_signal{};
CANRXMessage<2> rx_message_gps{can_bus, kGPS_CAN, lon_signal, lat_signal};

// BMS Cell V Summary

//
//
// END LP CAN
//
//
#else
//
//
// START HP CAN
//
//

//
// HP CAN Addresses
//

const int kTemp_CAN = 0x420;
const int kThrottle_CAN = 0x300;

//
// HP CAN Messages
//

// Temp
CANSignal<float, 0, 16, CANTemplateConvertFloat(0.01), CANTemplateConvertFloat(0), false> coolant_temp_signal{};
CANSignal<float, 32, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), false> ambient_temp_signal{};
CANRXMessage<2> rx_message_temp{can_bus, kTemp_CAN, coolant_temp_signal, ambient_temp_signal};

// BMS

// Throttle
CANSignal<uint8_t, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> cur_throttle_signal{};
CANRXMessage<1> rx_message_throttle{can_bus, kThrottle_CAN, cur_throttle_signal};

//
//
// END HP CAN
//
//
#endif


const int CSpin = 5;
const int new_SDA = 21;
const int new_SCL = 22;

File sensorData;
String fileName;
String dataString = "";

Inverter inverter{can_bus};


//
// Write to SD on different time frames
//

void saveData()
{
  // reopen file
  sensorData = SD.open(fileName, FILE_APPEND);
  if (sensorData)
  {
    // print line into csv
    sensorData.println(dataString);
    Serial.println(dataString);
    sensorData.close();
  }
  else
  {
    Serial.println("Error saving values to file !");
  }
}

// Make these iteratively
void sensor10ms()
{
  DateTime now = rtc.now();
  String placeholder = "";

  #if (isLP)
  //                            Timestamp                     FL Speed                              FR Speed                             BL Speed                                BR Speed           Brake Temp, Brake Pressure   Accel_x             Accel_y                 Accel_z                Gyro_x                  Gyro_y                 Gyro_z      Long, Lat
  dataString = placeholder + now.timestamp() + ", " + float(FL_wheel_speed_signal) + ", " + float(FR_wheel_speed_signal) + ", " + float(BL_wheel_speed_signal) + ", " + float(BR_wheel_speed_signal) + ", , , , , , , " + float(accel_x) + ", " + float(accel_y) + ", " + float(accel_z) + ", " + float(gyro_x) + ", " + float(gyro_y) + ", " + float(gyro_z) + ", , ";

  #else
  //                            Timestamp                   RPM                          Inverter Temp                              Motor Temp       Coolant, Ambient Temp       Throttle
  dataString = placeholder + now.timestamp() + ", " + inverter.GetRPM() + ", " + inverter.GetInverterTemperature() + ", " + inverter.GetMotorTemperature() + ", , , " + float(cur_throttle_signal);
  
  #endif
  saveData();
}

void sensor100ms()
{
  // NEED TO FINISH
  //
  DateTime now = rtc.now();
  String placeholder = "";

  #if (isLP)
  //                            Timestamp                     FL Speed                              FR Speed                             BL Speed                                BR Speed           Brake Temp, Brake Pressure   Accel_x             Accel_y                 Accel_z                Gyro_x                  Gyro_y                 Gyro_z      Long, Lat
  dataString = placeholder + now.timestamp() + ", " + float(FL_wheel_speed_signal) + ", " + float(FR_wheel_speed_signal) + ", " + float(BL_wheel_speed_signal) + ", " + float(BR_wheel_speed_signal) + ", , , , , , , " + float(accel_x) + ", " + float(accel_y) + ", " + float(accel_z) + ", " + float(gyro_x) + ", " + float(gyro_y) + ", " + float(gyro_z) + ", , ";
  
  #else
  //                            Timestamp                   RPM                          Inverter Temp                              Motor Temp       Coolant, Ambient Temp       Throttle
  dataString = placeholder + now.timestamp() + ", " + inverter.GetRPM() + ", " + inverter.GetInverterTemperature() + ", " + inverter.GetMotorTemperature() + ", , , " + float(cur_throttle_signal);
  
  #endif

  saveData();
}

void sensor1000ms()
{
  DateTime now = rtc.now();
  String placeholder = "";

  #if (isLP)
  //                            Timestamp                     FL Speed                              FR Speed                             BL Speed                                BR Speed                           FL Brake Temp                        FR Brake Temp                       BL Brake Temp                       BR Brake Temp                      F Brake Pressure                B Brake Pressure                    Accel_x                Accel_y                 Accel_z                Gyro_x                  Gyro_y                 Gyro_z                  Longitude                 Latitude
  dataString = placeholder + now.timestamp() + ", " + float(FL_wheel_speed_signal) + ", " + float(FR_wheel_speed_signal) + ", " + float(BL_wheel_speed_signal) + ", " + float(BR_wheel_speed_signal) + ", " + float(FL_brake_temp_signal) + ", " + float(FR_brake_temp_signal) + ", " + float(BL_brake_temp_signal) + ", " + float(BR_brake_temp_signal) + ", " + float(F_Brake_pressure) + ", " + float(R_Brake_pressure) + ", " + float(accel_x) + ", " + float(accel_y) + ", " + float(accel_z) + ", " + float(gyro_x) + ", " + float(gyro_y) + ", " + float(gyro_z) + ", " + float(lon_signal) + ", " + float(lat_signal);
  
  #else
  //                            Timestamp                   RPM                          Inverter Temp                              Motor Temp                          Coolant Temp                         Ambient Temp                        Throttle
  dataString = placeholder + now.timestamp() + ", " + inverter.GetRPM() + ", " + inverter.GetInverterTemperature() + ", " + inverter.GetMotorTemperature() + "," + float(coolant_temp_signal) + "," + float(ambient_temp_signal) + ", " + float(cur_throttle_signal);
  
  #endif

  saveData();
}


//
// Start the Setup
//

DateTime init_RTC()
{
#ifndef ESP8266
  while (!Serial)
    ; // wait for serial port to connect. Needed for native USB
#endif

  Wire.begin(new_SDA, new_SCL);

  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1)
      delay(10);
  }

  if (!rtc.initialized() || rtc.lostPower())
  {
    Serial.println("RTC is NOT initialized, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  rtc.start();
  // Turn on if RTC drifts too far
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  DateTime now = rtc.now();

  float drift = -10;                                    // seconds plus or minus over oservation period - set to 0 to cancel previous calibration.
  float period_sec = (7 * 86400);                       // total obsevation period in seconds (86400 = seconds in 1 day:  7 days = (7 * 86400) seconds )
  float deviation_ppm = (drift / period_sec * 1000000); //  deviation in parts per million (μs)
  float drift_unit = 4.34;                              // use with offset mode PCF8523_TwoHours
  // float drift_unit = 4.069; //For corrections every min the drift_unit is 4.069 ppm (use with offset mode PCF8523_OneMinute)
  int offset = round(deviation_ppm / drift_unit);
  rtc.calibrate(PCF8523_TwoHours, offset); // Un-comment to perform calibration once drift (seconds) and observation period (seconds) are correct
  // rtc.calibrate(PCF8523_TwoHours, 0); // Un-comment to cancel previous calibration

  Serial.print("Offset is ");
  Serial.println(offset); // Print to control offset

  return now;
}

void init_SD(DateTime now)
{
  if (!SD.begin(CSpin))
  {
    Serial.println("Card failed/not found.");
  }
  else
  {
    Serial.print("Card Initialized.");

    // New file name not working yet, does work with /test.txt
    fileName = "/test-" + String(now.month()) + "-" + String(now.day()) + "=" + String(now.hour()) + "-" + String(now.minute()) + ".csv";
    sensorData = SD.open(fileName, FILE_WRITE);
    if (isLP)
    {
      dataString = "Timestamp, FL Speed, FR Speed, BL Speed, BR Speed, FL Brake Temp, FR Brake Temp, BL Brake Temp, BR Brake Temp, F Brake Pressure, B Brake Pressure, Accel_x, Accel_y, Accel_z, Gyro_x, Gyro_y, Gyro_z, Longitude, Latitude";
    }
    else
    {
      dataString = "Timestamp, RPM, Inverter Temp, Motor Temp, Coolant Temp, Ambient Temp, Throttle";
    }

    saveData();
  }
}

void setup(void)
{
  Serial.begin(9600);

  pinMode(GPIO_NUM_26, OUTPUT);
  digitalWrite(GPIO_NUM_26, LOW);
  pinMode(GPIO_NUM_33, OUTPUT);
  digitalWrite(GPIO_NUM_33, LOW);

  Serial.print("Initializing CAN...");
  can_bus.Initialize(ICAN::BaudRate::kBaud1M);

  #if (isLP)
  //
  // LP CAN Messages
  //
  can_bus.RegisterRXMessage(rx_message_FLwheel);
  can_bus.RegisterRXMessage(rx_message_FRwheel);
  can_bus.RegisterRXMessage(rx_message_BLwheel);
  can_bus.RegisterRXMessage(rx_message_BRwheel);
  can_bus.RegisterRXMessage(rx_message_brake);
  can_bus.RegisterRXMessage(rx_message_accel);
  can_bus.RegisterRXMessage(rx_message_gyro);
  can_bus.RegisterRXMessage(rx_message_gps);

  #else
  //
  // HP CAN Messages
  //
  can_bus.RegisterRXMessage(rx_message_temp);
  can_bus.RegisterRXMessage(rx_message_throttle);
  
  #endif

  Serial.print("CAN Initialized");

  #if (isLP)
  Serial.print("No inverter for LP")
  
  #else
  Serial.print("Initializing Inverter...");
  inverter.RequestMotorTemperature(100);
  inverter.RequestPowerStageTemp(100);
  inverter.RequestRPM(100);
  Serial.print("Inverter Initialized");
  
  #endif

  Serial.print("Initializing RTC...");
  DateTime now = init_RTC();
  Serial.print("RTC Initialized");

  Serial.print("Initializing SD card...");
  init_SD(now);
  Serial.print("SD Card Initialized");

  Serial.print("Initializing Timers...");
  timer_group.AddTimer(10U, sensor10ms);
  timer_group.AddTimer(100U, sensor100ms);
  timer_group.AddTimer(1000, sensor1000ms);
  Serial.print("Timers Initialized");
}

void loop()
{
  can_bus.Tick();
  timer_group.Tick(millis());
}
