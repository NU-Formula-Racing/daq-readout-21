// #include <Arduino.h>
// #include <SD.h>
// #include <SPI.h>
// //#include <esp32-hal-gpio.h>

// #include "RTClib.h"
// #include "virtualTimer.h"
// #include "can_interface.h"
// #include "virtualTimer.h"
// #include "..\lib\logger.h"

// RTC_PCF8523 rtc;

// // Initialize canbus based on teensy board
// #if defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41)
// #include "teensy_can.h"
// // The bus number is a template argument for Teensy: TeensyCAN<bus_num>
// TeensyCAN<1> can_bus{};
// #endif

// #ifdef ARDUINO_ARCH_ESP32
// #include "esp_can.h"
// // The tx and rx pins are constructor arguments to ESPCan, which default to TX = 5, RX = 4; Currenlty tx=32, rx=27
// ESPCAN can_bus{GPIO_NUM_32, GPIO_NUM_27};
// #endif

// LoggerBoard::LoggerBoard()
// {
//     hp_lp = "";
//     dataString = "";
//     sensorData;
// };
