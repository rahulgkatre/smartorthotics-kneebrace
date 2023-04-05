// Note:
// Arduino compiles the files in this project as follows
// Concatenate KneeBrace.ino + BNO08X.ino + ... + Webserver.ino
// So after KneeBrace.ino (the .ino with same name as directory), it is in alphabetical order
// Then compile the concatenated .ino normally
// Had to put all the includes and defines in this file to avoid errors
// Setup and loop can be put in here because it just calls functions defined in other files
// If a function requires access to variables from other files, it must be further down in alphabetical ordering
// For example, functions in Webserver.ino can access variables of BNO08X.ino and Webpage.ino, but not the other way around

// Inludes and defines for BNO08X.ino
#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <I2C.h>
#include <Wire.h>
#include <rtos.h>
// #include "rp2040/utils.h"

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9

// Pins for I2C mode
#define BNO08X_SDA 2
#define BNO08X_SCL 3

// Print Out
#define PRINT true

// No Reset
#define BNO08X_RESET -1

// Analysis
#define WINDOW_SIZE 10

// Includes and defines for Filesystem.ino
// Uncomment if working with Wifi-enabled feather
// #include <FS.h>
// #include <LittleFS.h>

// #define FORMAT_LITTLEFS_IF_FAILED false

// Includes and defines for Network.ino
// #include <WiFi.h>

// Includes and defines for Webpage.ino
// None at this time

// Includes and defines for Webserver.ino
// #include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>

// Includes and defines for GaitAnalysis.ino

void setup()
{
    // Serial port for debugging purposes
    Serial.begin(115200);
    bno08XSetup();

    // Uncomment if working with Wifi-enabled feather
    // if (!fileSystemSetup()) {
    //     return;
    // }

    // networkSetup();
    // webServerSetup();
}

void loop() {
    // webServerLoop();
    bno08XLoop();
}
