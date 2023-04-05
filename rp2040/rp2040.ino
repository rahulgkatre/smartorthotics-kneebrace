#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <I2C.h>
#include <Wire.h>
#include <rtos.h>
#include "utils.h"

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9

// Pins for I2C mode
#define BNO08X_SDA 2
#define BNO08X_SCL 3

// Sampling stuff
#define SAMPLING_RATE 100

// Print Out
#define PRINT true

#define BNO08X_RESET -1

euler_t ypr;
accel_t acc;
gyro_t gyro;

rtos::Thread updater;
long updater_sleep_time_millis = (long)(1.0 / SAMPLING_RATE * 1000);

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

  // Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void update() {
  while (true) {
    
    unsigned long now = millis();
    
    if (bno08x.getSensorEvent(&sensorValue)) {
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, false);
        case SH2_LINEAR_ACCELERATION:
          getAccel(&sensorValue.un.linearAcceleration, &acc);
        case SH2_GYROSCOPE_CALIBRATED:
          getGyro(&sensorValue.un.gyroscope, &gyro);
          break;
      }
    }
    
    if (PRINT) {
      // Serial.print("\t"); Serial.print(ypr.yaw/180);
      Serial.print("\t"); Serial.print(abs(gyro.y)); // Rotational acceleration on y-axis
      Serial.print("\t"); Serial.println(magnitude_acc(&acc, true));
    }

    uint32_t wait_time = updater_sleep_time_millis - (millis() - now);
    if (wait_time > 0) {
      rtos::ThisThread::sleep_for(wait_time);
    } 
  }
}

void setup(void) {

  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!bno08x.begin_I2C((uint8_t)BNO08x_I2CADDR_DEFAULT, new TwoWire(BNO08X_SDA, BNO08X_SCL), (int32_t)0)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports(reportType, reportIntervalUs);

  updater.start(mbed::callback(update));
  updater.join();

  Serial.println("Reading events");
  delay(100);
}

void loop() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
}