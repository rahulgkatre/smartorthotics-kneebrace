#include "utils.h"

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// 1000 Hz standard update rate
#define REPORT_RATE_US 500
// 10 Hz interpretation rate
#define INTER_RATE_US  100000
// 400 Hz YPR update rate (do we need this?)
#define ARVR_ROTVEC_US 2500
// 100 Hz serial output rate
#define UPDATE_RATE_MS 10
#define UPDATE_RATE_CORRECTION 2

xyz_t accel;
xyz_t gyro;
euler_t ypr;
quaternion_t rot_vec;
steps_t step_ctr;
activity_t activity;

bool mockBNO08X = true;

String accelToJsonString() {
  return "{\"label\":\"Acceleration\",\"data\":{\"x\":" + String(accel.x) + ",\"y\":" + String(accel.y) + ",\"z\":" + String(accel.z) + "}}";
}

String gyroToJsonString() {
  return "{\"label\":\"Gyroscope\",\"data\":{\"px\":" + String(gyro.x) + ",\"py\":" + String(gyro.y) + ",\"pz\":" + String(gyro.z) + "}}";
}

String eulerToJsonString() {
  return "{\"label\":\"Euler Angles\",\"data\":{\"yaw\":" + String(ypr.yaw) + ",\"pitch\":" + String(ypr.pitch) + ",\"roll\":" + String(ypr.roll) + "}}";
}

String quaternionToJsonString() {
  return "{\"label\":\"Rotation Vector\",\"data\":{\"real\":" + String(rot_vec.real) + ",\"i\":" + String(rot_vec.i) + ",\"j\":" + String(rot_vec.j) + ",\"k\":" + String(rot_vec.k) + "}}";
}

String stepsToJsonString() {
  return "{\"label\":\"Step Counter\",\"data\":{\"steps\":" + String(step_ctr.steps) + ",\"latency\":" + String(step_ctr.latency) + "}}";
}

String getActivityJsonString() {
  return "{\"label\":\"Activity Classification\",\"data\":{\"mostLikely\":" + String(activity.mostLikely) + "\"}}}";
}

String getMockJsonString() {
  accel = {random(-10, 10), random(-10, 10), random(-10, 10)};
  rot_vec = {random(-10, 10), random(-10, 10), random(-10, 10), random(-10, 10)};

  String mockXYZJsonString = accelToJsonString();
  String mockQJsonString = quaternionToJsonString();
  return "[" + mockXYZJsonString + "," + mockQJsonString + "]";
}


void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, REPORT_RATE_US)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_RATE_US)) {
    Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_STEP_COUNTER, INTER_RATE_US)) {
    Serial.println("Could not enable step counter");
  }
  if (!bno08x.enableReport(SH2_PERSONAL_ACTIVITY_CLASSIFIER, INTER_RATE_US)) {
    Serial.println("Could not enable personal activity classifier");
  }
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, ARVR_ROTVEC_US)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void getMostLikelyActivity(uint8_t activity_id) {
  switch (activity_id) {
  case PAC_UNKNOWN:
    activity.mostLikely = "Unknown";
    break;
  case PAC_IN_VEHICLE:
    activity.mostLikely = "In Vehicle";
    break;
  case PAC_ON_BICYCLE:
    activity.mostLikely = "On Bicycle";
    break;
  case PAC_ON_FOOT:
    activity.mostLikely = "On Foot";
    break;
  case PAC_STILL:
    activity.mostLikely = "Still";
    break;
  case PAC_TILTING:
    activity.mostLikely = "Tilting";
    break;
  case PAC_WALKING:
    activity.mostLikely = "Walking";
    break;
  case PAC_RUNNING:
    activity.mostLikely = "Running";
    break;
  case PAC_ON_STAIRS:
    activity.mostLikely = "On Stairs";
    break;
  default:
    activity.mostLikely = "NOT LISTED";
  }
}

void getSensorData() {

  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_GYROSCOPE_CALIBRATED:
        gyro.x = sensorValue.un.gyroscope.x;
        gyro.y = sensorValue.un.gyroscope.y;
        gyro.z = sensorValue.un.gyroscope.z;
        break;
      case SH2_LINEAR_ACCELERATION:
        accel.x = sensorValue.un.linearAcceleration.x;
        accel.y = sensorValue.un.linearAcceleration.y;
        accel.z = sensorValue.un.linearAcceleration.z;
        break;
      case SH2_STEP_COUNTER:
        step_ctr.steps = sensorValue.un.stepCounter.steps;
        step_ctr.latency = sensorValue.un.stepCounter.latency;
        break;
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr);
        break;
      case SH2_PERSONAL_ACTIVITY_CLASSIFIER:
        getMostLikelyActivity(sensorValue.un.personalActivityClassifier.mostLikelyState);
        activity.unknownConf = sensorValue.un.personalActivityClassifier.confidence[PAC_UNKNOWN];
        activity.inVehicleConf = sensorValue.un.personalActivityClassifier.confidence[PAC_IN_VEHICLE];
        activity.onBicycleConf = sensorValue.un.personalActivityClassifier.confidence[PAC_ON_BICYCLE];
        activity.onFootConf = sensorValue.un.personalActivityClassifier.confidence[PAC_ON_FOOT];
        activity.stillConf = sensorValue.un.personalActivityClassifier.confidence[PAC_STILL];
        activity.tiltingConf = sensorValue.un.personalActivityClassifier.confidence[PAC_TILTING];
        activity.walkingConf = sensorValue.un.personalActivityClassifier.confidence[PAC_WALKING];
        activity.runningConf = sensorValue.un.personalActivityClassifier.confidence[PAC_RUNNING];
        activity.onStairsConf = sensorValue.un.personalActivityClassifier.confidence[PAC_ON_STAIRS];
        break;
    }
  }
}

unsigned long last = 0;

void bno08XSetup() {
  
  while (!Serial) delay(10);

  if (!bno08x.begin_I2C((uint8_t)BNO08x_I2CADDR_DEFAULT, new TwoWire(BNO08X_SDA, BNO08X_SCL), (int32_t)0)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }

  mockBNO08X = false;
  
  Serial.println("BNO08x Found!");
  setReports();
  
  Serial.println("Reading events");
  delay(100);

  last = millis();
}

void bno08XLoop() {
  // delay(10);
  unsigned long curr = millis();
  if (curr - last >= UPDATE_RATE_MS - UPDATE_RATE_CORRECTION) {
    Serial.print(curr - last);
    Serial.print("\t"); Serial.print(ypr.yaw);
    Serial.print("\t"); Serial.print(gyro.y); // Rotational acceleration on y-axis
    Serial.print("\t"); Serial.println(magnitude(&accel, true));
    last = curr;
  }
  if (!mockBNO08X) {
    getSensorData();
  }
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
}
