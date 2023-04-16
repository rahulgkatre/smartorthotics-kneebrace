#include "sh2_SensorValue.h"
#include <cmath>
#include <string>


/* ======================================================================================================================== */
/* ======================================================================================================================== */
/* ======================================================================================================================== */


// Datatype for storing accelerometer (linear) and gyro information
struct xyz_t {
  float x;
  float y;
  float z;
  xyz_t() : x(0), y(0), z(0) {}
  xyz_t(float nx, float ny, float nz) : x(nx), y(ny), z(nz) {}
  xyz_t(const xyz_t& other) : x(other.x), y(other.y), z(other.z) {}
  void set(float nx, float ny, float nz) {
    x = nx;
    y = ny;
    z = nz;
    return;
  }
  xyz_t& operator=(const xyz_t& val) {
    x = val.x;
    y = val.y;
    z = val.z;
    return *this;
  }
  xyz_t operator+(const xyz_t& val) const {
    return xyz_t(x + val.x, y + val.y, z + val.z);
  }
  xyz_t operator-(const xyz_t& val) const {
    return xyz_t(x - val.x, y - val.y, z - val.z);
  }
  xyz_t& operator+=(const xyz_t& val) {
    x += val.x;
    y += val.y;
    z += val.z;
    return *this;
  }
  xyz_t& operator-=(const xyz_t& val) {
    x -= val.x;
    y -= val.y;
    z -= val.z;
    return *this;
  }
};

// Orientation in the world frame
struct euler_t {
  float yaw;
  float pitch;
  float roll;
  void set(float ny, float np, float nr) {
    yaw = ny;
    pitch = np;
    roll = nr;
    return;
  }
};

// Raw output from the sensor
struct quaternion_t {
  float real;
  float i;
  float j;
  float k;
  void set(float nr, float ni, float nj, float nk) {
    real = nr;
    i = ni;
    j = nj;
    k = nk;
    return;
  }
};

// Step counter from initialization
struct steps_t {
  int steps;
  float latency;
  void set(float ns, float nl) {
    steps = ns;
    latency = nl;
    return;
  }
};

struct activity_t
{
  uint8_t most_likely;
  void set(uint8_t ml) {
    most_likely = ml;
    return;
  }
  String getMostLikelyActivity() {
  switch (most_likely) {
    case PAC_UNKNOWN:
      return "Unknown";
      break;
    case PAC_IN_VEHICLE:
      return "In Vehicle";
      break;
    case PAC_ON_BICYCLE:
      return "On Bicycle";
      break;
    case PAC_ON_FOOT:
      return "On Foot";
      break;
    case PAC_STILL:
      return "Still";
      break;
    case PAC_TILTING:
      return "Tilting";
      break;
    case PAC_WALKING:
      return "Walking";
      break;
    case PAC_RUNNING:
      return "Running";
      break;
    case PAC_ON_STAIRS:
      return "On Stairs";
      break;
    default:
      return "NOT LISTED";
    } 
    return "None";
  }
};

float magnitude(xyz_t* input, bool sqrt = false) {
  float sum = input->x * input->x + input->y * input->y + input->z * input->z;
  if (sqrt) {
    sum = std::sqrt(sum);
  }
  return sum;
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
    return;
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
    return;
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
    return;
}


/* ======================================================================================================================== */
/* ======================================================================================================================== */
/* ======================================================================================================================== */

// TODO function defines whether peak is peak depending on certain criteria...
bool natural_gait(float y_val, float delta_y) {
  return std::abs(y_val) > PEAK_DEADZONE_Y && std::abs(delta_y) < DELTA_ALIVEZONE;
}

class window_filter_xyz {
private:
  int num_readings;
  xyz_t sum;
  xyz_t avg;
  xyz_t prev_avg;
  xyz_t prev_vals[WINDOW_SIZE];

  xyz_t prev_delta;
  xyz_t delta;
  bool hit_peak_y = 0;
  int last_peak = 0;
  int last_pos = 0;
  int last_neg = 0;
  float stance_percent = 0;
  float second_last_neg_value = 0;
  float last_neg_value = 0;
  float neg_peak_ratio = 0;

  xyz_t prev_var;
  xyz_t var; 
public:
  window_filter_xyz() : num_readings(0), sum(xyz_t()), avg(xyz_t()), prev_avg(xyz_t()), prev_var(xyz_t()), var(xyz_t()) {
    for (int i = 0; i < WINDOW_SIZE; i++) {
      prev_vals[i] = xyz_t();
    }
  }
  // Applies moving average and modifies existing reading's values.
  void update(xyz_t* reading) {

    // For std
    xyz_t prev_reading = xyz_t();
    xyz_t prev_avg = xyz_t(avg);

    // Square reading to make peaks more visible?
    reading->x = reading->x * 4;
    reading->y = reading->y * 4;
    reading->z = reading->z * 4;

    // Calculate rolling sum
    if (num_readings < WINDOW_SIZE) {
      prev_vals[num_readings] = *reading;
    } else {
      prev_reading = xyz_t(prev_vals[num_readings % WINDOW_SIZE]);
      sum -= prev_reading;
      prev_vals[num_readings % WINDOW_SIZE] = *reading;
    }
    sum += *reading;
    num_readings += 1;

    // Find averages
    prev_avg = avg;
    avg.x = sum.x / std::min(WINDOW_SIZE, num_readings);
    avg.y = sum.y / std::min(WINDOW_SIZE, num_readings);
    avg.z = sum.z / std::min(WINDOW_SIZE, num_readings);
    prev_delta = delta;
    delta = avg - prev_avg;

    if (num_readings - last_peak > PEAK_SLEEP && (delta.y > 0 && prev_delta.y < 0 && avg.y < 0 || delta.y < 0 && prev_delta.y > 0 && avg.y > 0)) {
      hit_peak_y = 1;
      last_peak = num_readings;
    } else if (num_readings - last_peak > 4) {
      hit_peak_y = 0;
    }

    // Variance
    prev_var = var;
    var += xyz_t((INFLUENCE * (reading->x - prev_reading.x) * (reading->x - avg.x + prev_reading.x - prev_avg.x)) / (WINDOW_SIZE-1), 
            + (INFLUENCE * (reading->y - prev_reading.y) * (reading->y - avg.y + prev_reading.y - prev_avg.y)) / (WINDOW_SIZE-1), 
            + (INFLUENCE * (reading->z - prev_reading.z) * (reading->z - avg.z + prev_reading.z - prev_avg.z)) / (WINDOW_SIZE-1));

    return;
  }
  xyz_t* get_avg() {
    return &avg;
  }
  float get_std(bool avg=true) {
    float sum = var.x + var.y + var.z;
    if (sum < 0) {
      return 0;
    }
    return avg ? std::sqrt(sum / 3) : std::sqrt(sum);
  }
  float get_prev_std_y() {
    if (prev_var.y < 0) {
      return 0;
    }
    return std::sqrt(prev_var.y);
  }
  // returns 1 for positive peak, -1 for negative peak, 0 for no peak [ for gyro y only ]
  int peak_detection_y() {
    float difference = avg.y - (prev_avg.y);
    if (hit_peak_y && natural_gait(avg.y, delta.y) && std::abs(difference) > THRESHOLD * std::sqrt(prev_var.y)) {
      if (difference > 0) {
        stance_percent = (float)(last_neg - last_pos) / (float)(num_readings - last_pos);
        neg_peak_ratio = std::max(0.0f, std::min(1.0f, second_last_neg_value / last_neg_value));
        last_pos = num_readings;
        hit_peak_y = 0;
        return 1;
      } else {
        second_last_neg_value = last_neg_value;
        last_neg_value = avg.y;
        last_neg = num_readings;
        hit_peak_y = 0;
        return -1;
      }
    }
    return 0;
  }
  xyz_t* get_delta() {
    return &delta;
  }
  float get_stance_percent() {
    return stance_percent;
  }
  float get_peak_ratio() {
    return neg_peak_ratio;
  }
};


/* ======================================================================================================================== */
/* ======================================================================================================================== */
/* ======================================================================================================================== */


Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

xyz_t accel;
xyz_t gyro;
euler_t ypr;
quaternion_t rot_vec;
steps_t step_ctr;
activity_t activity;

window_filter_xyz accel_filter;
window_filter_xyz gyro_filter;

bool mockBNO08X = true;
unsigned long last_reading = 0;
unsigned long gyro_prev = 0;
unsigned long gyro_update = 0;

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
  return "{\"label\":\"Activity Classification\",\"data\":{\"mostLikely\":" + activity.getMostLikelyActivity() + "\"}}}";
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
  Serial.println("Reading events");
}

void getSensorData() {

  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_GYROSCOPE_CALIBRATED:
        gyro.set(sensorValue.un.gyroscope.x, sensorValue.un.gyroscope.y, sensorValue.un.gyroscope.z);
        gyro_filter.update(&gyro);
        gyro_prev = gyro_update;
        gyro_update = millis();
        break;
      case SH2_LINEAR_ACCELERATION:
        accel.set(sensorValue.un.linearAcceleration.x, sensorValue.un.linearAcceleration.y, sensorValue.un.linearAcceleration.z);
        accel_filter.update(&accel);
        break;
      case SH2_STEP_COUNTER:
        step_ctr.set(sensorValue.un.stepCounter.steps, sensorValue.un.stepCounter.latency);
        break;
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr);
        break;
      case SH2_PERSONAL_ACTIVITY_CLASSIFIER:
        activity.set(sensorValue.un.personalActivityClassifier.mostLikelyState);
        break;
    }
  }
}

void bno08XSetup() {
  
  while (!Serial) delay(10);

//  TwoWire wire = TwoWire(BNO08X_SDA, BNO08X_SCL)
  Wire.setSDA(BNO08X_SDA);
  Wire.setSCL(BNO08X_SCL);
//  Wire.setPins(

  // SETUP I2C
  if (!bno08x.begin_I2C((uint8_t)BNO08x_I2CADDR_DEFAULT, &Wire, (int32_t)0)) { //, &wire, (int32_t)0) { //** REQUIRED FOR MBED
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  // FLAG FOR NO IMU
  mockBNO08X = false;
  
  // SET UP IMU REPORTS
  setReports();
  delay(100);
  last_reading = millis();
//  gyro_prev = millis();
//  gyro_update = millis();
}

void bno08XLoop() {
  // delay(10);
  unsigned long curr = millis();
  if (PRINT && curr - last_reading >= UPDATE_RATE_MS - UPDATE_RATE_CORRECTION) {
//    Serial.print("\t"); Serial.print(curr - last_reading);
//    Serial.print("\t"); Serial.print(gyro_update - gyro_prev);
//    Serial.print("\t"); Serial.print(ypr.yaw);
    Serial.print("\t"); Serial.print(gyro_filter.get_avg()->y); // Rotational acceleration on y-axis
    Serial.print("\t"); Serial.print(gyro_filter.get_prev_std_y() * THRESHOLD); // Rotational acceleration on y-axis
    Serial.print("\t"); Serial.print(-1 * (gyro_filter.get_prev_std_y() * THRESHOLD)); // Rotational acceleration on y-axis
    Serial.print("\t"); Serial.print(gyro_filter.peak_detection_y() * 5); // Rotational acceleration on y-axis
    Serial.print("\t"); Serial.print(gyro_filter.get_stance_percent());
    Serial.print("\t"); Serial.print(gyro_filter.get_peak_ratio());
    Serial.print("\t"); Serial.print(magnitude(accel_filter.get_avg(), true));
    Serial.print("\t"); Serial.print(step_ctr.steps);
    Serial.print("\t"); Serial.print(int_to_volts(pcf.analogRead(3), 8, ADC_REFERENCE_VOLTAGE));
    
//    Serial.print("\t"); Serial.print(activity.getMostLikelyActivity());
    Serial.print("\t"); Serial.print(gyro_filter.get_delta()->y);
    Serial.println();
    last_reading = curr;
  }
  if (!mockBNO08X) {
    getSensorData();
  }
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
}
