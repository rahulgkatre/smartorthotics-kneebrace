#include "sh2_SensorValue.h"
#ifndef UTILS
#define UTILS

#include <cmath>

// Datatype for storing accelerometer (linear) and gyro information
typedef struct xyz_t {
  float x;
  float y;
  float z;
};

// Orientation in the world frame
typedef struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

// Raw output from the sensor
typedef struct quaternion_t {
  float real;
  float i;
  float j;
  float k;
};

// Step counter from initialization
typedef struct steps_t {
  int steps;
  float latency;
};

// Stability classification. We only want to read when the sensor is moving (reading = 4)
typedef struct stability_t {
  char *classification;
};

typedef struct activity_t
{
  char *mostLikely;
  float unknownConf;
  float inVehicleConf;
  float onBicycleConf;
  float onFootConf;
  float stillConf;
  float tiltingConf;
  float walkingConf;
  float runningConf;
  float onStairsConf;
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
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

#endif