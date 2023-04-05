#include "sh2_SensorValue.h"
#ifndef UTILS
#define UTILS

#include <cmath>

typedef struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

typedef struct accel_t {
  float x;
  float y;
  float z;
};

typedef struct gyro_t {
  float x;
  float y;
  float z;
};

double magnitude_acc(accel_t* input, bool sqrt = false) {
  double sum = input->x * input->x + input->y * input->y + input->z * input->z;
  if (sqrt) {
    std::sqrt(sum);
  }
  return sum;
}

double magnitude_gyro(gyro_t* input, bool sqrt = false) {
  double sum = input->x * input->x + input->y * input->y + input->z * input->z;
  if (sqrt) {
    std::sqrt(sum);
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

void getAccel(sh2_Accelerometer_t* accel_vector, accel_t* acc) {
    acc->x = accel_vector->x;
    acc->y = accel_vector->y;
    acc->z = accel_vector->z;  
}

void getGyro(sh2_Gyroscope_t* gyro_vector, gyro_t* gyro) {
    gyro->x = gyro_vector->x;
    gyro->y = gyro_vector->y;
    gyro->z = gyro_vector->z;  
}



#endif