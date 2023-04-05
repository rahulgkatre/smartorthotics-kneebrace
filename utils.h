#include "sh2_SensorValue.h"
#ifndef UTILS
#define UTILS

#include <cmath>

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
  }
  xyz_t& operator=(const xyz_t& val) {
    x = val.x;
    y = val.y;
    z = val.z;
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
  }
};

// Step counter from initialization
struct steps_t {
  int steps;
  float latency;
  void set(float ns, float nl) {
    steps = ns;
    latency = nl;
  }
};

struct activity_t
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