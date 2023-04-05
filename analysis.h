#ifndef ANALYSIS
#define ANALYSIS

class window_filter_xyz {
private:
  int num_readings;
  xyz_t sum;
  xyz_t prev_vals[WINDOW_SIZE];
public:
  window_filter_xyz() : num_readings(0), sum(xyz_t()) {
    for (int i = 0; i < WINDOW_SIZE; i++) {
      prev_vals[i] = xyz_t();
    }
  }
  // Applies moving average and modifies existing reading's values.
  void update(xyz_t* reading) {
    if (num_readings < WINDOW_SIZE) {
      prev_vals[num_readings] = *reading;
    } else {
      sum -= prev_vals[num_readings % WINDOW_SIZE];
      prev_vals[num_readings % WINDOW_SIZE] = *reading;
    }
    sum += *reading;
    num_readings += 1;
    reading->x = sum.x / std::min(WINDOW_SIZE, num_readings);
    reading->y = sum.y / std::min(WINDOW_SIZE, num_readings);
    reading->z = sum.z / std::min(WINDOW_SIZE, num_readings);
  }
};

#endif
