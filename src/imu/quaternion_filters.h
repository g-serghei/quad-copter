// #include "imu2.cpp"
#include "imu.h"

void madgwick_quaternion_update(
    AxisType *angles,
    float deltat,
    float ax, float ay, float az,
    float gx, float gy, float gz);
