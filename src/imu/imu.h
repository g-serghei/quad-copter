#include "mpu9250.h"
#include <MedianFilter.h>

#ifndef SRC_IMU_H_
#define SRC_IMU_H_

typedef struct
{
    median_filter_t x, y, z;
} MeridialFilterType;

typedef struct
{
    float x, y, z;
} AxisType;

typedef struct
{
    AxisType accel, gyro;
} ImuType;

namespace IMU
{
    void init(bool debugModeEnabled);
    void calibrate();
    void updateData();
    void processData();
    void serialPrintf(const char *format, ...);
    void serialPrintlnf(const char *format, ...);
    void printAxis(AxisType axis);
}

#endif // SRC_IMU_H_