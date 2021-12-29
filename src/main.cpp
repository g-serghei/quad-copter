#include <Arduino.h>
#include "imu/imu.h"

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
    }

    IMU::init(true);
}

void loop()
{
    IMU::updateData();
    IMU::processData();
}
