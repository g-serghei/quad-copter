#include <Arduino.h>
#include "rc.h"
#include "imu.h"

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
    }

    // Imu::init();
    Rc::init();
}

void loop()
{
    // Imu::updateData();
    // Imu::processData();
}
