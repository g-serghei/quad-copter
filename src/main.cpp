#include <Arduino.h>
#include "rc.h"
#include "imu.h"

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
    }

    //Rc::init();
    Imu::init();
}

void loop()
{
    Imu::process();
    //Rc::process();
}
