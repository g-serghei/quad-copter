#include <Arduino.h>

namespace Log
{
    void error(String message)
    {
        Serial.println(message);
        while (1)
        {
        }
    }

    void info(String message)
    {
        Serial.println(message);
    }

    void info(Printable& message)
    {
        Serial.println(message);
    }
}