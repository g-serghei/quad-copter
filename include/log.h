#include <Arduino.h>

#ifndef SRC_LOG_H_
#define SRC_LOG_H_

namespace Log
{
    void error(String message);
    void info(String message);
    void info(Printable& message);
}

#endif

