#include "../inc/chronometer.h"

double Chronometer::getTime()
{
    return std::chrono::system_clock::now().time_since_epoch().count()/1000000000.0;
}
