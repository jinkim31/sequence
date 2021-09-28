#ifndef SEQUENCE_CHRONOMETER_H
#define SEQUENCE_CHRONOMETER_H

#include <chrono>

using namespace std;

class Chronometer
{
public:
    virtual double getTime();
};


#endif