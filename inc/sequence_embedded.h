#ifndef SEQUENCE_SEQUENCE_EMBEDDED_H
#define SEQUENCE_SEQUENCE_EMBEDDED_H

#include "sequence.h"
#include "chronometer.h"

using namespace std;
using namespace seq;

class EmbeddedChronometer : public Chronometer
{
private:
    double* currentTime;
public:
    EmbeddedChronometer(double* currentTimeVar);
    virtual double getTime();
    void setCurrentTimeVar(double* var);
};



#endif
