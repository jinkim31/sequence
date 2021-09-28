#include "../inc/sequence_embedded.h"

double EmbeddedChronometer::getTime()
{
    return Chronometer::getTime();
}

void EmbeddedChronometer::setCurrentTimeVar(double *var)
{
    this->currentTime = var;
}

EmbeddedChronometer::EmbeddedChronometer(double *currentTimeVar) : currentTime(currentTimeVar)
{
}
