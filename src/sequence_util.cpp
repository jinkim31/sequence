#include "../inc/sequence_util.h"

seq::Timeout::Timeout(double timeSec, function<void(void)> timeoutHandler)
{
    timeElapsed = 0.0;
    this->timeSec = timeSec;
    this->timeoutHandler = timeoutHandler;
}

seq::Timeout::Timeout(double timeSec) : Timeout(timeSec, NULL)
{}

seq::Timeout::Timeout() : Timeout(0.0)
{}

void seq::Timeout::setTime(double timeSec)
{ this->timeSec = timeSec; }

void seq::Timeout::setTimeoutHandler(function<void(void)> timeoutHandler)
{
    this->timeoutHandler = timeoutHandler;
}

void seq::Timeout::reset()
{ timeElapsed = 0.0; }

bool seq::Timeout::addTime(double timeDelta)
{
    if (timeSec < 0.0) return false;
    timeElapsed += timeDelta;
    if (timeElapsed >= timeSec)
    {
        if (timeoutHandler != NULL) timeoutHandler();
        return true;
    }
    return false;
}

double seq::Timeout::getTimeSec()
{ return timeSec; }

double seq::Timeout::getTimeElapsed()
{
    return timeElapsed;
}

bool seq::LambdaCondition::evaluate()
{
    return condition();
}

seq::LambdaCondition::LambdaCondition(function<bool(void)> condition) : condition(condition)
{
}

seq::LambdaCondition::~LambdaCondition()
{

}

void seq::LambdaCondition::reset()
{

}
