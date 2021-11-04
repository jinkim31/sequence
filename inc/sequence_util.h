#ifndef SEQUENCE_SEQUENCE_UTIL_H
#define SEQUENCE_SEQUENCE_UTIL_H

#include <iostream>
#include <functional>

using namespace std;

namespace seq
{
class Timeout
{
private:
    double timeSec;
    double timeElapsed;
    function<void(void)> timeoutHandler;
    bool timeoutHandlerAvailable;
public:
    Timeout(double timeSec, function<void(void)> timeoutHandler);

    Timeout(double timeSec);

    Timeout();

    void setTime(double timeSec);

    void setTimeoutHandler(function<void(void)> timeoutHandler);

    void reset();

    bool addTime(double timeDelta);

    double getTimeSec();

    double getTimeElapsed();
};

class Condition
{
public:
    virtual bool evaluate() = 0;
};

class LambdaCondition : public Condition
{
private:
    function<bool(void)> condition;
public:
    LambdaCondition(function<bool(void)> condition);
    ~LambdaCondition();
    virtual bool evaluate();
};

}
#endif //SEQUENCE_SEQUENCE_UTIL_H
