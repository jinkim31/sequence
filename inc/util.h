#ifndef SEQUENCE_UTIL_H
#define SEQUENCE_UTIL_H

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

        double getTimeSec() { return timeSec; }
    };
}
#endif //SEQUENCE_UTIL_H
