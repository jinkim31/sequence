#ifndef SEQUENCE_SEQUENCE_H
#define SEQUENCE_SEQUENCE_H

#include "sequence_core.h"
#include "util.h"

using namespace std;

namespace seq
{
namespace block
{
class Print : public Block
{
private:
    std::string text;
public:
    Print(const std::string &text);

    virtual bool update(SpinInfo spinInfo);

    virtual void reset();

    virtual string generateDebugName();
};

class Debug : public Block
{
private:
    std::string text;
public:
    Debug(const std::string &text);

    virtual bool update(SpinInfo spinInfo);

    virtual void reset();

    virtual string generateDebugName();
};

class Delay : public Block
{
private:
    Timeout timeout;
public:
    Delay(double timeSeconds);

    virtual bool update(SpinInfo spinInfo);

    virtual void reset();

    virtual string generateDebugName();
};

class Function : public Block
{
private:
    function<void(void)> func;
public:
    Function(function<void(void)> func);

    virtual bool update(SpinInfo spinInfo);

    virtual void reset();

    virtual string generateDebugName();
};

class WaitFor : public Block
{
private:
    Condition *breakCondition;
    Timeout timeout;
public:
    WaitFor(Condition *breakCondition, double timeout, function<void(void)> timeoutHandler);

    WaitFor(Condition *breakCondition, double timeout);

    WaitFor(Condition *breakCondition);

    virtual bool update(SpinInfo spinInfo);

    virtual void reset();

    virtual string generateDebugName();
};

class SequenceBlock : public Block
{
private:
protected:
    Sequence *sequence;
public:
    SequenceBlock(Sequence *sequence);

    ~SequenceBlock();

    virtual bool update(SpinInfo spinInfo);

    virtual void reset();

    virtual string generateDebugName();

    virtual void init(bool debug) final;

    virtual void print() final;
};

class LoopSequence : public SequenceBlock
{
private:
    Timeout timeout;
    Condition *breakCondition;
public:
    LoopSequence(Condition *breakCondition, double timeout, function<void(void)> timeoutHandler, Sequence *sequence);

    LoopSequence(Condition *breakCondition, double timeout, Sequence *sequence);

    LoopSequence(Condition *breakCondition, Sequence *sequence);

    virtual bool update(SpinInfo spinInfo);

    virtual string generateDebugName();
};

class WaitForBroadcast : public Block
{
private:
    string broadcastWaitingFor;
    Timeout timeout;
public:
    WaitForBroadcast(string broadcastMsg, double timeout, function<void(void)> timeoutHandler);

    WaitForBroadcast(string broadcastMsg, double timeout);

    WaitForBroadcast(string broadcastMsg);

    virtual bool update(SpinInfo spinInfo);

    virtual void reset();

    virtual string generateDebugName();
};

class Broadcast : public Block
{
private:
    string msg;
public:
    Broadcast(string msg, bool global = true);

    virtual bool update(SpinInfo spinInfo);

    virtual void reset();

    virtual string generateDebugName();
};

class StartSequence : public Block
{
private:
    string sequenceName;
public:
    StartSequence(string sequenceName);

    bool update(SpinInfo spinInfo) override;

    void reset() override;
};
}
}

#endif //SEQUENCE_SEQUENCE_H
