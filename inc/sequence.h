#ifndef SEQUENCE_SEQUENCE_H
#define SEQUENCE_SEQUENCE_H

#include "sequence_core.h"
#include "sequence_util.h"

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

    bool update(SpinInfo spinInfo) override;

    void reset() override;

    string generateDebugName() override;
};

class Debug : public Block
{
private:
    string text;
public:
    Debug(const string &text);

    bool update(SpinInfo spinInfo) override;

    void reset() override;

    string generateDebugName() override;
};

class Delay : public Block
{
private:
    Timeout timeout;
public:
    Delay(double timeSeconds);

    bool update(SpinInfo spinInfo) override;

    void reset() override;

    string generateDebugName() override;
};

class Function : public Block
{
private:
    function<void(void)> func;
public:
    Function(function<void(void)> func);

    bool update(SpinInfo spinInfo) override;

    void reset() override;

    string generateDebugName() override;
};

class WaitFor : public Block
{
private:
    shared_ptr<Condition> breakCondition;
    Timeout timeout;
public:
    WaitFor(shared_ptr<Condition> breakCondition, double timeout, function<void(void)> timeoutHandler);

    WaitFor(shared_ptr<Condition> breakCondition, double timeout);

    WaitFor(shared_ptr<Condition> breakCondition);

    bool update(SpinInfo spinInfo) override;

    void reset() override;

    string generateDebugName() override;
};

class SequenceBlock : public Block
{
private:
protected:
    shared_ptr<Sequence> sequence;
    SequenceBlock();
public:
    SequenceBlock(shared_ptr<Sequence> sequence);

    ~SequenceBlock();

    void setSequence(shared_ptr<Sequence> sequence);

    bool update(SpinInfo spinInfo) override;

    void reset() override;

    string generateDebugName() override;

    virtual void init(bool debug);

    virtual void print();
};

class LoopSequence : public SequenceBlock
{
private:
    Timeout timeout;
    shared_ptr<Condition> breakCondition;
public:
    LoopSequence(shared_ptr<Condition> breakCondition, double timeout, function<void(void)> timeoutHandler, shared_ptr<Sequence> sequence);

    LoopSequence(shared_ptr<Condition> breakCondition, double timeout, shared_ptr<Sequence> sequence);

    LoopSequence(shared_ptr<Condition> breakCondition, shared_ptr<Sequence> sequence);

    void reset() override;

    bool update(SpinInfo spinInfo) override;

    string generateDebugName() override;
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

    bool update(SpinInfo spinInfo) override;

    void reset() override;

    string generateDebugName() override;
};

class Broadcast : public Block
{
private:
    string msg;
public:
    Broadcast(string msg, bool global = true);

    bool update(SpinInfo spinInfo) override;

    void reset() override;

    string generateDebugName() override;
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
