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
    function<bool(void)> breakCondition;
    Timeout timeout;
public:
    WaitFor(function<bool(void)> breakCondition, double timeout, function<void(void)> timeoutHandler);

    WaitFor(function<bool(void)> breakCondition, double timeout);

    WaitFor(function<bool(void)> breakCondition);

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
    function<bool(void)> breakCondition;
public:
    LoopSequence(Sequence *sequence, function<bool(void)> breakCondition, double timeout,function<void(void)> timeoutHandler);

    LoopSequence(Sequence *sequence, function<bool(void)> breakCondition, double timeout);

    LoopSequence(Sequence *sequence, function<bool(void)> breakCondition);

    virtual bool update(SpinInfo spinInfo);

    virtual string generateDebugName();
};

}
}

#endif //SEQUENCE_SEQUENCE_H
