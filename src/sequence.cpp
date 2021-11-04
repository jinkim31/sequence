#include "../inc/sequence.h"

seq::block::Print::Print(const std::string &text)
{
    this->text = text;
    this->text = text;
}

bool seq::block::Print::update(SpinInfo spinInfo)
{
    cout << text << endl;
    return true;
}

string seq::block::Print::generateDebugName()
{
    return "Print(" + text + ")";
}

void seq::block::Print::reset()
{

}

seq::block::Delay::Delay(double timeSeconds)
{
    timeout.setTime(timeSeconds);
}

bool seq::block::Delay::update(SpinInfo spinInfo)
{
    if (timeout.addTime(spinInfo.timeDelta))
    {
        return true;
    }
    return false;
}

void seq::block::Delay::reset()
{
    timeout.reset();
}

string seq::block::Delay::generateDebugName()
{
    return string("Delay(") + to_string(timeout.getTimeSec()) + string(")");
}

seq::block::Function::Function(function<void(void)> func)
{
    this->func = func;
}

bool seq::block::Function::update(SpinInfo spinInfo)
{
    func();
    return true;
}

void seq::block::Function::reset()
{}

string seq::block::Function::generateDebugName()
{
    return "Function";
}

seq::block::WaitFor::WaitFor(Condition* breakCondition, double timeout, function<void(void)> timeoutHandler)
{
    this->breakCondition = breakCondition;
    this->timeout.setTime(timeout);
    this->timeout.setTimeoutHandler(timeoutHandler);
}

seq::block::WaitFor::WaitFor(Condition* breakCondition, double timeout) : WaitFor(breakCondition, timeout,
NULL)
{}

seq::block::WaitFor::WaitFor(Condition* breakCondition) : WaitFor(breakCondition, -1.0)
{}

bool seq::block::WaitFor::update(SpinInfo spinInfo)
{
    if (timeout.addTime(spinInfo.timeDelta))
    {
        Sequence::printDebug("timed out");
        return true;
    }

    if (breakCondition->evaluate())
    {
        //Sequence::printDebug("condition met");
        return true;
    }

    return false;
}

void seq::block::WaitFor::reset()
{
    timeout.reset();
}

string seq::block::WaitFor::generateDebugName()
{
    string name = "WaitFor(timeout:";
    if (timeout.getTimeSec() < 0) name += "none";
    else name += to_string(timeout.getTimeSec());
    name += ")";
    return name;
}

seq::block::SequenceBlock::SequenceBlock(Sequence *sequence)
{
    this->sequence = sequence;
    sequence->setAsOrigin(false);
}

seq::block::SequenceBlock::~SequenceBlock()
{
    delete (sequence);
}

bool seq::block::SequenceBlock::update(SpinInfo spinInfo)
{
    return sequence->update(spinInfo);
}

void seq::block::SequenceBlock::reset()
{
    sequence->stop();
    sequence->start();
}

void seq::block::SequenceBlock::init(bool debug)
{
    sequence->setHierarchyLevel(containerSequence->getHierarchyLevel() + 1);
    sequence->init(debug);
}

string seq::block::SequenceBlock::generateDebugName()
{ return string("Sequence(") + sequence->getName() + string(")"); }

void seq::block::SequenceBlock::print()
{
    Sequence::printDebug(generateDebugName(), true);
    sequence->print();
}

seq::block::LoopSequence::LoopSequence(Condition* breakCondition, double timeout,function<void(void)> timeoutHandler, Sequence *sequence) : SequenceBlock(sequence)
{
    this->breakCondition = breakCondition;
    this->timeout.setTime(timeout);
    this->timeout.setTimeoutHandler(timeoutHandler);
}

seq::block::LoopSequence::LoopSequence(Condition* breakCondition, double timeout, Sequence *sequence): LoopSequence(breakCondition, timeout, NULL, sequence)
{}

seq::block::LoopSequence::LoopSequence(Condition* breakCondition, Sequence *sequence) : LoopSequence(breakCondition, -1.0, sequence)
{}

bool seq::block::LoopSequence::update(SpinInfo spinInfo)
{
    if (timeout.addTime(spinInfo.timeDelta))
    {
        Sequence::printDebug("timed out");
        sequence->stop();
        return true;//tined out. forced finish
    }
    if (sequence->update(spinInfo))//inner sequence finished
    {
        sequence->stop();
        sequence->start();
    }
    if(breakCondition->evaluate())
    {
        sequence->stop();
        return true;
    }
    return false;
}

string seq::block::LoopSequence::generateDebugName()
{
    string name = "Loop(sequence:" + sequence->getName() + ", timeout:";
    if (timeout.getTimeSec() < 0) name += "none";
    else name += to_string(timeout.getTimeSec());
    name += ")";
    return name;
}

seq::block::Debug::Debug(const string &text) : text(text)
{}

bool seq::block::Debug::update(seq::SpinInfo spinInfo)
{
    if (containerSequence->debugEnabled()) Sequence::printDebug(text);
    else cout << text << endl;
    return true;
}

string seq::block::Debug::generateDebugName()
{
    return "Debug(" + text + ")";
}

void seq::block::Debug::reset()
{

}

seq::block::WaitForBroadcast::WaitForBroadcast(string broadcastMsg, double timeout, function<void(void)> timeoutHandler): broadcastWaitingFor(broadcastMsg)
{
    this->timeout.setTime(timeout);
    this->timeout.setTimeoutHandler(timeoutHandler);
}

seq::block::WaitForBroadcast::WaitForBroadcast(string broadcastMsg, double timeout) : WaitForBroadcast(broadcastMsg, timeout, NULL)
{
}

seq::block::WaitForBroadcast::WaitForBroadcast(string broadcastMsg) : WaitForBroadcast(broadcastMsg, -1)
{
}

bool seq::block::WaitForBroadcast::update(seq::SpinInfo spinInfo)
{
    if(timeout.addTime(spinInfo.timeDelta))
    {
        Sequence::printDebug("timeout");
        return true;
    }
    return spinInfo.broadcastMsg == broadcastWaitingFor;
}

void seq::block::WaitForBroadcast::reset()
{

}

string seq::block::WaitForBroadcast::generateDebugName()
{
    return "WaitForBroadcast("+broadcastWaitingFor+")";
}

bool seq::block::Broadcast::update(seq::SpinInfo spinInfo)
{
    Sequence::broadcast(msg);
    return true;
}

void seq::block::Broadcast::reset()
{

}

seq::block::Broadcast::Broadcast(string msg, bool global) : msg(msg)
{
}

string seq::block::Broadcast::generateDebugName()
{
    return "Broadcast("+msg+")";
}

bool seq::block::StartSequence::update(seq::SpinInfo spinInfo)
{
    Sequence::startSequence(sequenceName);
    return true;
}

void seq::block::StartSequence::reset()
{}

seq::block::StartSequence::StartSequence(string sequenceName) : sequenceName(sequenceName)
{

}