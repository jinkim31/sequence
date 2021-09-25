#include "../inc/base_blocks.h"

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

void seq::block::Print::reset() {}

seq::block::Delay::Delay(double timeSeconds)
{
  timeout.setTime(timeSeconds);
}

bool seq::block::Delay::update(SpinInfo spinInfo)
{
  if (timeout.addTime(spinInfo.timeDelta)) {
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

void seq::block::Function::reset() {}

seq::block::WaitFor::WaitFor(function<bool(void)> breakCondition, double timeout, function<void(void)> timeoutHandler)
{
  this->breakCondition = breakCondition;
  this->timeout.setTime(timeout);
  this->timeout.setTimeoutHandler(timeoutHandler);
}

seq::block::WaitFor::WaitFor(function<bool(void)> breakCondition, double timeout) : WaitFor(breakCondition, timeout,
                                                                                            NULL) {}

seq::block::WaitFor::WaitFor(function<bool(void)> breakCondition) : WaitFor(breakCondition, -1.0) {}

bool seq::block::WaitFor::update(SpinInfo spinInfo)
{
  if (timeout.addTime(spinInfo.timeDelta)) {
    return true;
  }

  return breakCondition();
}

void seq::block::WaitFor::reset() {}

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

string seq::block::SequenceBlock::generateDebugName() { return string("Sequence(") + sequence->geName() + string(")"); }

void seq::block::SequenceBlock::print()
{
  printDebug(generateDebugName(), true);
  sequence->print();
}

seq::block::LoopSequence::LoopSequence(Sequence *sequence, function<bool(void)> breakCondition, double timeout, function<void(void)> timeoutHandler) : SequenceBlock(sequence)
{
  this->breakCondition = breakCondition;
  this->timeout.setTime(timeout);
  this->timeout.setTimeoutHandler(timeoutHandler);
}

seq::block::LoopSequence::LoopSequence(Sequence *sequence, function<bool(void)> breakCondition, double timeout): LoopSequence(sequence, breakCondition, timeout, NULL) {}

seq::block::LoopSequence::LoopSequence(Sequence *sequence, function<bool(void)> breakCondition) : LoopSequence(sequence,breakCondition,-1.0) {}

bool seq::block::LoopSequence::update(SpinInfo spinInfo)
{
  if (timeout.addTime(spinInfo.timeDelta)) {
    return true;//tined out. forced finish
  }
  if (sequence->update(spinInfo))//inner sequence finished
  {
    sequence->stop();
    sequence->start();

    return breakCondition();
  }
  return false;
}

string seq::block::LoopSequence::generateDebugName()
{
  return string("Loop(") + sequence->geName() + string(")");
}
