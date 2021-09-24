#include "../inc/base_blocks.h"

seq::block::Print::Print(const std::string& text)
{
  this->text = text;
  this->text = text;
}
bool seq::block::Print::spinOnce(SpinInfo spinInfo)
{
  cout << text << endl;
  return true;
}
void seq::block::Print::reset(){}

seq::block::Delay::Delay(double timeSeconds)
{
  timeout.setTime(timeSeconds);
}

bool seq::block::Delay::spinOnce(SpinInfo spinInfo)
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

seq::block::Function::Function(function<void(void)> func)
{
  this->func = func;
}
bool seq::block::Function::spinOnce(SpinInfo spinInfo)
{
  func();
  return true;
}
void seq::block::Function::reset(){}

seq::block::WaitFor::WaitFor(function<bool(void)> breakCondition, double timeout, function<void(void)> timeoutHandler)
{
  this->breakCondition = breakCondition;
  this->timeout.setTime(timeout);
  this->timeout.setTimeoutHandler(timeoutHandler);
}
seq::block::WaitFor::WaitFor(function<bool(void)> breakCondition, double timeout) :WaitFor(breakCondition, timeout, NULL){}

seq::block::WaitFor::WaitFor(function<bool(void)> breakCondition) : WaitFor(breakCondition, -1.0){}

bool seq::block::WaitFor::spinOnce(SpinInfo spinInfo)
{
  if (timeout.addTime(spinInfo.timeDelta))
  {
    return true;
  }

  return breakCondition();
}
void seq::block::WaitFor::reset(){}

seq::block::SequenceBlock::SequenceBlock(Sequence *sequence)
{
  this->sequence = sequence;
  sequence->setIsMaster(false);
}
seq::block::SequenceBlock::~SequenceBlock()
{
  delete(sequence);
}

void seq::block::SequenceBlock::notifyStart()
{
  sequence->setHierarchyLevel(containerSequence->getHierarchyLevel()+1);
  sequence->reset();
  sequence->start();
}
bool seq::block::SequenceBlock::spinOnce(SpinInfo spinInfo)
{
  return sequence->spinOnce();
}
void seq::block::SequenceBlock::reset()
{
  sequence->reset();
}

seq::block::LoopSequence::LoopSequence(Sequence* sequence, function<bool(void)> breakCondition, double timeout, function<void(void)> timeoutHandler) : SequenceBlock(sequence)
{
  this->breakCondition = breakCondition;
  this->timeout.setTime(timeout);
  this->timeout.setTimeoutHandler(timeoutHandler);
}

seq::block::LoopSequence::LoopSequence(Sequence* sequence, function<bool(void)> breakCondition, double timeout) : LoopSequence(sequence, breakCondition, timeout, NULL) {}

seq::block::LoopSequence::LoopSequence(Sequence* sequence, function<bool(void)> breakCondition) : LoopSequence(sequence, breakCondition, -1.0) {}

bool seq::block::LoopSequence::spinOnce(SpinInfo spinInfo)
{
  if (timeout.addTime(spinInfo.timeDelta))
  {
    return true;//tined out. forced finish
  }
  if (sequence->spinOnce())//inner sequence finished
  {
    sequence->reset();
    sequence->start();

    return breakCondition();
  }
  return false;
}