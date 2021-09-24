#include "../inc/sequence.h"

using namespace seq;

Sequence::~Sequence()
{
  clear();
}

void seq::Sequence::setHierarchyLevel(unsigned int value)
{
  hierarchyLevel = value;
}

unsigned int seq::Sequence::getHierarchyLevel()
{
  return hierarchyLevel;
}

void Sequence::add(Block *block)
{
  if (isCompiled) {
    throw InvalidOperation("Can't add blocks to already compiled sequence.");
  }
  blockList.push_back(block);
}

void Sequence::clear()
{
  //cout<<"clearing sequence"<<endl;
  for (Block *block: blockList) {
    delete (block);
  }
  blockList.clear();
  currentStep = 0;
}

bool Sequence::spinOnce(SpinInfo spinInfo)
{
  while (true) {
    if (blockList[currentStep]->spinOnce(spinInfo)) {
      //step
      currentStep++;
      //reset the block, call callback
      if ((size_t) currentStep < blockList.size()) {
        blockList[currentStep]->printDebug(blockList[currentStep]->generateDebugName(), true);
        blockList[currentStep]->reset();
        blockList[currentStep]->startCallback();
      }
        //Check for end of the sequence.
      else {
        stop();
        finished = true;
        if (isOrigin)cout << string("Sequence terminated.(") + name + string(")") << endl;
        return true;
      }
    } else {
      //return when false returned.
      return false;
    }

    //exit loop.
    if (steadyStep) break;
  }

  return false;
}

bool Sequence::spinOnce()
{
  //cout<<"Running:"<<running<<" blocks:"<<currentStep<<"/"<<blockList.size()<<endl;
  if (!running) return false;

  //Calculate time duration since last spin.
  chrono::duration<double> timeUpdateDelta = chrono::system_clock::now() - timeLastUpdate;
  timeLastUpdate = chrono::system_clock::now();
  SpinInfo spinInfo;
  spinInfo.timeDelta = timeUpdateDelta.count();

  return spinOnce(spinInfo);
}

void Sequence::start()
{
  if (!blockList.empty()) {
    if (isOrigin)cout << string("Sequence started.(") + name + string(")") << endl;

    running = true;
    finished = false;
    blockList[0]->printDebug(blockList[currentStep]->generateDebugName(), true);
    blockList[0]->reset();
    blockList[0]->startCallback();
  }
}

void Sequence::suspend()
{
  running = false;
}

void Sequence::stop()
{
  running = false;
  currentStep = 0;
}

bool Sequence::isRunning()
{
  return running;
}

bool Sequence::isFinished()
{
  return finished;
}

void Sequence::enableSteadyStep(bool value)
{
  steadyStep = value;
}

void Sequence::compile(bool debug)
{
  this->debug = debug;
  for (Block *block: blockList) {
    block->setContainerSequence(this);
    block->compile(debug);
  }
}

bool Sequence::debugEnabled()
{
  return debug;
}

void Sequence::print()
{
  //if(isOrigin && !isCompiled) throw InvalidOperation("compile before print.");
  if(isOrigin)cout << string("Origin(") + name + string(")") << endl;
  for(Block* block : blockList)
  {
    block->print();
  }
  if(isOrigin)cout << string("Origin(") + name + string(")") << endl;
}

void seq::Block::printDebug(string msg, bool line)
{
  if (!containerSequence->debugEnabled()) return;
  cout << "|";
  /*when this function is used in a lambda expression that is passed as constructor's parameter, It captures 'this' as block that is one level lower than expected.
   * (so this->printDebug("") prints debug message at one hierarchy lower than I would like.)
   * !line==1 when line is false. It's a little trick to print in-algorithm debug messages in the right hierarchy level.
   */
  for (unsigned int i = 0; i < containerSequence->getHierarchyLevel() + !line; i++) cout << " |";
  if (line)cout << "___";
  else cout << "   ";
  cout << msg << endl;
}

seq::Block::Block()
{
  containerSequence = nullptr;
}

string seq::Block::generateDebugName()
{
  return string("Block(") + typeid(*this).name() + string(")");
}

void Block::compile(bool debug)
{

}

void Block::setContainerSequence(Sequence *sequence)
{
  this->containerSequence = sequence;
}

void Block::startCallback()
{

}
void Block::print()
{
  printDebug(generateDebugName(), true);
}
