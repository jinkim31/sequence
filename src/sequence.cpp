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

void Sequence::add(Block* block)
{
	blockList.push_back(block);
	block->setContainerSequence(this);
}

void Sequence::clear()
{
	//cout<<"clearing sequence"<<endl;
	for(Block* block : blockList)
	{
		delete(block);
	}
	blockList.clear();
	currentStep = 0;
}

bool Sequence::spinOnce()
{
	//cout<<"Running:"<<running<<" blocks:"<<currentStep<<"/"<<blockList.size()<<endl;
	if(!running) return false;

	while(true)
	{
		//Calculate time duration since last spin.
		chrono::duration<double> timeUpdateDelta = chrono::system_clock::now() - timeLastUpdate;
		timeLastUpdate = chrono::system_clock::now();
		Block::SpinInfo spinInfo;
		spinInfo.timeDelta = timeUpdateDelta.count();

		requestedStep = 1;
		if(blockList[currentStep]->spinOnce(spinInfo))
		{
			//step
			currentStep += requestedStep;
			//notify start of the block
			if ((size_t)currentStep < blockList.size())
			{
				blockList[currentStep]->printDebugMsg();
				blockList[currentStep]->reset();
				blockList[currentStep]->notifyStart();
			}
			//Check for end of the sequence.
			else
			{
				reset();
				finished = true;
				if (isOrigin)cout << string("Sequence terminated.(") + name + string(")") << endl;
				return true;
			}
		}
		else
		{
			//return when false returned.
			return false;
		}

		//exit loop.
		if(steadyStep) break;
	}

	return false;
}

void Sequence::step(int value)
{
	requestedStep += value;
}

void Sequence::start()
{
	if(!blockList.empty())
	{
		if (isOrigin)cout << string("Sequence started.(") + name + string(")") << endl;

		running = true;
		finished = false;
		blockList[0]->printDebugMsg();
		blockList[0]->reset();
		blockList[0]->notifyStart();
	}
}
void Sequence::suspend()
{
	running = false;
}
void Sequence::reset()
{
	running = false;
	currentStep = 0;
	blockList[currentStep]->reset();
    timeLastUpdate = chrono::system_clock::now();
}

bool Sequence::isRunning()
{
	return running;
}

bool Sequence::isFinished()
{
	return finished;
}
void Sequence::insertInitiator()
{

}

void Sequence::enableSteadyStep(bool value)
{
	steadyStep =value;
}

void Sequence::compile()
{

}

void seq::Block::printDebugMsg()
{
  cout << "|";
  for (unsigned int i = 0; i < containerSequence->getHierarchyLevel(); i++) cout << " |";
  cout << "___";
  cout << " " <<getBlockDescription()<< endl;
}

seq::Block::Block()
{
  containerSequence = nullptr;
}

void seq::Block::setContainerSequence(Sequence* sequence)
{
  containerSequence = sequence;
}

string seq::Block::getBlockDescription()
{
  return string("Block(") + typeid(*this).name() + string(")");
}