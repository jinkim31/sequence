#include "../inc/sequence.h"

using namespace seq;

Sequence::Sequence()
{
	timeLastUpdate = chrono::system_clock::now();
	currentStep = 0;
	finished = false;
	running = false;
	steadyStep = true;
}

Sequence* Sequence::clone()
{
	Sequence *sequence = new Sequence();
	for(Block* block :blockList)
	{
		sequence->add(block->clone());
	}

	return sequence;
}

Sequence::~Sequence()
{
	clear();
}

void Sequence::add(Block* block)
{
	blockList.push_back(block);
}

void Sequence::clear()
{
	//cout<<"clearing sequence"<<endl;
	for(Block* block : blockList)
	{
		delete(block);
	}
	blockList.clear();
	blockList.resize(0);
	currentStep = 0;
}

bool Sequence::spinOnce()
{
	//cout<<"Running:"<<running<<" blocks:"<<currentStep<<"/"<<blockList.size()<<endl;
	if(!running) return false;

#ifdef SEQUENCE_ROS_ENV
	while(ros::ok)	//enable ros node shutdown using ctrl+c in terminal
#else
	while(true)
#endif
	{
		//Check for end of the sequence.
		if(blockList.empty() || currentStep == blockList.size())
		{
			reset();
			finished = true;
			cout<<"end of sequence"<<endl;
			return true;
		}

		//Prpcess a block.

		//Calculate time duration since last spin.
		chrono::duration<double> timeUpdateDelta = chrono::system_clock::now() - timeLastUpdate;
		timeLastUpdate = chrono::system_clock::now();
		Block::SpinInfo spinInfo;
		spinInfo.timeDelta = timeUpdateDelta;

		requestedStep = 1;
		if(blockList[currentStep]->spinOnce(spinInfo))
		{
			blockList[currentStep]->reset();
			//step
			currentStep += requestedStep;
			//notify start of the block
			if(currentStep < blockList.size()) blockList[currentStep]->notifyStart();
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
		running = true;
		finished = false;
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
	blockList.insert(blockList.begin(), new block::Delay(1));
}

void Sequence::enableSteadyStep(bool value)
{
	steadyStep =value;
}

void Block::reset(){}
Block* Block::clone()
{
	Block *result = doClone();
	//cout<<"cloning block("<<typeid(this).name()<<"->"<<typeid(result).name()<<")"<<endl;
	if(typeid(result) != typeid(this))
	{
		cout<<"DoCloneNotImplmentedException"<<endl;
		return nullptr;
	}

	return result;
}

block::Print::Print(const std::string& text)
{
	this->text = text;
	this->text = text;
}
bool block::Print::spinOnce(SpinInfo spinInfo)
{
	cout << text << endl;
	return true;
}
void block::Print::reset(){}
Block* block::Print::doClone(){return new Print(text);}

block::Delay::Delay(int timeSeconds)
{
	this->timeSeconds = timeSeconds;
	this->timeElapsed = 0;
}

bool block::Delay::spinOnce(SpinInfo spinInfo)
{
	timeElapsed += spinInfo.timeDelta.count();
	return (timeElapsed > timeSeconds);
}
void block::Delay::reset()
{
	timeElapsed = 0;
}
Block* block::Delay::doClone(){return new Delay(timeSeconds);}

block::Function::Function(function<void(void)> func)
{
	this->func = func;
}
bool block::Function::spinOnce(SpinInfo spinInfo)
{
	func();
	return true;
}
void block::Function::reset(){}
Block* block::Function::doClone(){return new Function(func);}

block::WaitFor::WaitFor(function<bool(void)> func)
{
	this->func = func;
}
bool block::WaitFor::spinOnce(SpinInfo spinInfo)
{
	return func();
}
void block::WaitFor::reset(){}
Block* block::WaitFor::doClone(){return new WaitFor(func);}

block::SequenceBlock::SequenceBlock(Sequence *sequence)
{
    this->sequence = sequence;
	sequence = nullptr;
}
block::SequenceBlock::~SequenceBlock()
{
	if(sequence != nullptr) delete(sequence);
}
bool block::SequenceBlock::spinOnce(SpinInfo spinInfo)
{
    return sequence->spinOnce();
}
void block::SequenceBlock::reset()
{
    sequence->reset();
    sequence->start();
}
Block* block::SequenceBlock::doClone()
{
	return new SequenceBlock(sequence);
}
