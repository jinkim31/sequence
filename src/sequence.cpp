#include "../inc/sequence.h"

using namespace seq;

Sequence::Sequence()
{
	timeLastUpdate = chrono::system_clock::now();
	currentStep = 0;
	finished = false;
	running = false;
	steadyStep = false;
	requestedStep = 0;
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
			//cout<<"end of sequence"<<endl;
			return true;
		}

		//Prpcess a block.

		//Calculate time duration since last spin.
		chrono::duration<double> timeUpdateDelta = chrono::system_clock::now() - timeLastUpdate;
		timeLastUpdate = chrono::system_clock::now();
		Block::SpinInfo spinInfo;
		spinInfo.timeDelta = timeUpdateDelta.count();

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

}

void Sequence::enableSteadyStep(bool value)
{
	steadyStep =value;
}

seq::Timeout::Timeout(double timeSec, function<void(void)> timeoutHandler)
{
	timeElapsed = 0.0;
	this->timeSec = timeSec;
	this->timeoutHandler = timeoutHandler;
}
seq::Timeout::Timeout(double timeSec) : Timeout(timeSec, NULL){}
seq::Timeout::Timeout() :Timeout(0.0) {}

void Timeout::setTime(double timeSec) {this->timeSec = timeSec;}
void seq::Timeout::setTimeoutHandler(function<void(void)> timeoutHandler)
{
	this->timeoutHandler = timeoutHandler;
}
void Timeout::reset() { timeElapsed = 0.0; }
bool Timeout::addTime(double timeDelta)
{
	if (timeSec < 0.0) return false;
	timeElapsed += timeDelta;
	//cout << timeElapsed << "/" << timeSec << endl;
	if (timeElapsed >= timeSec)
	{
		if (timeoutHandler != NULL) timeoutHandler();
		return true;
	}
	return false;
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

seq::block::Delay::Delay(double timeSeconds)
{
	timeout.setTime(timeSeconds);
}

bool seq::block::Delay::spinOnce(SpinInfo spinInfo)
{
	return timeout.addTime(spinInfo.timeDelta);
}

void seq::block::Delay::reset()
{
	timeout.reset();
}

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

seq::block::WaitFor::WaitFor(function<bool(void)> breakCondition, double timeout, function<void(void)> timeoutHandler)
{
	this->breakCondition = breakCondition;
	this->timeout.setTime(timeout);
	this->timeout.setTimeoutHandler(timeoutHandler);
}
seq::block::WaitFor::WaitFor(function<bool(void)> breakCondition, double timeout) :WaitFor(breakCondition, timeout, NULL){}

block::WaitFor::WaitFor(function<bool(void)> breakCondition) : WaitFor(breakCondition, -1.0){}

bool block::WaitFor::spinOnce(SpinInfo spinInfo)
{
	if (timeout.addTime(spinInfo.timeDelta))
	{
		return true;
	}
	
	return breakCondition();
}
void block::WaitFor::reset(){}

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

seq::block::LoopSequence::LoopSequence(Sequence* sequence, function<bool(void)> breakCondition, double timeout, function<void(void)> timeoutHandler) : SequenceBlock(sequence)
{
	this->breakCondition = breakCondition;
	this->timeout.setTime(timeout);
	this->timeout.setTimeoutHandler(timeoutHandler);
}

seq::block::LoopSequence::LoopSequence(Sequence* sequence, function<bool(void)> breakCondition, double timeout) : LoopSequence(sequence, breakCondition, timeout, NULL) {}

block::LoopSequence::LoopSequence(Sequence* sequence, function<bool(void)> breakCondition) : LoopSequence(sequence, breakCondition, -1.0) {}

bool block::LoopSequence::spinOnce(SpinInfo spinInfo)
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