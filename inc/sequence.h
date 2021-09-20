#ifndef SEQUENCE_H_
#define SEQUENCE_H_

#include <iostream>
#include <vector>
#include <tuple>
#include <functional>
#include <string>
#include <cstdarg>
#include <chrono>

using namespace std;

namespace seq
{
class Sequence;

class Block
{
private:
protected:
	Sequence* containerSequence;
public:
	void printDebugMsg();
	struct SpinInfo
	{
		double timeDelta;
	};
	Block();
	void setContainerSequence(Sequence* sequence);
	virtual bool spinOnce(SpinInfo spinInfo) = 0;	//returns true when block is 'finished' and good to move on to the next block.
	virtual void reset() = 0;	//Sequence calls reset() and resets the block before moving on to the next one to make sure it can be used again.
	virtual void notifyStart() = 0;
	virtual string getBlockDescription();
};

class Sequence
{
private:
	vector<Block*> blockList;
	string name;
	bool debug;
	bool isMaster;
	int currentStep;
	bool running;
	bool finished;
	bool steadyStep;
	int requestedStep;
	unsigned int hierarchyLevel;
	chrono::system_clock::time_point timeLastUpdate;

	//Recurse through variadic arguments.
	template <typename... BlockPtrs>
	void addArgs(string name, BlockPtrs... blockPtrs)
	{
		this->name = name;
		addArgs(blockPtrs...);
	}
	template <typename... BlockPtrs>
	void addArgs(Block* blockPtr, BlockPtrs... blockPtrs)
	{
		add(blockPtr);
		addArgs(blockPtrs...);
	}
	//End of recursion. Push final argument.
	void addArgs(Block* blockPtr)
	{
		add(blockPtr);
		return; 
	}
public:
	template <typename... BlockPtrs>
	Sequence(BlockPtrs... blockPtrs)
	{
		timeLastUpdate = chrono::system_clock::now();
		currentStep = 0;
		finished = false;
		running = false;
		steadyStep = false;
		requestedStep = 0;
		hierarchyLevel = 0;
		isMaster = true;
  		addArgs(blockPtrs...);
	}

	~Sequence();

	void setHierarchyLevel(unsigned int value);
	void setIsMaster(bool value) {isMaster = value; }
	string geName() { return name; }
	unsigned int getHierarchyLevel();
	void add(Block* block);
	void clear();
	bool spinOnce();	//update ongoing block or advance to the next one. Returns true when the sequence is finished.
	void step(int value);	//step blocks forcefully in sequence by value(nagative to backtrack).
	void start();	//activate the sequence.
	void suspend();	//pause the sequence without resetting the progress.
	void reset();	//stop and reset the sequence.
	bool isRunning();
	bool isFinished();	//true when the sequence is finished successfully. Its return value can be reset to false using stop() or restarting the sequence using start().
	void insertInitiator();	//insert block that waits for publisher ready.(0.1sec delay)
	void enableSteadyStep(bool value);
};

class Timeout
{
private:
	double timeSec;
	double timeElapsed;
	function<void(void)> timeoutHandler;
	bool timeoutHandlerAvailable;
public:
	Timeout(double timeSec, function<void(void)> timeoutHandler);
	Timeout(double timeSec);
	Timeout();
	void setTime(double timeSec);
	void setTimeoutHandler(function<void(void)> timeoutHandler);
	void reset();
	bool addTime(double timeDelta);
	double getTimeSec() { return timeSec; }
};

namespace block
{
class Print : public Block
{
private:
	std::string text;
public:
	Print(const std::string& text);
	virtual void notifyStart(){}
	virtual bool spinOnce(SpinInfo spinInfo);
	virtual void reset();
};

class Delay : public Block
{
private:
	Timeout timeout;
public:
	Delay(double timeSeconds);
	virtual void notifyStart(){}
	virtual bool spinOnce(SpinInfo spinInfo);
	virtual void reset();
	virtual string getBlockDescription() { return string("Delay(")+to_string(timeout.getTimeSec())+string(")"); }
};

class Function : public Block
{
private:
	function<void(void)> func;
public:
	Function(function<void(void)> func);
	virtual void notifyStart(){}
	virtual bool spinOnce(SpinInfo spinInfo);
	virtual void reset();
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
	virtual void notifyStart(){}
	virtual bool spinOnce(SpinInfo spinInfo);
	virtual void reset();
};

class SequenceBlock : public Block
{
private:
protected:
	Sequence* sequence;
public:
	SequenceBlock(Sequence *sequence);
	~SequenceBlock();
	virtual void notifyStart();
	virtual bool spinOnce(SpinInfo spinInfo);
	virtual void reset();
	virtual string getBlockDescription() { return string("Sequence(") + sequence->geName() + string(")"); }
};

class LoopSequence : public SequenceBlock
{
private:
	Timeout timeout;
	function<bool(void)> breakCondition;
public:
	LoopSequence(Sequence* sequence, function<bool(void)> breakCondition, double timeout, function<void(void)> timeoutHandler);
	LoopSequence(Sequence* sequence, function<bool(void)> breakCondition, double timeout);
	LoopSequence(Sequence* sequence, function<bool(void)> breakCondition);
	virtual bool spinOnce(SpinInfo spinInfo);
};

}
}
#endif
