#ifndef SEQUENCE_H
#define SEQUENCE_H

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

class Block;//forward declaration

class Sequence
{
private:
	vector<Block*> blockList;
	string name;
	bool debug;
	bool isOrigin;
	int currentStep;
	bool running;
	bool finished;
	bool steadyStep;
	int requestedStep;
	unsigned int hierarchyLevel;
    bool isCompiled;
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
        isOrigin = true;
  		addArgs(blockPtrs...);
	}

	~Sequence();

  void compile();
	void setHierarchyLevel(unsigned int value);
	void setIsMaster(bool value) { isOrigin = value; }
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
class Block
{
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
}
#endif
