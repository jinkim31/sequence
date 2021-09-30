#ifndef SEQUENCE_H
#define SEQUENCE_H

#include <iostream>
#include <vector>
#include <queue>
#include <functional>
#include <string>
#include <cstdarg>
#include <chrono>
#include <stdexcept>
#include "util.h"

using namespace std;

namespace seq
{

/*Exceptions*/
class InvalidOperation : runtime_error
{
public:
    InvalidOperation(string msg) : runtime_error(msg.c_str())
    {};
};

/*Structs*/
struct SpinInfo
{
    double timeDelta;
    string broadcastMsg;
};

/*Classes*/
class Block;//forward declaration

class BroadcastCondition : public Condition
{
private:
    string msg;
public:
    BroadcastCondition(string msg);
    virtual bool evaluate();
};

class Sequence
{
private:
    vector<Block *> blockList;
    string name;
    bool isOrigin;
    int currentStep;
    bool running;
    bool finished;
    bool steadyStep;
    bool debug;
    unsigned int hierarchyLevel;
    bool isCompiled;
    static double timeLastUpdate;
    static queue<string> broadcastQueue;
    static string currentBroadcast;

    //Recurse through variadic arguments.
    template<typename... BlockPtrs>
    void addArgs(string name, BlockPtrs... blockPtrs)
    {
        this->name = name;
        addArgs(blockPtrs...);
    }

    template<typename... BlockPtrs>
    void addArgs(Block *blockPtr, BlockPtrs... blockPtrs)
    {
        add(blockPtr);
        addArgs(blockPtrs...);
    }

    //End of recursion. Push final argument.
    void addArgs(Block *blockPtr)
    {
        add(blockPtr);
        return;
    }

    static vector<Sequence *> sequenceList;
    static Block *ongoingBlock;
public:
    template<typename... BlockPtrs>
    Sequence(BlockPtrs... blockPtrs)
    {
        timeLastUpdate = -1;
        currentStep = 0;
        finished = false;
        running = false;
        steadyStep = false;
        hierarchyLevel = 0;
        isOrigin = false;
        isCompiled = false;
        name = "unnamed sequence";
        addArgs(blockPtrs...);
    }

    ~Sequence();

    void compile(bool debug = false);

    void init(bool debug); //sets hierarchy for debug
    void setHierarchyLevel(unsigned int value);

    void setAsOrigin(bool value)
    { isOrigin = value; }

    string getName()
    { return name; }

    unsigned int getHierarchyLevel();

    void add(Block *block);

    void clear();

    bool update(SpinInfo spinInfo);

    void start();  //start or resume the sequence.
    void suspend();  //pause the sequence without resetting the progress.
    void stop();  //stop the sequence.
    bool isRunning();

    bool isFinished();

    void enableSteadyStep(bool value);

    bool debugEnabled();

    void print();

    static void spinOnce();

    static void spinOnce(double loopRate);

    static void broadcast(string msg);

    static void printDebug(string msg, bool line = false);

    static string getCurrentBroadcast();

    static void startSequence(string sequenceName);
};

class Block
{
protected:
    Sequence *containerSequence;
public:

    Block();

    void setContainerSequence(Sequence *sequence);

    Sequence *getContainerSequence();

    virtual string generateDebugName();

    virtual bool
    update(SpinInfo spinInfo) = 0;  //returns true when block is 'finished' and good to move on to the next block.
    virtual void reset() = 0;  //Sequence calls reset() before processing each block.
    virtual void startCallback(); //callback that is called once when the block is starting.
    virtual void endCallback();

    virtual void init(bool debug);

    virtual void print();
};
}
#endif