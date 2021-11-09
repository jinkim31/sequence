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
#include <memory>
#include "sequence_util.h"

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
};

/*Classes*/
class Block;//forward declaration
class Broadcast;;//forward declaration

class IVariable
{
private:
    string id;
public:
    IVariable(string id) : id(id){}
    virtual ~IVariable() {}
    virtual void init() = 0;
    string getId(){return id;}
};

template <typename T>
class Variable : public IVariable
{
private:
    T val;
    T initialVal;
    bool initValueDefined;
public:
    Variable(string id, T initialVal): IVariable(id), initialVal(initialVal){val = initialVal; initValueDefined = true;}
    Variable(string id) : IVariable(id){initValueDefined = false;};
    virtual ~Variable(){}
    void set(T val){this->val = val;}
    T& get(){return val;};
    void init() override{if(initValueDefined) val = initialVal;}
};

class BroadcastCondition : public Condition
{
private:
    string msg;
    bool triggered;
public:
    BroadcastCondition(string msg);
    ~BroadcastCondition();
    bool evaluate() override;

    void reset() override;
};

class Sequence
{
private:
    vector<shared_ptr<Block>> blockList;
    vector<shared_ptr<IVariable>> variableList;
    string name;
    bool isOrigin;
    int currentStep;
    bool running;
    bool finished;
    bool steadyStep;
    bool debug;
    unsigned int hierarchyLevel;
    bool isCompiled;

    //Recurse through variadic arguments.
    template<typename... BlockPtrs>
    void addArgs(string name, BlockPtrs... blockPtrs)
    {
        this->name = name;
        addArgs(blockPtrs...);
    }

    template<typename... BlockPtrs>
    void addArgs(shared_ptr<Block> blockPtr, BlockPtrs... blockPtrs)
    {
        add(blockPtr);
        addArgs(blockPtrs...);
    }

    //End of recursion. Push final argument.
    void addArgs(shared_ptr<Block> blockPtr)
    {
        add(blockPtr);
        return;
    }

    void initVariables();
    static double timeLastUpdate;
    static vector<Sequence *> sequenceList;
    static shared_ptr<Block> ongoingBlock;

public:
    Sequence()
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
    }
    virtual ~Sequence();

    template<typename... BlockPtrs>
    Sequence(BlockPtrs... blockPtrs) : Sequence()
    {
        addArgs(blockPtrs...);
    }

    template<typename... BlockPtrs>
    void compose(BlockPtrs... blockPtrs)
    {
        addArgs(blockPtrs...);
    }

    void compile(bool debug = false);

    void init(bool debug); //sets hierarchy for debug
    void setHierarchyLevel(unsigned int value);

    void setAsOrigin(bool value)
    { isOrigin = value; }

    string getName()
    { return name; }

    unsigned int getHierarchyLevel();

    void add(shared_ptr<Block> block);

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

    void addVariable(shared_ptr<IVariable> variableUnique);

    template<typename T>
    shared_ptr<Variable<T>> getVariable(string id)
    {
        shared_ptr<IVariable> g;

        vector<shared_ptr<IVariable>>::iterator iter;
        //cout<<variableList.size()<<" variables (get)"<<endl;
        for(iter = variableList.begin(); iter != variableList.end(); iter++)
        {
            if((*iter)->getId() == id)
            {
                //cout<<"found variable"<<endl;
                return dynamic_pointer_cast<Variable<T>>(*iter);
            }
        }

        std::cerr<<"[ Sequence ] No variable with ID "<<id<<" found."<<endl;
        return shared_ptr<Variable<T>>(nullptr);
    }

    static void spinOnce();

    static void spinOnce(double loopRate);

    static void printDebug(string msg, bool line = false);

    static void startSequence(string sequenceName);

    static Sequence* getSequenceByName(string name);

    static Sequence* thisSequence();
    static Broadcast broadcast;
};

class Block
{
protected:
    Sequence *containerSequence;
public:

    Block();
    virtual ~Block(){}

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

class Broadcast
{
public:
    class BroadcastListener;
private:
    vector<shared_ptr<BroadcastListener>> listeners;
public:
    class BroadcastListener
    {
    private:
        function<void(void)> broadcastHandler;
        string msg;
    public:
        BroadcastListener(string &msg, function<void(void)> broadcastHandler);

        void notify(string msg);
    };

    void addBroadcastListener(shared_ptr<BroadcastListener> listener);

    void broadcast(string msg);
};

}
#endif