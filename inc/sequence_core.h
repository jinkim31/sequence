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

/***********************************************************************************************************************
 * Exceptions
 **********************************************************************************************************************/

class InvalidOperationException : runtime_error
{
public:
    explicit InvalidOperationException(string msg) : runtime_error(msg.c_str())
    {};
};

class NoSuchVariableException : runtime_error
{
public:
    explicit NoSuchVariableException(string msg) : runtime_error(msg.c_str())
    {};
};

/***********************************************************************************************************************
 * Structs
 **********************************************************************************************************************/

struct SpinInfo
{
    double timeDelta;
};

/***********************************************************************************************************************
 * Classes
 **********************************************************************************************************************/

class Block;        //forward declaration
class Broadcast;    //forward declaration

class IVariable
{
private:
    string id;
public:
    explicit IVariable(string id) : id(id){}
    virtual ~IVariable() = default;
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
    Variable(const string &id, const T &initialVal): IVariable(id), initialVal(initialVal){val = initialVal; initValueDefined = true;}
    explicit Variable(const string &id) : IVariable(id){initValueDefined = false;};
    ~Variable() override = default;
    void set(const T &val){this->val = val;}
    T& get(){return val;};
    void init() override{if(initValueDefined) val = initialVal;}
};

class BroadcastCondition : public Condition
{
private:
    string msg;
    bool triggered;
public:
    explicit BroadcastCondition(const string& msg);
    ~BroadcastCondition() = default;
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
    Sequence* parentSequence;

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
        debug = false;
        isCompiled = false;
        name = "unnamed sequence";
        parentSequence = nullptr;
    }
    virtual ~Sequence();

    template<typename... BlockPtrs>
    explicit Sequence(BlockPtrs... blockPtrs) : Sequence()
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

    unsigned int getHierarchyLevel() const;

    void add(shared_ptr<Block> block);

    void clear();

    bool update(SpinInfo spinInfo);

    void start();  //start or resume the sequence.
    void suspend();  //pause the sequence without resetting the progress.
    void stop();  //stop the sequence.
    bool isRunning() const;

    bool isFinished() const;

    void enableSteadyStep(bool value);

    bool debugEnabled();

    void print();

    void setParentSequence(Sequence* sequence);

    template<typename T>
    void addVariable(const string &id, const T &initValue)
    {
        auto variableShared = make_shared<Variable<T>>(id, initValue);
        variableList.push_back(variableShared);
    }

    template<typename T>
    void addVariable(const string &id)
    {
        auto variableShared = make_shared<Variable<T>>(id);
        variableList.push_back(variableShared);
    }

    template<typename T>
    T& getSequenceVariable(string id)
    {
        //cout<<"searching sequence "<<this->name<<endl;
        vector<shared_ptr<IVariable>>::iterator iter;
        //cout<<variableList.size()<<" variables (get)"<<endl;
        for(iter = variableList.begin(); iter != variableList.end(); iter++)
        {
            if((*iter)->getId() == id)
            {
                //cout<<"found variable"<<endl;
                return dynamic_pointer_cast<Variable<T>>(*iter)->get();
            }
        }

        if(parentSequence == nullptr)
        {
            std::cerr<<"[ Sequence ] No variable with ID "<<id<<" found."<<endl;
            throw NoSuchVariableException("No variable with ID " + id + " found.");
        }

        return parentSequence->getSequenceVariable<T>(id);
    }

    static void spinOnce();

    static void spinOnce(double loopRate);

    static void printDebug(const string& msg, bool line = false);

    static void startSequence(const string& sequenceName);

    static Sequence* getSequenceByName(const string& name);

    template<typename T>
    static T& getVariable(string id)
    {
        return thisSequence()->getSequenceVariable<T>(id);
    }

    static Sequence* thisSequence();
    static Broadcast broadcast;
};

class Block
{
protected:
    Sequence *containerSequence;
public:

    Block();
    virtual ~Block()= default;

    virtual void setContainerSequence(Sequence *sequence);

    Sequence *getContainerSequence();

    virtual string generateDebugName();

    virtual bool
    update(SpinInfo spinInfo) = 0;      //returns true when block is 'finished' and good to move on to the next block.
    virtual void reset() = 0;           //Sequence calls reset() before processing each block.
    virtual void startCallback();       //callback that is called once when the block is starting.
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
        BroadcastListener(const string &msg, function<void(void)> broadcastHandler);
        ~BroadcastListener() = default;
        void notify(const string& msg);
    };


    void addBroadcastListener(const shared_ptr<BroadcastListener>& listener);

    void broadcast(const string& msg);
};

}
#endif