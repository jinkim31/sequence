#include "../inc/sequence_core.h"

using namespace seq;

vector<Sequence *> Sequence::sequenceList;
Block *Sequence::ongoingBlock;
double Sequence::timeLastUpdate;
queue<string> Sequence::broadcastQueue;
string Sequence::currentBroadcast;

seq::BroadcastCondition::BroadcastCondition(string msg) : msg(msg)
{
}

bool seq::BroadcastCondition::evaluate()
{
    return Sequence::getCurrentBroadcast() == msg;
}


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
    if (isCompiled)
    {
        return;
    }
    blockList.push_back(block);
}

void Sequence::clear()
{
    //cout<<"clearing sequence"<<endl;
    for (Block *block: blockList)
    {
        delete (block);
    }
    blockList.clear();
    currentStep = 0;
}

bool Sequence::update(SpinInfo spinInfo)
{
    if (!running) return false;

    while (true)
    {
        //cout<<"calling "<<blockList[currentStep]->generateDebugName()<<endl;
        if (blockList[currentStep]->update(spinInfo))
        {
            //step
            blockList[currentStep]->endCallback();
            currentStep++;
            //reset the block, call callback
            if (currentStep < blockList.size())
            {
                Sequence::ongoingBlock = blockList[currentStep];
                if (debug)Sequence::printDebug(blockList[currentStep]->generateDebugName() +" layer:"+ to_string(hierarchyLevel), true);
                blockList[currentStep]->reset();
                blockList[currentStep]->startCallback();
            }
            //End of the sequence.
            else
            {
                stop();
                finished = true;
                running = false;
                if (isOrigin && debug)cout << string("Sequence terminated.(") + name + string(")") << endl;
                return true;
            }
        }
        else
        {
            //return when false returned.
            return false;
        }

        //exit loop.
        if (steadyStep) break;
    }

    return false;
}

void Sequence::start()
{
    if (!blockList.empty())
    {
        if (isOrigin && debug)cout << string("Sequence started.(") + name + string(")") << endl;

        running = true;
        finished = false;
        Sequence::ongoingBlock = blockList[currentStep];
        if(debug)Sequence::printDebug(blockList[currentStep]->generateDebugName(), true);
        blockList[currentStep]->reset();
        blockList[currentStep]->startCallback();
    }
}

void Sequence::suspend()
{
    running = false;
}

void Sequence::stop()
{
    if(currentStep < blockList.size())
    {
        blockList[currentStep]->endCallback();
    }
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

void Sequence::init(bool debug)
{
    if (isCompiled)
    {
        return;
    }

    this->debug = debug;
    for (Block *block: blockList)
    {
        block->setContainerSequence(this);
        block->init(debug);
    }
    isCompiled = true;
}

bool Sequence::debugEnabled()
{
    return debug;
}

void Sequence::print()
{
    //if(isOrigin && !isCompiled) throw InvalidOperation("init before print.");
    if (isOrigin)cout << string("Origin(") + name + string(")") << endl;
    for (Block *block: blockList)
    {
        block->print();
    }
    if (isOrigin)cout << string("Origin(") + name + string(")") << endl;
}

void Sequence::spinOnce()
{
    //Calculate time duration since last spin.
    double currentTime = chrono::system_clock::now().time_since_epoch().count() / 1000000000.0;

    if (timeLastUpdate == -1)
    {
        timeLastUpdate = currentTime;
    }

    spinOnce(currentTime - timeLastUpdate);

    timeLastUpdate = currentTime;
}

void Sequence::compile(bool debug)
{
    this->isOrigin = true;
    Sequence::sequenceList.push_back(this);
    init(debug);
}

void Sequence::printDebug(string msg, bool line)
{
    if (!ongoingBlock->getContainerSequence()->debugEnabled()) return;
    cout << "|";
    for (unsigned int i = 0; i < ongoingBlock->getContainerSequence()->getHierarchyLevel(); i++) cout << " |";
    if (line)cout << "___";
    else cout << ">  ";
    cout << msg << endl;
}

void Sequence::spinOnce(double loopRate)
{
    SpinInfo spinInfo;
    spinInfo.timeDelta = loopRate;
    if(broadcastQueue.empty())
    {
        spinInfo.broadcastMsg="";
    }
    else
    {
        spinInfo.broadcastMsg = broadcastQueue.front();
        currentBroadcast = broadcastQueue.front();
        broadcastQueue.pop();
    }

    for (Sequence *sequence: sequenceList)
    {
        if (sequence->update(spinInfo))
        {

        }
    }
}

void Sequence::broadcast(string msg)
{
    Sequence::broadcastQueue.push(msg);
}

string Sequence::getCurrentBroadcast()
{
    return currentBroadcast;
}

void Sequence::startSequence(string sequenceName)
{
    for(Sequence* sequence : sequenceList)
    {
        if(sequence->getName() == sequenceName)
        {
            sequence->start();
        }
    }
}

Sequence* Sequence::getSequenceByName(string name)
{
    for(Sequence* sequence : sequenceList)
    {
        if(sequence->getName() == name)
        {
            return sequence;
        }
    }

    return nullptr;
}


seq::Block::Block()
{
    containerSequence = nullptr;
}

string seq::Block::generateDebugName()
{
    return string("Unknown block");
}

void Block::init(bool debug)
{

}

void Block::setContainerSequence(Sequence *sequence)
{
    this->containerSequence = sequence;
}

void Block::startCallback()
{

}

void Block::endCallback()
{

}

void Block::print()
{
    Sequence::printDebug(generateDebugName(), true);
}

Sequence *Block::getContainerSequence()
{
    return containerSequence;
}
