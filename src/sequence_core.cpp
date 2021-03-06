
#include "../inc/sequence_core.h"

using namespace seq;

vector<Sequence *> Sequence::sequenceList;
shared_ptr<Block> Sequence::ongoingBlock;
double Sequence::timeLastUpdate;
Broadcast Sequence::broadcast;

seq::BroadcastCondition::BroadcastCondition(const string& msg) : msg(msg)
{
    triggered = false;
    Sequence::broadcast.addBroadcastListener(make_shared<Broadcast::BroadcastListener>(msg, [&]{triggered= true;}));
}

bool seq::BroadcastCondition::evaluate()
{
    return triggered;
}

void BroadcastCondition::reset()
{
    triggered = false;
}

Sequence::~Sequence()
{
    clear();
}

void seq::Sequence::setHierarchyLevel(unsigned int value)
{
    hierarchyLevel = value;
}

unsigned int seq::Sequence::getHierarchyLevel() const
{
    return hierarchyLevel;
}

void Sequence::add(shared_ptr<Block> block)
{
    if (isCompiled)
    {
        return;
    }
    block->setContainerSequence(this);
    blockList.push_back(block);
}

void Sequence::clear()
{
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
                if (debug)Sequence::printDebug(blockList[currentStep]->generateDebugName(), true);
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

bool Sequence::isRunning() const
{
    return running;
}

bool Sequence::isFinished() const
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
    for (const shared_ptr<Block>& block: blockList)
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
    else cout << string("Inner(") + name + string(")") << endl;
    for (const shared_ptr<Block>& block: blockList)
    {
        block->print();
    }
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

void Sequence::printDebug(const string& msg, bool line)
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
    SpinInfo spinInfo{};
    spinInfo.timeDelta = loopRate;

    for (Sequence *sequence: sequenceList)
    {
        if (sequence->update(spinInfo))
        {

        }
    }
}

void Sequence::startSequence(const string& sequenceName)
{
    for(Sequence* sequence : sequenceList)
    {
        if(sequence->getName() == sequenceName)
        {
            sequence->start();
        }
    }
}

Sequence* Sequence::getSequenceByName(const string& name)
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

void Sequence::initVariables()
{
    vector<shared_ptr<IVariable>>::iterator iter;

    for(iter = variableList.begin(); iter != variableList.end(); iter++)
    {
        (*iter)->init();
    }
}

Sequence *Sequence::thisSequence()
{
    if(ongoingBlock == nullptr)
    {
        throw SequenceComponentNullException("Cannot determine /'thisSequence/' now. use object name or pointer to directly access the sequence.");
    }
    return ongoingBlock->getContainerSequence();
}

void Sequence::setParentSequence(Sequence* sequence)
{
    this->parentSequence = sequence;
}

seq::Block::Block()
{
    containerSequence = nullptr;
}

string seq::Block::generateDebugName()
{
    return "Unknown block";
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

Sequence* Block::getContainerSequence()
{
    return containerSequence;
}

Broadcast::BroadcastListener::BroadcastListener(const string &msg, function<void(void)> broadcastHandler): msg(msg), broadcastHandler(broadcastHandler)
{
}

void Broadcast::BroadcastListener::notify(const string& msg)
{
    if (this->msg == msg)broadcastHandler();
}

void Broadcast::addBroadcastListener(const shared_ptr<BroadcastListener>& listener)
{
    listeners.push_back(listener);
}

void Broadcast::broadcast(const string& msg)
{
    vector<shared_ptr<BroadcastListener>>::iterator iter;
    for (iter = listeners.begin(); iter != listeners.end(); iter++) (*iter)->notify(msg);
}
