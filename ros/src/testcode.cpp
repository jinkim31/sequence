#include <iostream>
#include <ros/ros.h>
#include "../inc/sequence_ros.h"

using namespace std;
using namespace seq;

class TestBlock : public block::SequenceBlock
{
public:
    TestBlock():SequenceBlock()
    {
        shared_ptr<Sequence> sequence = make_shared<Sequence>();
        sequence->compose("test", make_shared<block::Debug>("hi"));
        setSequence(sequence);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sequence_test_node");
    ros::NodeHandle nh;

    Sequence sequence;
    sequence.addVariable(make_shared<Variable<int>>("cnt",0));
    sequence.compose
    (
    make_shared<block::LoopSequence>(make_shared<BroadcastCondition>("break"), make_shared<Sequence>
    (
        make_shared<block::Function>([&]
        {
            int& cnt = sequence.getVariable<int>("cnt") ->get();
            cnt++;
            Sequence::printDebug(to_string(cnt));
            if(cnt>=10)
            {
                Sequence::broadcast.broadcast("break");
            }
        }),
        make_shared<block::Delay>(0.3)
    ))
    );
    sequence.compile(true);
    sequence.start();


    ros::Rate loopRate(100);
    while (ros::ok())
    {
        Sequence::spinOnce();
        ros::spinOnce();
        loopRate.sleep();
    }
}