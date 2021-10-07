#include <iostream>
#include <ros/ros.h>
#include "../inc/sequence_ros.h"

using namespace std;
using namespace seq;

class TestSequence :public block::SequenceBlock
{
public:
    TestSequence() : SequenceBlock(new Sequence
    (
        "test sequence",
        new block::Debug("1"),
        new block::Debug("2")
    ))
    {

    }
};

int main(int argc, char **argv)
{
    int loopCnt = 0;

    ros::init(argc, argv, "sequence_test_node");
    ros::NodeHandle nh;

    Sequence sequence
    (
    "main",
    new TestSequence(),
    new TestSequence(),
    new TestSequence
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