#include <iostream>
#include <ros/ros.h>
#include "../inc/sequence_ros.h"

using namespace std;
using namespace seq;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sequence_test_node");
    ros::NodeHandle nh;

    Sequence sequence;
    sequence.addVariable<int>("i",0);
    sequence.compose
    (
        "Main Sequence",
        make_shared<block::Debug>("Inner sequence start"),
        make_shared<block::LoopSequence>(make_shared<LambdaCondition>([&]
        {return Sequence::getVariable<int>("i")>2;}), make_shared<Sequence>
        (
            make_shared<block::Function>([&]
            {
                int& i = Sequence::getVariable<int>("i");
                Sequence::printDebug("i="+to_string(i));
                i++;
            })
        )),
        make_shared<block::Debug>("Inner sequence end")
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