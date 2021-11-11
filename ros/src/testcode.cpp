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
    sequence.addVariable(make_shared<Variable<int>>("cnt",0));
    sequence.compose
    (
        "Main Sequence",
        make_shared<block::Debug>("Inner sequence start"),
        make_shared<block::SequenceBlock>(make_shared<Sequence>
        (
            "Inner Sequence",
            make_shared<block::Debug>("Sequence in a sequence"),
            make_shared<block::Delay>(1.0)
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