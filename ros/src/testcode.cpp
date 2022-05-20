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
        make_shared<block::Debug>("asd")
    );
    sequence.compile(true);
    sequence.start();


    ros::Rate loopRate(100);
    while (ros::ok())
    {
        Sequence::spinOnce();
        ros::spinOnce();
        loopRate.sleep();
        //if(sequence.isFinished()) return 0;
    }
}