#include <iostream>
#include <ros/ros.h>
#include "../inc/sequence_ros.h"

using namespace std;
using namespace seq;

int main(int argc, char **argv)
{
    int loopCnt = 0;

    ros::init(argc, argv, "sequence_test_node");
    ros::NodeHandle nh;

    Sequence sequence
    (
        "main",
        new block::LoopSequence(new Sequence
        (
            new block::WaitForBroadcast("Hi"),
            new block::Debug("R")
        ), []{return false;})
    );

    sequence.compile(true);
    sequence.start();

    Sequence broadcaster
    (
        "broadcaster",
        new block::Broadcast("Hi")
    );

    broadcaster.compile();

    ros::Rate loopRate(10);

    while (ros::ok())
    {
        if(loopCnt++%10 == 0 && !broadcaster.isRunning()) broadcaster.start();
        Sequence::spinOnce();
        ros::spinOnce();
        loopRate.sleep();
    }
}