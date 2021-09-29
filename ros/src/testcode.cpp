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
        new block::LoopSequence(new BroadcastCondition("Stop"), 3,new Sequence
        (
            new block::Delay(10000)
        )),
        new block::Debug("stopped")
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
        //if(loopCnt++%100 == 30) Sequence::broadcast("Stop");
        Sequence::spinOnce();
        ros::spinOnce();
        loopRate.sleep();
    }
}