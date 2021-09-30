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
    new block::Debug("h"),
    new block::Delay(1)
    );
    sequence.compile(true);
    sequence.start();

    ros::Rate loopRate(100);
    while (ros::ok())
    {
        Sequence::spinOnce();
        ros::spinOnce();
        loopRate.sleep();

        if(Sequence::getSequenceByName("main")->isFinished()) Sequence::getSequenceByName("main")->start();
    }
}