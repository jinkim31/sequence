#include <iostream>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "../inc/sequence_ros.h"

using namespace std;
using namespace seq;

int main(int argc, char **argv)
{
    double currentTime = 0;

    ros::init(argc, argv, "sequence_test_node");
    ros::NodeHandle nh;
    Sequence sequence

    (
        new block::Delay(1),
        new block::Delay(1),
        new block::Delay(1),
        new block::Delay(1),
        new block::Delay(1)
        );

    sequence.compile(true);
    sequence.start();

    ros::Rate loopRate(10);
    while (ros::ok())
    {
        Sequence::spinOnce(0.1);
        ros::spinOnce();
        loopRate.sleep();
    }
}