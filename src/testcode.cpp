#include <iostream>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "../inc/sequence_ros.h"
#include "../inc/sequence_embedded.h"

using namespace std;
using namespace seq;

int main(int argc, char **argv)
{
    double currentTime = 0;

    cout<<"0"<<endl;
    ros::init(argc, argv, "sequence_test_node");
    ros::NodeHandle nh;
    Sequence sequence
        (
            //new block_ros::ServiceCall<std_srvs::Empty>("move_base/clear_costmaps")
            new block::Delay(1),
            new block::Delay(2),
            new block::Delay(3)
        );

    sequence.compile(true);
    sequence.start();

    EmbeddedChronometer chronometer(&currentTime);
    Sequence::setChronometer(chronometer);

    ros::Rate loopRate(10);
    while (ros::ok())
    {
        currentTime+=0.01;
        Sequence::spinOnce();
        ros::spinOnce();
        loopRate.sleep();
    }
}