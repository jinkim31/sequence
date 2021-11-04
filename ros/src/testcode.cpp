#include <iostream>
#include <ros/ros.h>
#include "../inc/sequence_ros.h"

using namespace std;
using namespace seq;

class Var
{
public:
    int a;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sequence_test_node");
    ros::NodeHandle nh;

    Sequence sequence(new block::Delay(1.0));
    shared_ptr<Variable<int>> intVar = make_shared<Variable<int>>("testvar",123);
    sequence.addVariable(intVar);
    cout<<"value:"<<sequence.getVariable<int>("testvar")->get()<<endl;
    sequence.getVariable<int>("testvar")->set(456);
    cout<<"value:"<<sequence.getVariable<int>("testvar")->get()<<endl;

    ros::Rate loopRate(100);
    while (ros::ok())
    {
        Sequence::spinOnce();
        ros::spinOnce();
        loopRate.sleep();
    }
}