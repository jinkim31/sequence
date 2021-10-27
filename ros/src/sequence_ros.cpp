
#include "../inc/sequence_ros.h"

bool block_ros::TerminalCommand::update(SpinInfo spinInfo)
{
    system(command.c_str());
    cout<<"asdf"<<endl;
    return true;
}

void block_ros::TerminalCommand::reset()
{

}

block_ros::TerminalCommand::TerminalCommand(string command) : command(command)
{

}
