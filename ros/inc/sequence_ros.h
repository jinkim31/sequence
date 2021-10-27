#ifndef _SEQUENCE_ROS_H
#define _SEQUENCE_ROS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/Twist.h>
#include "../../inc/sequence_core.h"
#include "../../inc/sequence.h"

using namespace std;
using namespace seq;

namespace block_ros
{
template<typename T>
class Publish : public Block
{
private:
    string topic;
    int queueSize;

protected:
    T *msg;
    T allocatorMsg;
    ros::Publisher publisher;

    Publish(string topic, int queueSize) : topic(topic), queueSize(queueSize)
    {
        ros::NodeHandle nh;
        publisher = nh.advertise<T>(topic, queueSize);
        msg = &allocatorMsg;
    }

public:
    Publish(T *msg, string topic, int queueSize) : msg(msg), topic(topic), queueSize(queueSize)
    {
        ros::NodeHandle nh;
        publisher = nh.advertise<T>(topic, queueSize);
    }

    virtual void startCallback()
    {}

    virtual bool update(SpinInfo spinInfo)
    {
        if (publisher.getNumSubscribers() == 0)
            ROS_WARN(
            "[Sequence] Publisher(%s %s) is not being subscribed by anything. \nIf it is the first block of the sequence, you might want to give some delay before it so that other nodes can subscribe.",
            typeid(T).name(), topic.c_str());
        publisher.publish(*msg);
        return true;
    }

    virtual void reset()
    {}

    virtual string generateDebugName()
    {
        return "Publish(" + string(typeid(T).name()) + ")";
    }
};

template<typename T>
class Subscribe : public Block
{
private:
    bool done;
    ros::Subscriber subscriber;
    function<bool(const T &)> userCallback;
    string topic;
    int queueSize;

    void callback(const T &msg)
    {
        if (userCallback(msg))
        {
            done = true;
        }
    }

protected:
    Subscribe(string topic, int queueSize)
    {
        done = false;
        this->topic = topic;
        this->queueSize = queueSize;
    }

    void setUserCallback(function<bool(const T &)> userCallback)
    {
        this->userCallback = userCallback;
    }

public:

    Subscribe(string topic, int queueSize, function<bool(const T &)> userCallback) : Subscribe(topic, queueSize)
    {
        this->userCallback = userCallback;
    }

    virtual void startCallback()
    {
        ros::NodeHandle nh;
        subscriber = nh.subscribe(topic, queueSize, &Subscribe::callback, this);

    }

    virtual void endCallback()
    {
        ros::NodeHandle nh;
        //disable subscriber by subscribing to topic+"_DISABLED_".
        subscriber = nh.subscribe(topic + "_DISABLED_", queueSize, &Subscribe::callback, this);
    }

    virtual bool update(SpinInfo spinInfo)
    {
        //if(subscriber.getNumPublishers()==0) ROS_WARN("[Sequence] Subscriber(%s %s) is not subscribing any publisher.\n", typeid(T).name(), topic.c_str());
        ros::spinOnce();
        return done;
    }

    virtual void reset()
    {
        done = false;
    }

    virtual string generateDebugName()
    {
        return "Subscribe(" + string(typeid(T).name()) + ")";
    }
};

template<typename T>
class ServiceCall : public Block
{
private:
    string serviceName;
    T *srv;
    T allocatorSrv;
protected:
    ros::ServiceClient client;
public:
    ServiceCall(T *srv, string serviceName) : srv(srv), serviceName(serviceName)
    {
        ros::NodeHandle nh;
        client = nh.serviceClient<T>(serviceName);
    }

    ServiceCall(string serviceName) : serviceName(serviceName)
    {
        srv = &allocatorSrv;
        ros::NodeHandle nh;
        client = nh.serviceClient<T>(serviceName);
    }

    virtual bool update(SpinInfo spinInfo)
    {
        client.template call(*srv);
        return true;
    }

    void virtual reset()
    {

    }

    virtual string generateDebugName()
    {
        return "ServiceCall(" + string(typeid(T).name()) + ")";
    }
};

class TerminalCommand : public Block
{
private:
    string command;
public:
    TerminalCommand(string command);
    virtual bool update(SpinInfo spinInfo);

    virtual void reset();
};

class NavGoal : public Publish<geometry_msgs::PoseStamped>
{
public:
    NavGoal(double posX, double posY, double posZ, double oriX, double oriY, double oriZ, double oriW) : Publish(
    "move_base_simple/goal", 1)
    {
        msg->header.frame_id = "map";
        msg->pose.position.x = posX;
        msg->pose.position.y = posY;
        msg->pose.position.z = posZ;
        msg->pose.orientation.x = oriX;
        msg->pose.orientation.y = oriY;
        msg->pose.orientation.z = oriZ;
        msg->pose.orientation.w = oriW;
    }

    virtual string generateDebugName()
    {
        return "NavGoal([" +
               to_string(msg->pose.position.x) + "," +
               to_string(msg->pose.position.y) + "," +
               to_string(msg->pose.position.z) + "] [" +
               to_string(msg->pose.orientation.x) + "," +
               to_string(msg->pose.orientation.y) + "," +
               to_string(msg->pose.orientation.z) + "," +
               to_string(msg->pose.orientation.w) + "])";
    }
};

class Twist : public Publish<geometry_msgs::Twist>
{
public:
    Twist(double linear, double angular) : Publish("cmd_vel", 1)
    {
        msg->linear.x = linear;
        msg->angular.z = angular;
    }

    virtual string generateDebugName()
    {
        return "Twist(" + to_string(msg->linear.x) + "," + to_string(msg->angular.z) + ")";
    }
};

class WaitForNavReach : public Subscribe<actionlib_msgs::GoalStatusArray>
{
public:
    WaitForNavReach() : Subscribe("move_base/status", 1)
    {
        setUserCallback([&](const actionlib_msgs::GoalStatusArray &msg)
        {
            if (!msg.status_list.empty() && msg.status_list.back().status == 3)
            {
                Sequence::printDebug("status==3");
                return true;
            }
            return false;
        });
    }

    virtual string generateDebugName()
    {
        return "WaitForNavReach";
    }
};

class WaitForNavStart : public Subscribe<actionlib_msgs::GoalStatusArray>
{
public:
    WaitForNavStart() : Subscribe("move_base/status", 1)
    {
        setUserCallback([&](const actionlib_msgs::GoalStatusArray &msg)
        {
            if (!msg.status_list.empty() && msg.status_list.back().status == 1)
            {
                Sequence::printDebug("status==1");
                return true;
            }
            return false;
        });
    }

    virtual string generateDebugName()
    {
        return "WaitForNavStart";
    }
};

class WaitForNavReachLatched : public block::SequenceBlock
{
public:
    WaitForNavReachLatched() : block::SequenceBlock(new Sequence
    (
    new WaitForNavStart,
    new WaitForNavReach
    ))
    {}

    virtual string generateDebugName()
    {
        return "WaitForNavReachLatched";
    }
};

}

#endif