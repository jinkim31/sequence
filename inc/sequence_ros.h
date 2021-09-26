#ifndef _SEQUENCE_ROS_H
#define _SEQUENCE_ROS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/Twist.h>
#include "sequence_core.h"

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

    virtual Block *doClone()
    {
      return new Publish(msg, topic, queueSize);
    }

protected:
    T *msg;
    T allocatorMsg;
    ros::Publisher publisher;

    Publish(string topic, int queueSize)
    {
      ros::NodeHandle nh;
      publisher = nh.advertise<T>(topic, queueSize);
      this->topic = topic;
      this->topic = topic;
      this->queueSize = queueSize;
      msg = &allocatorMsg;
    }

public:
    Publish(T *msg, string topic, int queueSize) : Publish(topic, queueSize)
    {
      this->msg = msg;
    }

    virtual void startCallback() {}

    virtual bool update(SpinInfo spinInfo)
    {
      if (publisher.getNumSubscribers() == 0)
        ROS_WARN(
                "[Sequence] Publisher(%s %s) is not being subscribed by anything. \nIf it is the first block of the sequence, you might want to give some delay before it so that other nodes can subscribe.",
                typeid(T).name(), topic.c_str());
      publisher.publish(*msg);
      return true;
    }

    virtual void reset() {}
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
      if (userCallback(msg)) {
        done = true;
      }
    }

    virtual Block *doClone()
    {
      return new Subscribe<T>(topic, queueSize, userCallback);
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

    virtual bool update(SpinInfo spinInfo)
    {
      //if(subscriber.getNumPublishers()==0) ROS_WARN("[Sequence] Subscriber(%s %s) is not subscribing any publisher.\n", typeid(T).name(), topic.c_str());
      ros::spinOnce();
      return done;
    }

    virtual void reset()
    {
      ros::NodeHandle nh;
      //'Disable' subscriber by subscribing to topic+"_DISABLED_".
      subscriber = nh.subscribe(topic + "_DISABLED_", queueSize, &Subscribe::callback, this);
      done = false;
    }
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
};

class Twist : public Publish<geometry_msgs::Twist>
{
public:
    Twist(double linear, double angular) : Publish("cmd_vel", 1)
    {
      msg->linear.x = linear;
      msg->angular.z = angular;
    }
};

class WaitForNavReach : public Subscribe<actionlib_msgs::GoalStatusArray>
{
public:
    WaitForNavReach() : Subscribe("move_base/status", 1)
    {
      setUserCallback([&](const actionlib_msgs::GoalStatusArray &msg) {
          return (!msg.status_list.empty() && msg.status_list.back().status == 3);
      });
    }
};

class WaitForNavStart : public Subscribe<actionlib_msgs::GoalStatusArray>
{
public:
    WaitForNavStart() : Subscribe("move_base/status", 1)
    {
      setUserCallback([&](const actionlib_msgs::GoalStatusArray &msg) {
          return (!msg.status_list.empty() && msg.status_list.back().status == 1);
      });
    }
};

class WaitForNavReachLatched : public block::SequenceBlock
{
public:
    WaitForNavReachLatched() : block::SequenceBlock(new Sequence
                                                            (
                                                                    new WaitForNavStart,
                                                                    new WaitForNavReach
                                                            )) {}
};

}

#endif