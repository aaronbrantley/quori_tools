#ifndef DIRECTION_LISTENER_H_
#define DIRECTION_LISTENER_H_

#include <ros/ros.h>

#include <std_msgs/Int32.h>

// class for getting direction from microphone input
class DirectionListener
{
  private:
    // ros
    ros::Subscriber directionSub;

    // std
    std::string nodeName;

    // primitive
    int direction;

  protected:
    void initializeRos (std::string name)
    {
      int argc = 0;
      char ** argv = nullptr;

      ros::init (argc, argv, name);

      ros::NodeHandle listenNode;

      directionSub = listenNode.subscribe ("/sound_direction", 1, & DirectionListener::directionCallback, this);

      ros::spinOnce ();
    }

    void directionCallback (const std_msgs::Int32::ConstPtr & directionMessage)
    {
      direction = directionMessage -> data;
    }

  public:
    double getSoundDirection ()
    {
      nodeName = "direction_listener";

      initializeRos (nodeName);

      std::cout << "direction of sound: " << direction << std::endl;

      return direction;
    }
};

#endif
