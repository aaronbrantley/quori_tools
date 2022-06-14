#include "DirectionListener.h"

#include <geometry_msgs/Twist.h>

void smoothTurn (int degrees, int scale);

int main (int argc, char ** argv)
{
  // classes
  DirectionListener soundDirection;

  // primitive
  int currentSoundDirection;

  // ros
  ros::init (argc, argv, "turn_to_sound");
  ros::NodeHandle waitForNewSoundNode;
  ros::Rate loopRate (1);

  while (waitForNewSoundNode.ok ())
  {
    currentSoundDirection = soundDirection.getSoundDirection ();

    // if a new sound direction is found
    if (currentSoundDirection != 0)
    {
      smoothTurn (currentSoundDirection, currentSoundDirection / 10);
    }

    ros::spinOnce ();
    loopRate.sleep ();
  }

  return 0;
}

void smoothTurn (int degrees, int scale)
{
  // primitive
  double rotation;
  int rate = 5;

  // ros
  ros::NodeHandle turnToSoundNode;
  ros::Rate loopRate (rate);
  ros::Publisher turnPublisher = turnToSoundNode.advertise <geometry_msgs::Twist> ("quori/base_controller/cmd_vel", 1);

  // geometry_msgs
  geometry_msgs::Twist rotationMessage;

  // convert degrees to radians
  rotation = degrees * 3.14 / 180;

  ROS_INFO_STREAM ("turning " << rotation << " rads at " << rotation / scale << " rads/s");

  // setup cmd_vel message
  rotationMessage.linear.x = 0;
  rotationMessage.linear.y = 0;
  rotationMessage.linear.z = 0;
  rotationMessage.angular.x = 0;
  rotationMessage.angular.y = 0;
  rotationMessage.angular.z = rotation / scale;

  for (int iterations = 0; iterations < scale * rate; iterations += 1)
  {
    turnPublisher.publish (rotationMessage);

    ROS_DEBUG_STREAM ("rotation message published");

    ros::spinOnce ();
    loopRate.sleep ();
  }
}
