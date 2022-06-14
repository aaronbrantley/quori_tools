#include "DirectionListener.h"

#include <geometry_msgs/Twist.h>

void smoothTurn (int degrees, int scale);

int main (int argc, char ** argv)
{
  // classes
  DirectionListener soundDirection;

  // ros
  ros::init (argc, argv, "turn_to_sound");
  ros::NodeHandle waitForNewSoundNode;
  ros::Rate loopRate (1);

  // primitive
  int currentSoundDirection;
  int difference = 0;

  while (waitForNewSoundNode.ok ())
  {
    currentSoundDirection = soundDirection.getSoundDirection ();

    // if a new sound direction is found
    if (currentSoundDirection - difference != 0)
    {
      smoothTurn (currentSoundDirection, 10);

      difference = currentSoundDirection;
    }

    ros::spinOnce ();
    loopRate.sleep ();
  }

  return 0;
}

void smoothTurn (int degrees, int scale)
{
  // ros
  ros::NodeHandle turnToSoundNode;
  ros::Rate loopRate (5);
  ros::Publisher turnPublisher = turnToSoundNode.advertise <geometry_msgs::Twist> ("quori/base_controller/cmd_vel", 1);

  // geometry_msgs
  geometry_msgs::Twist rotationMessage;

  // primitive
  double rotation;

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

  for (int iterations = 0; iterations < scale * 5; iterations += 1)
  {
    turnPublisher.publish (rotationMessage);

    ROS_INFO_STREAM ("rotation message published");

    ros::spinOnce ();
    loopRate.sleep ();
  }
}
