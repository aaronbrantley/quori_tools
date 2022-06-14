#include "DirectionListener.h"

#include <geometry_msgs/Twist.h>

void smoothTurn (int degrees, double scale);

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
    if (currentSoundDirection < -30 || currentSoundDirection > 30)
    {
      smoothTurn (currentSoundDirection, 540 / currentSoundDirection);
    }

    ros::spinOnce ();
    loopRate.sleep ();
  }

  return 0;
}

void smoothTurn (int degrees, double scale)
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
  rotation = degrees * 3.1415926 / 180;

  ROS_INFO_STREAM ("turning " << rotation << " rads at " << rotation / scale << " rads/s");

  // setup cmd_vel message
  rotationMessage.linear.x = 0;
  rotationMessage.linear.y = 0;
  rotationMessage.linear.z = 0;
  rotationMessage.angular.x = 0;
  rotationMessage.angular.y = 0;
  rotationMessage.angular.z = rotation / scale;

  double rotationTracker = 0;

  for (int iterations = 0; iterations < rate * scale; iterations += 1)
  {
    turnPublisher.publish (rotationMessage);

    if (iterations % 5 == 0)
    {
      rotationTracker += rotationMessage.angular.z;

      ROS_INFO_STREAM ("rotation in progress... " << rotationTracker << " rads rotated total");
    }


    ros::spinOnce ();
    loopRate.sleep ();
  }
}
