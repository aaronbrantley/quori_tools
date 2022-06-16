#include "DirectionListener.h"

#include <geometry_msgs/Twist.h>

void smoothTurn (int degrees, double time);

int main (int argc, char ** argv)
{
  // classes
  DirectionListener soundDirection;

  // primitive
  int currentSoundDirection;
  int previousSoundDirection = 0;

  // ros
  ros::init (argc, argv, "turn_to_sound");
  ros::NodeHandle waitForNewSoundNode;
  ros::Rate loopRate (1);

  while (waitForNewSoundNode.ok ())
  {
    currentSoundDirection = soundDirection.getSoundDirection ();

    // if a new sound direction is found (and more than 15 degrees different from the last sound)
    if (std::abs (currentSoundDirection - previousSoundDirection) > 15)
    {
      // 0 degrees is to the right of the robot and mic installation is a bit off center, shift by about 90 degrees for correct turning
      smoothTurn (currentSoundDirection + 95, 4);

      previousSoundDirection = currentSoundDirection;
    }

    ros::spinOnce ();
    loopRate.sleep ();
  }

  return 0;
}

void smoothTurn (int degrees, double time)
{
  // primitive
  double rotation = degrees * 3.1415926 / 180;;
  int rate = 5;

  // ros
  ros::NodeHandle turnToSoundNode;
  ros::Rate loopRate (rate);
  ros::Publisher turnPublisher = turnToSoundNode.advertise <geometry_msgs::Twist> ("quori/base_controller/cmd_vel", 1);

  // geometry_msgs
  geometry_msgs::Twist rotationMessage;

  ROS_INFO_STREAM ("turning " << rotation << " rads at " << rotation / time << " rads/s...");

  // setup cmd_vel message
  rotationMessage.linear.x = 0;
  rotationMessage.linear.y = 0;
  rotationMessage.linear.z = 0;
  rotationMessage.angular.x = 0;
  rotationMessage.angular.y = 0;
  rotationMessage.angular.z = rotation / time;

  double rotationTracker = 0;

  for (int iterations = 0; iterations < rate * time; iterations += 1)
  {
    turnPublisher.publish (rotationMessage);

    if (iterations % 5 == 0)
    {
      rotationTracker += rotationMessage.angular.z;

      ROS_INFO_STREAM (rotationTracker << " rads rotated");
    }


    ros::spinOnce ();
    loopRate.sleep ();
  }
}
