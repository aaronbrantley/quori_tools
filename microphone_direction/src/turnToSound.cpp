#include "DirectionListener.h"

#include <geometry_msgs/Twist.h>

void smoothTurn (int degrees, double seconds);

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
    // i dont know if sound_direction gives the right angle but i will trust it
    currentSoundDirection = soundDirection.getSoundDirection ();

    // if a new sound direction is found (unlikely that two sounds will come from the exact same direction)
    if (currentSoundDirection != previousSoundDirection)
    {
      smoothTurn (currentSoundDirection, 4);

      previousSoundDirection = currentSoundDirection;
    }

    ros::spinOnce ();
    loopRate.sleep ();
  }

  return 0;
}

void smoothTurn (int degrees, double seconds)
{
  // sound_direction never gave a value outside of this range, but just in case
  if (degrees < -180)
  {
    ROS_INFO_STREAM ("clamping turn to -180 degrees");

    degrees = -180;
  }
  else if (degrees > 180)
  {
    ROS_INFO_STREAM ("clamping turn to 180 degrees");

    degrees = 180;
  }

  // primitive
  double rotation = degrees * 3.1415926 / 180;
  int rate = 10;

  // ros
  ros::NodeHandle turnToSoundNode;
  ros::Rate loopRate (rate);
  ros::Publisher turnPublisher = turnToSoundNode.advertise <geometry_msgs::Twist> ("quori/base_controller/cmd_vel", 1);

  // geometry_msgs
  geometry_msgs::Twist rotationMessage;

  ROS_INFO_STREAM ("turning " << degrees << " degrees at " << degrees / seconds << " degrees/s...");

  // setup cmd_vel message
  rotationMessage.linear.x = 0;
  rotationMessage.linear.y = 0;
  rotationMessage.linear.z = 0;
  rotationMessage.angular.x = 0;
  rotationMessage.angular.y = 0;
  // angular velocity
  rotationMessage.angular.z = rotation / seconds;

  // keep track of rotation progress
  double rotationTracker = 0;

  for (int iterations = 0; iterations < rate * seconds; iterations += 1)
  {
    // send cmd_vel message
    turnPublisher.publish (rotationMessage);

    // condition is true once every second while turning
    if (iterations % rate == 0)
    {
      rotationTracker += degrees / seconds;

      ROS_INFO_STREAM (rotationTracker << " degrees rotated");
    }


    ros::spinOnce ();
    loopRate.sleep ();
  }
}
