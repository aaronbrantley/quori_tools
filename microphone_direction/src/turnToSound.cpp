#include "DirectionListener.h"

#include <geometry_msgs/Twist.h>

int main (int argc, char ** argv)
{
  // classes
  DirectionListener soundDirection;

  // ros
  ros::init (argc, argv, "turn_to_sound");
  ros::NodeHandle turnToSoundNode;
  ros::Publisher turnPublisher = turnToSoundNode.advertise <geometry_msgs::Twist> ("quori/base_controller/cmd_vel", 1);
  ros::Rate loopRate (1);

  //geometry_msgs
  geometry_msgs::Twist rotationMessage;

  // primitive
  int currentSoundDirection;
  double rotation;
  int subtract = 0;

  while (turnToSoundNode.ok ())
  {
    currentSoundDirection = soundDirection.getSoundDirection ();

    // if a new sound direction is found
    if (currentSoundDirection - subtract != 0)
    {
      // convert degrees to radians
      rotation = currentSoundDirection * 3.14 / 180;

      ROS_INFO_STREAM ("turning " << rotation << " rads");

      rotationMessage.linear.x = 0;
      rotationMessage.linear.y = 0;
      rotationMessage.linear.z = 0;

      rotationMessage.angular.x = 0;
      rotationMessage.angular.y = 0;
      rotationMessage.angular.z = rotation;

      turnPublisher.publish (rotationMessage);

      subtract = currentSoundDirection;
    }

    ros::spinOnce ();
    loopRate.sleep ();
  }

  return 0;
}
