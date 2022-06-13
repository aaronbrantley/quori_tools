#include "DirectionListener.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

int main (int argc, char ** argv)
{
  // classes
  DirectionListener soundDirection;

  // ros
  ros::init (argc, argv, "turn_to_sound");
  ros::NodeHandle turnToSoundNode;
  ros::Rate loopRate (1);

  // actionlib
  actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> actionClient ("move_base", true);

  // move_base_msgs
  move_base_msgs::MoveBaseGoal goal;

  // primitive
  int currentSoundDirection;
  double rotation;
  int subtract = 0;

  while (turnToSoundNode.ok ())
  {
    currentSoundDirection = soundDirection.getSoundDirection ();

    // if a new sound direction is published
    if (currentSoundDirection - subtract != 0)
    {
      // convert degrees to radians
      rotation = currentSoundDirection * 3.14 / 180;

      ROS_INFO_STREAM ("turning " << rotation << " rads");

      // goal header
      goal.target_pose.header.frame_id = "respeaker_base";
      goal.target_pose.header.stamp = ros::Time::now ();

      // goal position
      goal.target_pose.pose.position.x = 0;
      goal.target_pose.pose.position.y = 0;
      goal.target_pose.pose.position.z = 0;

      // goal orientation
      goal.target_pose.pose.orientation.x = 0;
      goal.target_pose.pose.orientation.y = 0;
      goal.target_pose.pose.orientation.w = rotation;

      // send goal
      actionClient.sendGoal (goal);
      actionClient.waitForResult ();

      subtract = currentSoundDirection;
    }

    ros::spinOnce ();
    loopRate.sleep ();
  }

  return 0;
}
