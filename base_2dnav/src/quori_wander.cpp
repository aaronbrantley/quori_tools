#include "Behaviors.h"

int main (int argc, char ** argv)
{
  // initialize node
  ros::init (argc, argv, "quori_wander");
  //ROS_DEBUG ("initialized node quori_wander");

  // declare listeners
  // PoseListener subscribes to amcl_pose topic
  PoseListener currentPose;

  // behavior handling
  Behaviors behavior;
  Engaging engagingBehavior;

  ros::Rate loopRate (1);
  ros::spinOnce ();
  loopRate.sleep ();
  // spinOnce has to be called twice before getting correct pose, there's probably a better way to do this
  ros::spinOnce ();

  // loop until ctrl-c is pressed or ros::shutdown is called
  while (ros::ok ())
  {
    // whether or not the goal has been reached
    bool goalReached = false;

    std::vector <std::vector <double>> emptyList;

    // engaging behavior with no person detection causes wandering
    goalReached = behavior.goToGoal (engagingBehavior.findGoal (currentPose.getPose (), emptyList));

    if (goalReached)
    {
      ROS_INFO ("behavior finished successfully\n");
    }
    else
    {
      ROS_WARN ("behavior not successful\n");
    }

    // get new information from subscriptions
    ros::spinOnce ();
    loopRate.sleep ();
  }
}
