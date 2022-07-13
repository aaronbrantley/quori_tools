#include "Behaviors.h"
#include "Listeners.h"

int main (int argc, char ** argv)
{
  // classes
  PoseListener currentPose;
  Engaging engagingBehavior;

  // ros
  ros::init (argc, argv, "quori_wander");
  ros::NodeHandle wanderNode;
  ros::Rate loopRate (1);

  // loop until ctrl-c is pressed or ros::shutdown is called
  while (wanderNode.ok ())
  {
    // whether or not the goal has been reached
    bool goalReached = false;

    std::vector <std::vector <double>> emptyList;

    std::vector <double> goal = engagingBehavior.findGoal (currentPose.getPose (), emptyList);

    // engaging behavior with no person detection causes wandering
    goalReached = engagingBehavior.goToGoal (goal);

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

  return 0;
}
