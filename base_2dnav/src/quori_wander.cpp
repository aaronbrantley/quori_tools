#include "Listeners.h"
#include "Behaviors.h"

int main (int argc, char ** argv)
{
	// classes
	PoseListener currentPose;
	Engaging engagingBehavior;

	// ros
	ros::init (argc, argv, "quori_wander");
	ros::NodeHandle wanderNode;
	ros::Rate loopRate (1);

	/*
	* 	loop until ctrl-c is pressed or
	* 	ros::shutdown is called
	*/
	while (wanderNode.ok ())
	{
		bool goalReached = false;

		std::vector <double> goal = engagingBehavior.findGoal (currentPose.getPose ());

		/*
		* 	engaging behavior with no person
		* 	detection causes wandering
		*/
		goalReached = engagingBehavior.goToGoal (goal);

		if (goalReached)
		{
			ROS_INFO_STREAM ("behavior finished successfully\n");
		}
		else
		{
			ROS_WARN_STREAM ("behavior not successful\n");
		}

		ros::spinOnce ();
		loopRate.sleep ();
	}

	return 0;
}
