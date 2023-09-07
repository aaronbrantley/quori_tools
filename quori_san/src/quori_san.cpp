#include "Behaviors.h"
#include "san.h"

int main (int argc, char ** argv)
{
	Behaviors behavior;

	ros::init (argc, argv, "quori_san");
	ros::NodeHandle sanNode;
	ros::Rate loopRate (1);

	/*
	* 	spinOnce has to be called twice before getting correct pose
	* 	there's probably a better way to do this
	*/
	ros::spinOnce ();
	loopRate.sleep ();
	ros::spinOnce ();

	/*
	* 	loop until ctrl-c is pressed or
	* 	ros::shutdown is called
	*/
	while (sanNode.ok ())
	{
		voting::Point goal;
		bool goalReached = false;

		/*
		* 	set a goal based on density and
		* 	vulnerability then attempt to go
		* 	there
		*/
		goal = san::findBehaviorGoal ();
		goalReached = behavior.goToGoal ();

		if (goalReached)
		{
			ROS_INFO ("Behavior finished successfully");
		}

		else
		{
			ROS_WARN ("Behavior not successful");
		}

		ros::spinOnce ();
		loopRate.sleep ();
	}

	return 0;
}
