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
	while (ros::ok ())
	{
		/*
		* 	room density
		* 	0 = empty
		* 	10 = crowded
		*/
		double density = 0;

		/*
		* 	room vulnerability
		* 	0 = sociable
		* 	10 = serious
		*/
		double vulnerability = 0;

		point goal;
		bool goalReached = false;

		/*
		* 	set a goal based on density and
		* 	vulnerability then attempt to go
		* 	there
		*/
		goal = san::findBehaviorGoal (san::readRoom (density, vulnerability));
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
		loopRate.sleep();
	}

	return 0;
}
