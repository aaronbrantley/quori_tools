#include "Behaviors.h"
#include "san.h"

/*
* 	return a letter given its position
* 	in the alphabet
*/
char getRandomBehavior ()
{
	// all valid behaviors
	std::string behaviors = "ecrs";

	// random number generation
	std::srand (std::time (nullptr));

	// random value between 0 and behavior list size - 1
	int randomNumber = std::rand () % behaviors.size ();

	return behaviors.at (randomNumber);
}

int main (int argc, char ** argv)
{
	// classes
	Behaviors behavior;

	// initialize node
	ros::init (argc, argv, "quori_san");
	ros::NodeHandle sanNode;
	ros::Rate loopRate (1);

	/*
	* 	spinOnce has to be called twice
	* 	before getting correct pose,
	* 	there's probably a better way to do
	* 	this
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
		point goal;
		bool goalReached = false;

		/*
		* 	set a goal based on random
		* 	selection of a behavior
		*/
		goal = san::findBehaviorGoal (getRandomBehavior ());
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
